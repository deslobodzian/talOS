#include "talOS/vision/metal/gpu_sparse_clusters.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <vector>

namespace {

// Number of pixels a set must contain to seed a cluster (matches upstream).
constexpr uint32_t kMinSetPixels = 25;

struct Entry {
  uint64_t id;
  zarray_t* cluster;
  Entry* next;
};

// Always clears touched slots in thread_local scratch buffers before mem_pools
// is destroyed, preventing dangling Entry* pointers (UAF) or leaked clusters if
// any allocation throws.
struct ThreadLocalScratchGuard {
  std::vector<std::vector<uint32_t>>& row_candidates;
  std::vector<uint32_t>& touched_rows;
  std::vector<Entry*>& clustermap;
  std::vector<uint32_t>& touched_buckets;
  bool destroy_unreturned_clusters{true};

  ~ThreadLocalScratchGuard() noexcept {
    for (uint32_t b : touched_buckets) {
      if (b < clustermap.size()) {
        if (destroy_unreturned_clusters) {
          for (Entry* e = clustermap[b]; e != nullptr; e = e->next) {
            if (e->cluster != nullptr) {
              zarray_destroy(e->cluster);
              e->cluster = nullptr;
            }
          }
        }
        clustermap[b] = nullptr;
      }
    }
    touched_buckets.clear();
    for (uint32_t y : touched_rows) {
      if (y < row_candidates.size()) {
        row_candidates[y].clear();
      }
    }
    touched_rows.clear();
  }
};

inline uint32_t RepresentativeOf(const uint32_t* parents, uint32_t total,
                                 uint32_t id) {
  if (id >= total) return 0xFFFFFFFFu;
  const uint32_t rep = parents[id];
  if (rep >= total) return 0xFFFFFFFFu;
  return rep;
}

}  // namespace

extern "C" zarray_t* BuildSparseClustersFromGpuLabels(
    const image_u8_t* threshim, int w, int h, int ts,
    const uint32_t* gpu_parents, const uint32_t* gpu_sizes,
    const uint32_t* candidate_coords, uint32_t candidate_count) {
  zarray_t* clusters = zarray_create(sizeof(zarray_t*));
  if (clusters == nullptr) {
    return nullptr;
  }
  if (threshim == nullptr || threshim->buf == nullptr ||
      gpu_parents == nullptr || gpu_sizes == nullptr || w <= 0 || h <= 0 ||
      ts < w) {
    return clusters;
  }
  const uint64_t total64 = static_cast<uint64_t>(w) * static_cast<uint64_t>(h);
  if (total64 > UINT32_MAX) {
    return clusters;
  }
  const uint32_t total = static_cast<uint32_t>(total64);
  const size_t width = static_cast<size_t>(w);
  const size_t height = static_cast<size_t>(h);
  const size_t stride = static_cast<size_t>(ts);

  try {
    // Organize candidates into rows for cache-friendly access. Bounds-check
    // both coordinates: corrupt GPU writes must not cause OOB access.
    // Retain high-water storage; only candidate rows are cleared each frame.
    thread_local std::vector<std::vector<uint32_t>> row_candidates;
    thread_local std::vector<uint32_t> touched_rows;
    thread_local std::vector<Entry*> clustermap;
    thread_local std::vector<uint32_t> touched_buckets;

    // mem_pools MUST be declared before scratch_guard so scratch_guard runs
    // first on exit/unwind while Entry nodes are still valid.
    std::vector<std::unique_ptr<Entry[]>> mem_pools;
    ThreadLocalScratchGuard scratch_guard{row_candidates, touched_rows,
                                          clustermap, touched_buckets};

    if (row_candidates.size() < height) {
      row_candidates.resize(height);
    }
    touched_rows.clear();
    if (candidate_coords != nullptr) {
      for (uint32_t i = 0; i < candidate_count; ++i) {
        const uint32_t entry = candidate_coords[i];
        const uint32_t x = entry & 0x3FFFu;
        const uint32_t y = (entry >> 14) & 0x3FFFu;
        if (x < static_cast<uint32_t>(w) && y < static_cast<uint32_t>(h)) {
          if (row_candidates[y].empty()) touched_rows.push_back(y);
          row_candidates[y].push_back(entry);
        }
      }
    }

    const size_t nclustermap = std::max<size_t>(
        256, static_cast<size_t>(0.2 * static_cast<double>(total)));
    if (clustermap.size() < nclustermap) {
      clustermap.resize(nclustermap, nullptr);
    }
    touched_buckets.clear();
    int pool_loc = 0;
    constexpr int kPoolSize = 2048;
    mem_pools.push_back(std::make_unique<Entry[]>(kPoolSize));

    auto u64hash = [](uint64_t v) -> uint32_t {
      v = (v ^ (v >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
      v = (v ^ (v >> 27)) * UINT64_C(0x94d049bb133111eb);
      v = v ^ (v >> 31);
      return static_cast<uint32_t>(v);
    };

    const uint8_t* buf = threshim->buf;
    std::sort(touched_rows.begin(), touched_rows.end());
    for (uint32_t row_y : touched_rows) {
      const int y = static_cast<int>(row_y);
      if (y < 1 || y >= h - 1) continue;
      auto& row = row_candidates[static_cast<size_t>(y)];
      if (row.empty()) continue;

      // Sort row candidates by x.
      std::sort(row.begin(), row.end(), [](uint32_t a, uint32_t b) {
        return (a & 0x3FFFu) < (b & 0x3FFFu);
      });

      bool connected_last = false;
      int last_x = -2;

      for (uint32_t entry : row) {
        const int x = static_cast<int>(entry & 0x3FFFu);
        const uint32_t flags = (entry >> 28) & 0xFu;

        if (x < 1 || x >= w - 1) {
          connected_last = false;
          last_x = x;
          continue;
        }
        if (x != last_x + 1) {
          connected_last = false;
        }
        last_x = x;

        const size_t y_sz = static_cast<size_t>(y);
        const size_t x_sz = static_cast<size_t>(x);
        const uint8_t v0 = buf[y_sz * stride + x_sz];
        if (v0 == 127) {
          connected_last = false;
          continue;
        }

        // Labels are pre-flattened on GPU, so the representative is one read.
        // gpu_sizes counts members including the root, matching
        // unionfind_get_set_size semantics.
        const uint64_t rep0 = RepresentativeOf(
            gpu_parents, total, static_cast<uint32_t>(y_sz * width + x_sz));
        if (rep0 == 0xFFFFFFFFu || rep0 >= total ||
            gpu_sizes[rep0] < kMinSetPixels) {
          connected_last = false;
          continue;
        }

        bool connected = false;

        auto do_conn = [&](int dx, int dy) {
          const size_t ny = static_cast<size_t>(y + dy);
          const size_t nx = static_cast<size_t>(x + dx);
          const uint8_t v1 = buf[ny * stride + nx];
          if (v0 + v1 == 255) {
            const uint64_t rep1 = RepresentativeOf(
                gpu_parents, total, static_cast<uint32_t>(ny * width + nx));
            if (rep1 != 0xFFFFFFFFu && rep1 < total &&
                gpu_sizes[rep1] > kMinSetPixels - 1) {
              const uint64_t clusterid =
                  (rep0 < rep1) ? ((rep1 << 32) + rep0) : ((rep0 << 32) + rep1);

              const uint32_t bucket =
                  static_cast<uint32_t>(u64hash(clusterid) % nclustermap);
              Entry* e = clustermap[bucket];
              while (e && e->id != clusterid) {
                e = e->next;
              }

              if (!e) {
                if (clustermap[bucket] == nullptr) {
                  touched_buckets.push_back(bucket);
                }
                if (pool_loc == kPoolSize) {
                  mem_pools.push_back(std::make_unique<Entry[]>(kPoolSize));
                  pool_loc = 0;
                }
                zarray_t* new_cluster =
                    zarray_create(sizeof(gpu_cluster_point_t));
                if (new_cluster == nullptr) {
                  throw std::bad_alloc();
                }
                e = &mem_pools.back()[pool_loc++];
                e->id = clusterid;
                e->cluster = new_cluster;
                e->next = clustermap[bucket];
                clustermap[bucket] = e;
              }

              gpu_cluster_point_t p{
                  static_cast<uint16_t>(2 * x + dx),
                  static_cast<uint16_t>(2 * y + dy),
                  static_cast<int16_t>(dx * (static_cast<int>(v1) - v0)),
                  static_cast<int16_t>(dy * (static_cast<int>(v1) - v0)),
                  0.0f,
              };
              zarray_add(e->cluster, &p);
              return true;
            }
          }
          return false;
        };

        // 4-connectivity
        if (flags & 1) do_conn(1, 0);
        if (flags & 2) do_conn(0, 1);

        // 8-connectivity
        if (!connected_last && (flags & 4)) {
          do_conn(-1, 1);
        }
        connected = false;
        if (flags & 8) {
          connected = do_conn(1, 1);
        }
        connected_last = connected;
      }
    }

    std::sort(touched_buckets.begin(), touched_buckets.end());
    for (uint32_t i : touched_buckets) {
      for (Entry* e = clustermap[i]; e != nullptr; e = e->next) {
        zarray_t* c = e->cluster;
        e->cluster = nullptr;
        zarray_add(clusters, reinterpret_cast<const void*>(&c));
      }
      clustermap[i] = nullptr;
    }
    scratch_guard.destroy_unreturned_clusters = false;
    return clusters;
  } catch (...) {
    for (int i = 0; i < zarray_size(clusters); ++i) {
      zarray_t* c = nullptr;
      zarray_get(clusters, i, reinterpret_cast<void*>(&c));
      if (c != nullptr) zarray_destroy(c);
    }
    zarray_destroy(clusters);
    return nullptr;
  }
}
