#include "talOS/vision/metal/gpu_sparse_clusters.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace {

// Number of pixels a set must contain to seed a cluster (matches upstream).
constexpr uint32_t kMinSetPixels = 25;

inline uint32_t RepresentativeOf(const uint32_t* parents, uint32_t total,
                                 uint32_t id) {
  if (id >= total) return 0xFFFFFFFFu;
  return parents[id];
}

}  // namespace

extern "C" zarray_t* BuildSparseClustersFromGpuLabels(
    const image_u8_t* threshim, int w, int h, int ts,
    const uint32_t* gpu_parents, const uint32_t* gpu_sizes,
    const uint32_t* candidate_coords, uint32_t candidate_count) {
  zarray_t* clusters = zarray_create(sizeof(zarray_t*));
  if (threshim == nullptr || gpu_parents == nullptr || gpu_sizes == nullptr ||
      w <= 0 || h <= 0 || ts <= 0) {
    return clusters;
  }
  const uint32_t total = static_cast<uint32_t>(w) * static_cast<uint32_t>(h);

  // Organize candidates into rows for cache-friendly access. Bounds-check
  // both coordinates: corrupt GPU writes must not cause OOB access.
  // Retain high-water storage; only candidate rows are cleared each frame.
  thread_local std::vector<std::vector<uint32_t>> row_candidates;
  if (row_candidates.size() < static_cast<size_t>(h)) {
    row_candidates.resize(static_cast<size_t>(h));
  }
  thread_local std::vector<uint32_t> touched_rows;
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

  struct Entry {
    uint64_t id;
    zarray_t* cluster;
    Entry* next;
  };

  const int nclustermap = std::max(256, static_cast<int>(0.2 * w * h));
  thread_local std::vector<Entry*> clustermap;
  if (clustermap.size() < static_cast<size_t>(nclustermap)) {
    clustermap.resize(static_cast<size_t>(nclustermap), nullptr);
  }
  thread_local std::vector<uint32_t> touched_buckets;
  touched_buckets.clear();
  std::vector<std::unique_ptr<Entry[]>> mem_pools;
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

      const uint8_t v0 = buf[y * ts + x];
      if (v0 == 127) {
        connected_last = false;
        continue;
      }

      // Labels are pre-flattened on GPU, so the representative is one read.
      // gpu_sizes counts members including the root, matching
      // unionfind_get_set_size semantics.
      const uint64_t rep0 = RepresentativeOf(gpu_parents, total,
                                             static_cast<uint32_t>(y * w + x));
      if (rep0 == 0xFFFFFFFFu || gpu_sizes[rep0] < kMinSetPixels) {
        connected_last = false;
        continue;
      }

      bool connected = false;

      auto do_conn = [&](int dx, int dy) {
        const uint8_t v1 = buf[(y + dy) * ts + x + dx];
        if (v0 + v1 == 255) {
          const uint64_t rep1 = RepresentativeOf(
              gpu_parents, total, static_cast<uint32_t>((y + dy) * w + x + dx));
          if (rep1 != 0xFFFFFFFFu && gpu_sizes[rep1] > kMinSetPixels - 1) {
            const uint64_t clusterid =
                (rep0 < rep1) ? ((rep1 << 32) + rep0) : ((rep0 << 32) + rep1);

            const uint32_t bucket =
                u64hash(clusterid) % static_cast<uint32_t>(nclustermap);
            Entry* e = clustermap[bucket];
            while (e && e->id != clusterid) {
              e = e->next;
            }

            if (!e) {
              if (clustermap[bucket] == nullptr) {
                touched_buckets.push_back(bucket);
              }
              if (pool_loc == kPoolSize) {
                pool_loc = 0;
                mem_pools.push_back(std::make_unique<Entry[]>(kPoolSize));
              }
              e = &mem_pools.back()[pool_loc++];
              e->id = clusterid;
              e->cluster = zarray_create(sizeof(gpu_cluster_point_t));
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
    for (Entry* e = clustermap[i]; e; e = e->next) {
      zarray_add(clusters, reinterpret_cast<const void*>(&e->cluster));
    }
    clustermap[i] = nullptr;
  }
  for (uint32_t y : touched_rows) row_candidates[y].clear();

  return clusters;
}
