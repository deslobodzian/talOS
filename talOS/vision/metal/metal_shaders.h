#pragma once

namespace talos::vision::metal {

inline constexpr const char* kMetalShadersSource = R"(#include <metal_stdlib>
using namespace metal;

// 1. rgb_to_gray: converts RGB/RGBA/BGR/BGRA/YUYV textures/buffers to 8-bit grayscale luminance
// format:
//   0 = RGB (3 bytes/px: R, G, B)
//   1 = RGBA (4 bytes/px: R, G, B, A)
//   2 = YUYV (4 bytes per 2 px: Y0, U0, Y1, V0)
//   3 = GRAY (1 byte/px: Y)
//   4 = BGR (3 bytes/px: B, G, R)
//   5 = BGRA (4 bytes/px: B, G, R, A)
kernel void rgb_to_gray(
    device const uchar* input_buf [[buffer(0)]],
    device uchar* output_buf [[buffer(1)]],
    constant uint& width [[buffer(2)]],
    constant uint& height [[buffer(3)]],
    constant uint& in_stride [[buffer(4)]],
    constant uint& out_stride [[buffer(5)]],
    constant uint& format [[buffer(6)]],
    uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= width || gid.y >= height) {
        return;
    }

    uchar gray = 0;
    if (format == 0) {
        // RGB
        uint idx = gid.y * in_stride + gid.x * 3;
        uint r = input_buf[idx];
        uint g = input_buf[idx + 1];
        uint b = input_buf[idx + 2];
        gray = static_cast<uchar>((299 * r + 587 * g + 114 * b + 500) / 1000);
    } else if (format == 1) {
        // RGBA
        uint idx = gid.y * in_stride + gid.x * 4;
        uint r = input_buf[idx];
        uint g = input_buf[idx + 1];
        uint b = input_buf[idx + 2];
        gray = static_cast<uchar>((299 * r + 587 * g + 114 * b + 500) / 1000);
    } else if (format == 2) {
        // YUYV: byte order Y0, U0, Y1, V0 (4 bytes per 2 pixels)
        uint pair_start = (gid.x / 2) * 4;
        uint y_offset = (gid.x % 2 == 0) ? 0 : 2;
        gray = input_buf[gid.y * in_stride + pair_start + y_offset];
    } else if (format == 4) {
        // BGR
        uint idx = gid.y * in_stride + gid.x * 3;
        uint b = input_buf[idx];
        uint g = input_buf[idx + 1];
        uint r = input_buf[idx + 2];
        gray = static_cast<uchar>((299 * r + 587 * g + 114 * b + 500) / 1000);
    } else if (format == 5) {
        // BGRA
        uint idx = gid.y * in_stride + gid.x * 4;
        uint b = input_buf[idx];
        uint g = input_buf[idx + 1];
        uint r = input_buf[idx + 2];
        gray = static_cast<uchar>((299 * r + 587 * g + 114 * b + 500) / 1000);
    } else {
        // Already GRAY
        gray = input_buf[gid.y * in_stride + gid.x];
    }

    output_buf[gid.y * out_stride + gid.x] = gray;
}

// 2. decimate_2x: 2x downsampling with box/Gaussian filter
kernel void decimate_2x(
    device const uchar* input_buf [[buffer(0)]],
    device uchar* output_buf [[buffer(1)]],
    constant uint& in_width [[buffer(2)]],
    constant uint& in_height [[buffer(3)]],
    constant uint& in_stride [[buffer(4)]],
    constant uint& out_stride [[buffer(5)]],
    uint2 gid [[thread_position_in_grid]])
{
    uint out_width = in_width / 2;
    uint out_height = in_height / 2;
    if (gid.x >= out_width || gid.y >= out_height) {
        return;
    }

    uint x0 = gid.x * 2;
    uint y0 = gid.y * 2;

    uchar val = input_buf[y0 * in_stride + x0];
    output_buf[gid.y * out_stride + gid.x] = val;
}

// 2b. rgb_to_gray_decimate_2x: fused color conversion and 2x downsampling
kernel void rgb_to_gray_decimate_2x(
    device const uchar* input_buf [[buffer(0)]],
    device uchar* output_buf [[buffer(1)]],
    constant uint& in_width [[buffer(2)]],
    constant uint& in_height [[buffer(3)]],
    constant uint& in_stride [[buffer(4)]],
    constant uint& out_stride [[buffer(5)]],
    constant uint& format [[buffer(6)]],
    uint2 gid [[thread_position_in_grid]])
{
    uint out_width = in_width / 2;
    uint out_height = in_height / 2;
    if (gid.x >= out_width || gid.y >= out_height) {
        return;
    }

    uint x0 = gid.x * 2;
    uint y0 = gid.y * 2;

    uchar gray = 0;
    if (format == 0) {
        // RGB
        uint idx = y0 * in_stride + x0 * 3;
        uint r = input_buf[idx];
        uint g = input_buf[idx + 1];
        uint b = input_buf[idx + 2];
        gray = static_cast<uchar>((299 * r + 587 * g + 114 * b + 500) / 1000);
    } else if (format == 1) {
        // RGBA
        uint idx = y0 * in_stride + x0 * 4;
        uint r = input_buf[idx];
        uint g = input_buf[idx + 1];
        uint b = input_buf[idx + 2];
        gray = static_cast<uchar>((299 * r + 587 * g + 114 * b + 500) / 1000);
    } else if (format == 2) {
        // YUYV
        uint pair_start = (x0 / 2) * 4;
        uint y_offset = (x0 % 2 == 0) ? 0 : 2;
        gray = input_buf[y0 * in_stride + pair_start + y_offset];
    } else if (format == 4) {
        // BGR
        uint idx = y0 * in_stride + x0 * 3;
        uint b = input_buf[idx];
        uint g = input_buf[idx + 1];
        uint r = input_buf[idx + 2];
        gray = static_cast<uchar>((299 * r + 587 * g + 114 * b + 500) / 1000);
    } else if (format == 5) {
        // BGRA
        uint idx = y0 * in_stride + x0 * 4;
        uint b = input_buf[idx];
        uint g = input_buf[idx + 1];
        uint r = input_buf[idx + 2];
        gray = static_cast<uchar>((299 * r + 587 * g + 114 * b + 500) / 1000);
    } else {
        // GRAY
        gray = input_buf[y0 * in_stride + x0];
    }

    output_buf[gid.y * out_stride + gid.x] = gray;
}

// 3. gradient_and_threshold: computes Sobel gradient magnitude and quantized gradient directions / adaptive threshold
kernel void gradient_and_threshold(
    device const uchar* input_buf [[buffer(0)]],
    device uchar* grad_mag_buf [[buffer(1)]],
    device uchar* grad_dir_buf [[buffer(2)]],
    device uchar* thresh_buf [[buffer(3)]],
    constant uint& width [[buffer(4)]],
    constant uint& height [[buffer(5)]],
    constant uint& stride [[buffer(6)]],
    constant uint& min_diff [[buffer(7)]],
    uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= width || gid.y >= height) {
        return;
    }

    uint x = gid.x;
    uint y = gid.y;

    if (x == 0 || x >= width - 1 || y == 0 || y >= height - 1) {
        grad_mag_buf[y * stride + x] = 0;
        grad_dir_buf[y * stride + x] = 0;
        thresh_buf[y * stride + x] = 127;
        return;
    }

    int p00 = input_buf[(y - 1) * stride + (x - 1)];
    int p01 = input_buf[(y - 1) * stride + x];
    int p02 = input_buf[(y - 1) * stride + (x + 1)];
    int p10 = input_buf[y * stride + (x - 1)];
    int p11 = input_buf[y * stride + x];
    int p12 = input_buf[y * stride + (x + 1)];
    int p20 = input_buf[(y + 1) * stride + (x - 1)];
    int p21 = input_buf[(y + 1) * stride + x];
    int p22 = input_buf[(y + 1) * stride + (x + 1)];

    int gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20);
    int gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02);

    int mag = (abs(gx) + abs(gy)) / 4;
    if (mag > 255) mag = 255;
    grad_mag_buf[y * stride + x] = static_cast<uchar>(mag);

    uchar dir = 0;
    if (abs(gx) > abs(gy)) {
        dir = (gx > 0) ? 0 : 2;
    } else {
        dir = (gy > 0) ? 1 : 3;
    }
    grad_dir_buf[y * stride + x] = dir;

    int min_val = min(min(min(min(p00, p01), min(p02, p10)), min(min(p11, p12), min(p20, p21))), p22);
    int max_val = max(max(max(max(p00, p01), max(p02, p10)), max(max(p11, p12), max(p20, p21))), p22);

    if (max_val - min_val < static_cast<int>(min_diff)) {
        thresh_buf[y * stride + x] = 127;
    } else {
        int th = min_val + (max_val - min_val) / 2;
        thresh_buf[y * stride + x] = (p11 > th) ? 255 : 0;
    }
}

// 4. candidate_mask: marks quad candidate edge pixels
kernel void candidate_mask(
    device const uchar* thresh_buf [[buffer(0)]],
    device uchar* mask_buf [[buffer(1)]],
    constant uint& width [[buffer(2)]],
    constant uint& height [[buffer(3)]],
    constant uint& stride [[buffer(4)]],
    uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= width || gid.y >= height) {
        return;
    }

    uint x = gid.x;
    uint y = gid.y;

    if (x == 0 || x >= width - 1 || y == 0 || y >= height - 1) {
        mask_buf[y * stride + x] = 0;
        return;
    }

    uchar v0 = thresh_buf[y * stride + x];
    if (v0 == 127) {
        mask_buf[y * stride + x] = 0;
        return;
    }

    uchar right = thresh_buf[y * stride + (x + 1)];
    uchar down  = thresh_buf[(y + 1) * stride + x];
    uchar left  = thresh_buf[y * stride + (x - 1)];
    uchar up    = thresh_buf[(y - 1) * stride + x];

    bool is_edge = (v0 + right == 255) || (v0 + down == 255) ||
                   (v0 + left == 255) || (v0 + up == 255);

    mask_buf[y * stride + x] = is_edge ? 255 : 0;
}

// 5. extract_edge_candidates: identifies transition pixels and outputs packed coordinates
kernel void extract_edge_candidates(
    device const uchar* thresh_buf [[buffer(0)]],
    device uint* candidate_coords [[buffer(1)]],
    device atomic_uint* candidate_count [[buffer(2)]],
    constant uint& width [[buffer(3)]],
    constant uint& height [[buffer(4)]],
    constant uint& stride [[buffer(5)]],
    constant uint& max_candidates [[buffer(6)]],
    uint2 gid [[thread_position_in_grid]])
{
    uint x = gid.x;
    uint y = gid.y;

    if (x < 1 || x >= width - 1 || y < 1 || y >= height - 1) {
        return;
    }

    uchar v0 = thresh_buf[y * stride + x];
    if (v0 == 127) {
        return;
    }

    uchar right = thresh_buf[y * stride + (x + 1)];
    uchar down  = thresh_buf[(y + 1) * stride + x];
    uchar dl    = thresh_buf[(y + 1) * stride + (x - 1)];
    uchar dr    = thresh_buf[(y + 1) * stride + (x + 1)];

    // Check 4 forward transition directions: (1,0), (0,1), (-1,1), (1,1)
    uchar flags = 0;
    if (v0 + right == 255) flags |= 1;
    if (v0 + down  == 255) flags |= 2;
    if (v0 + dl    == 255) flags |= 4;
    if (v0 + dr    == 255) flags |= 8;

    if (flags != 0) {
        uint idx = atomic_fetch_add_explicit(candidate_count, 1, memory_order_relaxed);
        if (idx < max_candidates) {
            candidate_coords[idx] = (static_cast<uint>(flags) << 28) |
                                   ((y & 0x3FFF) << 14) |
                                   (x & 0x3FFF);
        }
    }
}

// 6. GPU Connected Components: ccl_init
kernel void ccl_init(
    device const uchar* thresh_buf [[buffer(0)]],
    device atomic_uint* parent [[buffer(1)]],
    device atomic_uint* set_size [[buffer(2)]],
    constant uint& width [[buffer(3)]],
    constant uint& height [[buffer(4)]],
    constant uint& stride [[buffer(5)]],
    uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= width || gid.y >= height) return;
    uint idx = gid.y * width + gid.x;
    uchar v = thresh_buf[gid.y * stride + gid.x];
    if (v == 127) {
        atomic_store_explicit(&parent[idx], 0xFFFFFFFF, memory_order_relaxed);
    } else {
        atomic_store_explicit(&parent[idx], idx, memory_order_relaxed);
    }
    atomic_store_explicit(&set_size[idx], 0, memory_order_relaxed);
}

// Helper for GPU CCL: find root with path halving
inline uint ccl_find_root(device atomic_uint* parent, uint id) {
    uint root = atomic_load_explicit(&parent[id], memory_order_relaxed);
    if (root == 0xFFFFFFFF) return 0xFFFFFFFF;
    while (root != id) {
        uint next = atomic_load_explicit(&parent[root], memory_order_relaxed);
        atomic_store_explicit(&parent[id], next, memory_order_relaxed);
        id = root;
        root = next;
    }
    return root;
}

// Helper for GPU CCL: atomic union grafting higher node ID to lower node ID
//
// Correctness note: the graft MUST use compare-exchange on the root slot, not
// an unconditional fetch_min store. fetch_min overwrites parent[high] even
// when another thread already grafted high elsewhere, destroying the
// previously established link with nobody left to repair it: the requested
// merge is then silently split. CAS only fires while high is still a root, so
// established connectivity is never destroyed; on failure we continue with
// the observed parent, which strictly decreases min(a, b), so the loop still
// terminates.
inline void ccl_union(device atomic_uint* parent, uint a, uint b) {
    while (true) {
        a = ccl_find_root(parent, a);
        b = ccl_find_root(parent, b);
        if (a == b || a == 0xFFFFFFFF || b == 0xFFFFFFFF) return;
        uint high = max(a, b);
        uint low  = min(a, b);
        uint expected = high;
        if (atomic_compare_exchange_weak_explicit(
                &parent[high], &expected, low,
                memory_order_relaxed, memory_order_relaxed)) {
            break;
        }
        a = expected;
        b = low;
    }
}

// 7. GPU Connected Components: ccl_union_pass
kernel void ccl_union_pass(
    device const uchar* thresh_buf [[buffer(0)]],
    device atomic_uint* parent [[buffer(1)]],
    constant uint& width [[buffer(2)]],
    constant uint& height [[buffer(3)]],
    constant uint& stride [[buffer(4)]],
    uint2 gid [[thread_position_in_grid]])
{
    uint x = gid.x;
    uint y = gid.y;
    if (x >= width || y >= height) return;

    uchar v = thresh_buf[y * stride + x];
    if (v == 127) return;

    uint idx = y * width + x;

    // Edge domain mirrors connected_components() (do_unionfind_first_line /
    // do_unionfind_line2) exactly: border columns are never union sources and
    // no union extends into the last column, so the GPU partition matches the
    // CPU partition (modulo the CPU's conditional-skip heuristics, which are
    // transitivity optimizations preserving the closure). All comparisons are
    // written to stay well-defined for tiny widths (no unsigned underflow).
    //
    //   right (x,y)-(x+1,y):       CPU creates it via ref (x+1,y) for x+1 < w-1
    //   down (x,y)-(x,y+1):        CPU creates it via ref (x,y+1) for x in [1,w-1)
    //   down-right (white):        CPU creates it via ref (x+1,y+1) up-left
    //   down-left (white):         CPU creates it via ref (x-1,y+1) up-right,
    //                              which needs x-1 >= 1, i.e. x >= 2

    // Connect right (x + 1, y)
    if (x + 1 < width - 1) {
        uchar vr = thresh_buf[y * stride + (x + 1)];
        if (vr == v) {
            ccl_union(parent, idx, idx + 1);
        }
    }

    // Connect down (x, y + 1)
    if (x >= 1 && x < width - 1 && y + 1 < height) {
        uchar vd = thresh_buf[(y + 1) * stride + x];
        if (vd == v) {
            ccl_union(parent, idx, idx + width);
        }
    }

    // White pixels (255) are 8-connected: connect diagonals
    if (v == 255 && y + 1 < height) {
        // Down-right
        if (x + 1 < width - 1) {
            uchar vdr = thresh_buf[(y + 1) * stride + (x + 1)];
            if (vdr == 255) {
                ccl_union(parent, idx, idx + width + 1);
            }
        }
        // Down-left
        if (x >= 2) {
            uchar vdl = thresh_buf[(y + 1) * stride + (x - 1)];
            if (vdl == 255) {
                ccl_union(parent, idx, idx + width - 1);
            }
        }
    }
}

// 8. GPU Connected Components: ccl_flatten
kernel void ccl_flatten(
    device atomic_uint* parent [[buffer(0)]],
    constant uint& total_pixels [[buffer(1)]],
    uint gid [[thread_position_in_grid]])
{
    if (gid >= total_pixels) return;
    uint p = atomic_load_explicit(&parent[gid], memory_order_relaxed);
    if (p == 0xFFFFFFFF) return;
    while (true) {
        uint next = atomic_load_explicit(&parent[p], memory_order_relaxed);
        if (next == p) break;
        p = next;
    }
    atomic_store_explicit(&parent[gid], p, memory_order_relaxed);
}

// 9. GPU Connected Components: ccl_compute_sizes
//
// Sizes must be attributed to the TRUE root: ccl_flatten runs concurrently
// across threads, so parent[gid] may name an intermediate node whose own
// parent was still settling (stale read). Attributing to the intermediate
// undercounts the true root and can wrongly filter sets near the 25-pixel
// cutoff. Chasing here is read-only and runs after all union/flatten writes
// are ordered before this dispatch, so the array is stable: parent links
// only ever point to smaller ids (union grafts high->low, flatten writes
// roots), therefore the chase strictly decreases and always terminates at
// the exact root, deterministically.
kernel void ccl_compute_sizes(
    device const atomic_uint* parent [[buffer(0)]],
    device atomic_uint* set_size [[buffer(1)]],
    constant uint& total_pixels [[buffer(2)]],
    uint gid [[thread_position_in_grid]])
{
    if (gid >= total_pixels) return;
    uint root = atomic_load_explicit(&parent[gid], memory_order_relaxed);
    if (root == 0xFFFFFFFF) return;
    while (true) {
        uint next = atomic_load_explicit(&parent[root], memory_order_relaxed);
        if (next == root || next == 0xFFFFFFFF) break;
        root = next;
    }
    if (root != 0xFFFFFFFF) {
        atomic_fetch_add_explicit(&set_size[root], 1, memory_order_relaxed);
    }
}

// 5. tile_min_max_4x4: computes min and max luminance across 4x4 pixel tiles
kernel void tile_min_max_4x4(
    device const uchar* input_buf [[buffer(0)]],
    device uchar* im_min [[buffer(1)]],
    device uchar* im_max [[buffer(2)]],
    constant uint& width [[buffer(3)]],
    constant uint& height [[buffer(4)]],
    constant uint& stride [[buffer(5)]],
    uint2 gid [[thread_position_in_grid]])
{
    uint tw = width / 4;
    uint th = height / 4;
    if (gid.x >= tw || gid.y >= th) {
        return;
    }

    uint tx = gid.x;
    uint ty = gid.y;

    uchar min_v = 255;
    uchar max_v = 0;

    for (uint dy = 0; dy < 4; ++dy) {
        uint y = ty * 4 + dy;
        uint row_offset = y * stride + tx * 4;
        for (uint dx = 0; dx < 4; ++dx) {
            uchar v = input_buf[row_offset + dx];
            min_v = min(min_v, v);
            max_v = max(max_v, v);
        }
    }

    im_min[ty * tw + tx] = min_v;
    im_max[ty * tw + tx] = max_v;
}

// 6. tile_blur_and_threshold: 3x3 tile min/max neighborhood blur and pixel binarization
kernel void tile_blur_and_threshold(
    device const uchar* input_buf [[buffer(0)]],
    device const uchar* im_min [[buffer(1)]],
    device const uchar* im_max [[buffer(2)]],
    device uchar* thresh_buf [[buffer(3)]],
    constant uint& width [[buffer(4)]],
    constant uint& height [[buffer(5)]],
    constant uint& stride [[buffer(6)]],
    constant uint& min_diff [[buffer(7)]],
    uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= width || gid.y >= height) {
        return;
    }

    uint x = gid.x;
    uint y = gid.y;
    uint tw = width / 4;
    uint th = height / 4;

    if (tw == 0 || th == 0) {
        thresh_buf[y * stride + x] = 127;
        return;
    }

    uint tx = min(x / 4, tw - 1);
    uint ty = min(y / 4, th - 1);

    uchar min_v = 255;
    uchar max_v = 0;

    for (int dy = -1; dy <= 1; ++dy) {
        int nty = int(ty) + dy;
        if (nty < 0 || nty >= int(th)) continue;
        for (int dx = -1; dx <= 1; ++dx) {
            int ntx = int(tx) + dx;
            if (ntx < 0 || ntx >= int(tw)) continue;
            min_v = min(min_v, im_min[nty * tw + ntx]);
            max_v = max(max_v, im_max[nty * tw + ntx]);
        }
    }

    bool is_remainder = (x >= tw * 4) || (y >= th * 4);
    if (!is_remainder && ((max_v - min_v) < static_cast<uchar>(min_diff))) {
        thresh_buf[y * stride + x] = 127;
    } else {
        uchar thresh = min_v + (max_v - min_v) / 2;
        uchar v = input_buf[y * stride + x];
        thresh_buf[y * stride + x] = (v > thresh) ? 255 : 0;
    }
}
)";

}  // namespace talos::vision::metal
