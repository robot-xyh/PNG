#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace circle::perception {

struct LetterboxParams {
  float scale{1.0F};
  int top_pad{0};
  int left_pad{0};
  int unpadded_w{0};
  int unpadded_h{0};
};

inline LetterboxParams computeLetterbox(int src_w, int src_h, int model_w, int model_h) {
  LetterboxParams p;
  if (src_w <= 0 || src_h <= 0 || model_w <= 0 || model_h <= 0) {
    return p;
  }
  const float r = std::min(static_cast<float>(model_w) / static_cast<float>(src_w),
                           static_cast<float>(model_h) / static_cast<float>(src_h));
  p.scale = r;
  p.unpadded_w = static_cast<int>(std::round(static_cast<float>(src_w) * r));
  p.unpadded_h = static_cast<int>(std::round(static_cast<float>(src_h) * r));
  p.left_pad = static_cast<int>(std::round(
      (static_cast<float>(model_w) - static_cast<float>(p.unpadded_w)) / 2.0F - 0.1F));
  p.top_pad = static_cast<int>(std::round(
      (static_cast<float>(model_h) - static_cast<float>(p.unpadded_h)) / 2.0F - 0.1F));
  return p;
}

/** Letterbox RGB (src_w x src_h row-major) into model_w x model_h buffer (pad=114). */
inline void letterboxRgbToBuffer(const uint8_t* src_rgb, int src_w, int src_h,
                                 const LetterboxParams& lb, int model_w, int model_h,
                                 uint8_t* dst) {
  if (!src_rgb || !dst || model_w <= 0 || model_h <= 0 || src_w <= 0 || src_h <= 0) {
    return;
  }
  const size_t dst_stride = static_cast<size_t>(model_w) * 3u;
  std::memset(dst, 114, dst_stride * static_cast<size_t>(model_h));
  for (int y = 0; y < lb.unpadded_h; ++y) {
    const int dst_y = y + lb.top_pad;
    if (dst_y < 0 || dst_y >= model_h) {
      continue;
    }
    const int src_y = std::clamp(
        static_cast<int>(std::round(static_cast<float>(y) / std::max(lb.scale, 1.0e-6F))),
        0, src_h - 1);
    const uint8_t* src_row =
        src_rgb + static_cast<size_t>(src_y) * static_cast<size_t>(src_w) * 3u;
    uint8_t* dst_row = dst + static_cast<size_t>(dst_y) * dst_stride +
                       static_cast<size_t>(lb.left_pad) * 3u;
    if (lb.unpadded_w == src_w && lb.scale >= 0.999F) {
      std::memcpy(dst_row, src_row, static_cast<size_t>(lb.unpadded_w) * 3u);
      continue;
    }
    for (int x = 0; x < lb.unpadded_w; ++x) {
      const int src_x = std::clamp(
          static_cast<int>(std::round(static_cast<float>(x) / std::max(lb.scale, 1.0e-6F))),
          0, src_w - 1);
      const uint8_t* sp = src_row + static_cast<size_t>(src_x) * 3u;
      uint8_t* dp = dst_row + static_cast<size_t>(x) * 3u;
      dp[0] = sp[0];
      dp[1] = sp[1];
      dp[2] = sp[2];
    }
  }
}

}  // namespace circle::perception
