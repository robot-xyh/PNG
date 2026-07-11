#include "circle/vision/yolo_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace circle::vision {
namespace {

constexpr float kMinBoxDim = 1.0F;

float sigmoid(float x) { return 1.0F / (1.0F + std::exp(-x)); }

float fastSigmoid(float x) {
  if (x >= 8.0F) {
    return 1.0F;
  }
  if (x <= -8.0F) {
    return 0.0F;
  }
  const float ax = std::abs(x);
  const float x2 = x * x;
  const float v = x / (1.0F + ax + 0.555F * x2 / (1.0F + ax));
  return 0.5F + 0.5F * v;
}

int8_t clsInt8Threshold(float conf_thresh, int32_t zp, float scale) {
  if (conf_thresh <= 0.0F) {
    return -128;
  }
  if (conf_thresh >= 1.0F || scale <= 0.0F) {
    return 127;
  }
  const float logit = -std::log(1.0F / conf_thresh - 1.0F);
  const float q = logit / scale + static_cast<float>(zp);
  const int iq = static_cast<int>(std::floor(q));
  return static_cast<int8_t>(std::clamp(iq, -128, 127));
}

float iou(const YoloDetection& a, const YoloDetection& b) {
  const float x1 = std::max(a.x1, b.x1);
  const float y1 = std::max(a.y1, b.y1);
  const float x2 = std::min(a.x2, b.x2);
  const float y2 = std::min(a.y2, b.y2);
  const float w = std::max(0.0F, x2 - x1);
  const float h = std::max(0.0F, y2 - y1);
  const float inter = w * h;
  const float area_a = std::max(0.0F, a.x2 - a.x1) * std::max(0.0F, a.y2 - a.y1);
  const float area_b = std::max(0.0F, b.x2 - b.x1) * std::max(0.0F, b.y2 - b.y1);
  return inter / (area_a + area_b - inter + 1.0e-7F);
}

std::vector<YoloDetection> nms(std::vector<YoloDetection> candidates,
                               float iou_thresh,
                               int max_det) {
  std::sort(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) { return a.score > b.score; });
  std::vector<YoloDetection> keep;
  std::vector<bool> suppressed(candidates.size(), false);
  for (size_t i = 0; i < candidates.size() &&
                     static_cast<int>(keep.size()) < max_det;
       ++i) {
    if (suppressed[i]) {
      continue;
    }
    keep.push_back(candidates[i]);
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      if (!suppressed[j] && iou(candidates[i], candidates[j]) > iou_thresh) {
        suppressed[j] = true;
      }
    }
  }
  return keep;
}

void clampToImage(YoloDetection& d, int img_w, int img_h) {
  d.x1 = std::clamp(d.x1, 0.0F, static_cast<float>(img_w));
  d.x2 = std::clamp(d.x2, 0.0F, static_cast<float>(img_w));
  d.y1 = std::clamp(d.y1, 0.0F, static_cast<float>(img_h));
  d.y2 = std::clamp(d.y2, 0.0F, static_cast<float>(img_h));
}

void dflDecodeCellFloat(const float* box_ptr,
                        int g,
                        int reg_max,
                        int hw,
                        bool nhwc,
                        int box_c,
                        float out[4]) {
  for (int p = 0; p < 4; ++p) {
    float max_val = -1.0e30F;
    float vals[32];
    for (int r = 0; r < reg_max; ++r) {
      const int offset = nhwc ? (g * box_c + p * reg_max + r)
                              : ((p * reg_max + r) * hw + g);
      const float v = box_ptr[offset];
      vals[r] = v;
      max_val = std::max(max_val, v);
    }
    float sum_exp = 0.0F;
    float weighted = 0.0F;
    for (int r = 0; r < reg_max; ++r) {
      const float e = std::exp(vals[r] - max_val);
      sum_exp += e;
      weighted += e * static_cast<float>(r);
    }
    out[p] = weighted / std::max(sum_exp, 1.0e-12F);
  }
}

void dflDecodeCellInt8(const int8_t* box_ptr,
                       int g,
                       int reg_max,
                       int hw,
                       int32_t zp,
                       float scale,
                       bool nhwc,
                       int box_c,
                       float out[4]) {
  for (int p = 0; p < 4; ++p) {
    float max_val = -1.0e30F;
    float vals[32];
    for (int r = 0; r < reg_max; ++r) {
      const int offset = nhwc ? (g * box_c + p * reg_max + r)
                              : ((p * reg_max + r) * hw + g);
      const float v = (static_cast<float>(box_ptr[offset]) - zp) * scale;
      vals[r] = v;
      max_val = std::max(max_val, v);
    }
    float sum_exp = 0.0F;
    float weighted = 0.0F;
    for (int r = 0; r < reg_max; ++r) {
      const float e = std::exp(vals[r] - max_val);
      sum_exp += e;
      weighted += e * static_cast<float>(r);
    }
    out[p] = weighted / std::max(sum_exp, 1.0e-12F);
  }
}

}  // namespace

std::vector<YoloDetection> yoloPostprocess(
    const float* output_data,
    const uint32_t* output_shape,
    int n_dims,
    int img_w,
    int img_h,
    float scale,
    int top_pad,
    int left_pad,
    float conf_thresh,
    float iou_thresh,
    int max_det) {
  if (!output_data || !output_shape || n_dims < 2 || img_w <= 0 || img_h <= 0 ||
      scale <= 0.0F || max_det <= 0) {
    return {};
  }

  int num_boxes = 0;
  int num_features = 0;
  bool transposed = false;
  if (n_dims == 3) {
    const int d1 = static_cast<int>(output_shape[1]);
    const int d2 = static_cast<int>(output_shape[2]);
    if (d1 <= d2) {
      num_features = d1;
      num_boxes = d2;
      transposed = true;
    } else {
      num_boxes = d1;
      num_features = d2;
    }
  } else if (n_dims == 2) {
    num_boxes = static_cast<int>(output_shape[0]);
    num_features = static_cast<int>(output_shape[1]);
  } else if (n_dims == 4) {
    const int d1 = static_cast<int>(output_shape[1]);
    const int d2 = static_cast<int>(output_shape[2]);
    const int d3 = static_cast<int>(output_shape[3]);
    if (d1 > d3) {
      // [batch, features, grid_h, grid_w]
      num_features = d1;
      num_boxes = d2 * d3;
      transposed = true;
    } else {
      // [batch, grid_h, grid_w, features]
      num_boxes = d1 * d2;
      num_features = d3;
    }
  } else {
    return {};
  }
  if (num_boxes <= 0 || num_features < 5) {
    return {};
  }

  std::vector<YoloDetection> candidates;
  candidates.reserve(256);
  const bool has_objectness = (num_features == 85);
  const int class_offset = has_objectness ? 5 : 4;
  const int class_count = num_features - class_offset;
  if (class_count <= 0) {
    return {};
  }

  for (int i = 0; i < num_boxes; ++i) {
    const auto value_at = [&](int feature) {
      if (transposed) {
        return output_data[static_cast<size_t>(feature) * num_boxes + i];
      }
      return output_data[static_cast<size_t>(i) * num_features + feature];
    };
    const float cx = value_at(0);
    const float cy = value_at(1);
    const float w = value_at(2);
    const float h = value_at(3);

    float best_score = value_at(class_offset);
    int best_cls = 0;
    for (int c = 1; c < class_count; ++c) {
      const float s = value_at(class_offset + c);
      if (s > best_score) {
        best_score = s;
        best_cls = c;
      }
    }
    float score = best_score;
    if (score < 0.0F || score > 1.0F) {
      score = sigmoid(score);
    }
    if (has_objectness) {
      float obj = value_at(4);
      if (obj < 0.0F || obj > 1.0F) {
        obj = sigmoid(obj);
      }
      score *= obj;
    }
    if (score < conf_thresh) {
      continue;
    }

    YoloDetection d;
    d.x1 = ((cx - w * 0.5F) - static_cast<float>(left_pad)) / scale;
    d.y1 = ((cy - h * 0.5F) - static_cast<float>(top_pad)) / scale;
    d.x2 = ((cx + w * 0.5F) - static_cast<float>(left_pad)) / scale;
    d.y2 = ((cy + h * 0.5F) - static_cast<float>(top_pad)) / scale;
    d.score = score;
    d.class_id = best_cls;
    clampToImage(d, img_w, img_h);
    if (d.x2 - d.x1 >= 1.0F && d.y2 - d.y1 >= 1.0F) {
      candidates.push_back(d);
    }
  }
  return nms(std::move(candidates), iou_thresh, max_det);
}

std::vector<YoloDetection> yoloPostprocessMultihead(
    const float* const* output_data,
    const uint32_t* const* output_shapes,
    const int* output_dims,
    const bool* output_is_nhwc,
    int num_outputs,
    int img_w,
    int img_h,
    float scale,
    int top_pad,
    int left_pad,
    float conf_thresh,
    float iou_thresh,
    int max_det,
    int model_w,
    int model_h) {
  if (!output_data || !output_shapes || !output_dims || num_outputs < 4 ||
      img_w <= 0 || img_h <= 0 || model_w <= 0 || model_h <= 0 ||
      scale <= 0.0F || max_det <= 0 ||
      (num_outputs % 2 != 0 && num_outputs % 3 != 0)) {
    return {};
  }

  const int pair_per_branch =
      (num_outputs % 3 == 0) ? (num_outputs / 3 >= 2 ? num_outputs / 3 : 2) : 2;
  const int n_branches = num_outputs / pair_per_branch;
  float logit_thresh = -100.0F;
  if (conf_thresh > 0.0F && conf_thresh < 1.0F) {
    logit_thresh = -std::log(1.0F / conf_thresh - 1.0F);
  }

  std::vector<YoloDetection> candidates;
  candidates.reserve(512);

  for (int br = 0; br < n_branches; ++br) {
    const int box_idx = pair_per_branch * br;
    const int cls_idx = pair_per_branch * br + 1;
    if (!output_data[box_idx] || !output_data[cls_idx]) {
      continue;
    }

    const int box_ndim = output_dims[box_idx];
    const int cls_ndim = output_dims[cls_idx];
    const bool box_nhwc = output_is_nhwc && output_is_nhwc[box_idx];
    const bool cls_nhwc = output_is_nhwc && output_is_nhwc[cls_idx];

    int box_c = 0;
    int grid_h = 0;
    int grid_w = 0;
    if (box_ndim == 4) {
      if (box_nhwc) {
        grid_h = static_cast<int>(output_shapes[box_idx][1]);
        grid_w = static_cast<int>(output_shapes[box_idx][2]);
        box_c = static_cast<int>(output_shapes[box_idx][3]);
      } else {
        box_c = static_cast<int>(output_shapes[box_idx][1]);
        grid_h = static_cast<int>(output_shapes[box_idx][2]);
        grid_w = static_cast<int>(output_shapes[box_idx][3]);
      }
    } else if (box_ndim == 3) {
      box_c = static_cast<int>(output_shapes[box_idx][0]);
      grid_h = static_cast<int>(output_shapes[box_idx][1]);
      grid_w = static_cast<int>(output_shapes[box_idx][2]);
    } else {
      continue;
    }

    int nc = 0;
    if (cls_ndim == 4) {
      nc = cls_nhwc ? static_cast<int>(output_shapes[cls_idx][3])
                    : static_cast<int>(output_shapes[cls_idx][1]);
    } else if (cls_ndim == 3) {
      nc = static_cast<int>(output_shapes[cls_idx][0]);
    } else {
      continue;
    }
    if (box_c <= 0 || box_c % 4 != 0 || grid_h <= 0 || grid_w <= 0 || nc <= 0) {
      continue;
    }

    const int hw = grid_h * grid_w;
    const int reg_max = box_c / 4;
    if (reg_max <= 0 || reg_max > 32) {
      continue;
    }
    const float stride_h = static_cast<float>(model_h) / grid_h;
    const float stride_w = static_cast<float>(model_w) / grid_w;
    const float* box_ptr = output_data[box_idx];
    const float* cls_ptr = output_data[cls_idx];

    for (int row = 0; row < grid_h; ++row) {
      for (int col = 0; col < grid_w; ++col) {
        const int g = row * grid_w + col;

        int best_cls = 0;
        float best_raw = -1.0e30F;
        if (cls_nhwc) {
          const float* cell_cls = cls_ptr + static_cast<size_t>(g) * nc;
          for (int c = 0; c < nc; ++c) {
            if (cell_cls[c] > best_raw) {
              best_raw = cell_cls[c];
              best_cls = c;
            }
          }
        } else {
          for (int c = 0; c < nc; ++c) {
            const float v = cls_ptr[static_cast<size_t>(c) * hw + g];
            if (v > best_raw) {
              best_raw = v;
              best_cls = c;
            }
          }
        }
        if (best_raw < logit_thresh) {
          continue;
        }
        const float best_score = sigmoid(best_raw);
        if (best_score < conf_thresh) {
          continue;
        }

        float dist[4];
        dflDecodeCellFloat(box_ptr, g, reg_max, hw, box_nhwc, box_c, dist);

        const float cx = static_cast<float>(col) + 0.5F;
        const float cy = static_cast<float>(row) + 0.5F;
        YoloDetection d;
        d.x1 = ((cx - dist[0]) * stride_w - static_cast<float>(left_pad)) / scale;
        d.y1 = ((cy - dist[1]) * stride_h - static_cast<float>(top_pad)) / scale;
        d.x2 = ((cx + dist[2]) * stride_w - static_cast<float>(left_pad)) / scale;
        d.y2 = ((cy + dist[3]) * stride_h - static_cast<float>(top_pad)) / scale;
        d.score = best_score;
        d.class_id = best_cls;
        clampToImage(d, img_w, img_h);
        if ((d.x2 - d.x1) >= kMinBoxDim && (d.y2 - d.y1) >= kMinBoxDim) {
          candidates.push_back(d);
        }
      }
    }
  }

  return nms(std::move(candidates), iou_thresh, max_det);
}

std::vector<YoloDetection> yoloPostprocessMultiheadInt8(
    const int8_t* const* output_data,
    const uint32_t* const* output_shapes,
    const int* output_dims,
    const bool* output_is_nhwc,
    const int32_t* output_zps,
    const float* output_scales,
    int num_outputs,
    int img_w,
    int img_h,
    float scale,
    int top_pad,
    int left_pad,
    float conf_thresh,
    float iou_thresh,
    int max_det,
    int model_w,
    int model_h) {
  if (!output_data || !output_shapes || !output_dims || !output_zps ||
      !output_scales || num_outputs < 4 || img_w <= 0 || img_h <= 0 ||
      model_w <= 0 || model_h <= 0 || scale <= 0.0F || max_det <= 0 ||
      (num_outputs % 2 != 0 && num_outputs % 3 != 0)) {
    return {};
  }

  const int pair_per_branch =
      (num_outputs % 3 == 0) ? (num_outputs / 3 >= 2 ? num_outputs / 3 : 2) : 2;
  const int n_branches = num_outputs / pair_per_branch;
  std::vector<YoloDetection> candidates;
  candidates.reserve(512);

  for (int br = 0; br < n_branches; ++br) {
    const int box_idx = pair_per_branch * br;
    const int cls_idx = pair_per_branch * br + 1;
    if (!output_data[box_idx] || !output_data[cls_idx]) {
      continue;
    }

    const int box_ndim = output_dims[box_idx];
    const int cls_ndim = output_dims[cls_idx];
    const bool box_nhwc = output_is_nhwc && output_is_nhwc[box_idx];
    const bool cls_nhwc = output_is_nhwc && output_is_nhwc[cls_idx];

    int box_c = 0;
    int grid_h = 0;
    int grid_w = 0;
    if (box_ndim == 4) {
      if (box_nhwc) {
        grid_h = static_cast<int>(output_shapes[box_idx][1]);
        grid_w = static_cast<int>(output_shapes[box_idx][2]);
        box_c = static_cast<int>(output_shapes[box_idx][3]);
      } else {
        box_c = static_cast<int>(output_shapes[box_idx][1]);
        grid_h = static_cast<int>(output_shapes[box_idx][2]);
        grid_w = static_cast<int>(output_shapes[box_idx][3]);
      }
    } else if (box_ndim == 3) {
      box_c = static_cast<int>(output_shapes[box_idx][0]);
      grid_h = static_cast<int>(output_shapes[box_idx][1]);
      grid_w = static_cast<int>(output_shapes[box_idx][2]);
    } else {
      continue;
    }

    int nc = 0;
    if (cls_ndim == 4) {
      nc = cls_nhwc ? static_cast<int>(output_shapes[cls_idx][3])
                    : static_cast<int>(output_shapes[cls_idx][1]);
    } else if (cls_ndim == 3) {
      nc = static_cast<int>(output_shapes[cls_idx][0]);
    } else {
      continue;
    }
    if (box_c <= 0 || box_c % 4 != 0 || grid_h <= 0 || grid_w <= 0 || nc <= 0) {
      continue;
    }

    const int hw = grid_h * grid_w;
    const int reg_max = box_c / 4;
    if (reg_max <= 0 || reg_max > 32) {
      continue;
    }
    const float stride_h = static_cast<float>(model_h) / grid_h;
    const float stride_w = static_cast<float>(model_w) / grid_w;
    const int32_t cls_zp = output_zps[cls_idx];
    const float cls_scale = output_scales[cls_idx];
    const int32_t box_zp = output_zps[box_idx];
    const float box_scale = output_scales[box_idx];
    const int8_t cls_i8_thr = clsInt8Threshold(conf_thresh, cls_zp, cls_scale);
    const int8_t* box_ptr = output_data[box_idx];
    const int8_t* cls_ptr = output_data[cls_idx];

    for (int row = 0; row < grid_h; ++row) {
      for (int col = 0; col < grid_w; ++col) {
        const int g = row * grid_w + col;

        int best_cls = 0;
        int8_t best_raw = -128;
        if (cls_nhwc) {
          const int8_t* cell_cls = cls_ptr + static_cast<size_t>(g) * nc;
          for (int c = 0; c < nc; ++c) {
            if (cell_cls[c] > best_raw) {
              best_raw = cell_cls[c];
              best_cls = c;
            }
          }
        } else {
          for (int c = 0; c < nc; ++c) {
            const int8_t v = cls_ptr[static_cast<size_t>(c) * hw + g];
            if (v > best_raw) {
              best_raw = v;
              best_cls = c;
            }
          }
        }
        if (best_raw < cls_i8_thr) {
          continue;
        }
        const float raw_f = (static_cast<float>(best_raw) - cls_zp) * cls_scale;
        const float best_score = fastSigmoid(raw_f);
        if (best_score < conf_thresh) {
          continue;
        }

        float dist[4];
        dflDecodeCellInt8(box_ptr, g, reg_max, hw, box_zp, box_scale,
                          box_nhwc, box_c, dist);

        const float cx = static_cast<float>(col) + 0.5F;
        const float cy = static_cast<float>(row) + 0.5F;
        YoloDetection d;
        d.x1 = ((cx - dist[0]) * stride_w - static_cast<float>(left_pad)) / scale;
        d.y1 = ((cy - dist[1]) * stride_h - static_cast<float>(top_pad)) / scale;
        d.x2 = ((cx + dist[2]) * stride_w - static_cast<float>(left_pad)) / scale;
        d.y2 = ((cy + dist[3]) * stride_h - static_cast<float>(top_pad)) / scale;
        d.score = best_score;
        d.class_id = best_cls;
        clampToImage(d, img_w, img_h);
        if ((d.x2 - d.x1) >= kMinBoxDim && (d.y2 - d.y1) >= kMinBoxDim) {
          candidates.push_back(d);
        }
      }
    }
  }

  return nms(std::move(candidates), iou_thresh, max_det);
}

}  // namespace circle::vision
