#pragma once

#include <cstdint>
#include <vector>

namespace circle::vision {

struct YoloDetection {
  float x1{0.0F};
  float y1{0.0F};
  float x2{0.0F};
  float y2{0.0F};
  float score{0.0F};
  int class_id{-1};
};

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
    int max_det);

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
    int model_h);

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
    int model_h);

}  // namespace circle::vision
