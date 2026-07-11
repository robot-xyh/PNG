#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CIRCLE_RKNN_BRIDGE_ABI_VERSION 1

typedef struct CircleRknnConfig {
  uint32_t struct_size;
  float conf_threshold;
  float iou_threshold;
  float min_score;
  float min_bbox_area;
  float max_bbox_aspect_ratio;
  int32_t max_det;
  int32_t core_mask;
  int32_t temporal_gating_enabled;
  float gate_radius_px;
  float reacquire_area_ratio;
  int32_t track_hint_max_misses;
} CircleRknnConfig;

typedef struct CircleRknnResult {
  uint32_t struct_size;
  int32_t valid;
  float x1;
  float y1;
  float x2;
  float y2;
  float score;
  int32_t class_id;
  int32_t track_id;
  int32_t raw_count;
  int32_t accepted_count;
  int32_t selected_index;
  int32_t reject_code;
  float preprocess_ms;
  float inference_ms;
  float postprocess_ms;
  float total_ms;
} CircleRknnResult;

enum CircleRknnRejectCode {
  CIRCLE_RKNN_REJECT_NONE = 0,
  CIRCLE_RKNN_REJECT_NO_CANDIDATES = 1,
  CIRCLE_RKNN_REJECT_FILTERED = 2,
};

uint32_t circle_rknn_abi_version(void);

void* circle_rknn_create(const char* model_path,
                         const CircleRknnConfig* config,
                         char* error,
                         size_t error_size);

int circle_rknn_infer(void* handle,
                      const uint8_t* rgb,
                      int32_t width,
                      int32_t height,
                      int32_t stride_bytes,
                      CircleRknnResult* result,
                      char* error,
                      size_t error_size);

const char* circle_rknn_output_schema(void* handle);

void circle_rknn_destroy(void* handle);

#ifdef __cplusplus
}
#endif
