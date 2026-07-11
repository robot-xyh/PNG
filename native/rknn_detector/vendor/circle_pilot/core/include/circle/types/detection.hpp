#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "circle/types/time.hpp"

namespace circle::types {

struct CameraIntrinsics {
  float fx{0.0F};
  float fy{0.0F};
  float cx{0.0F};
  float cy{0.0F};
};

/** Normalized vision detection independent of ROS yolo_msgs. */
struct Detection {
  float cx{0.0F};
  float cy{0.0F};
  float width{0.0F};
  float height{0.0F};
  float score{0.0F};
  int class_id{-1};
  std::string class_name;
};

struct FrameDetection {
  uint64_t seq{0};
  TimestampNs capture_ns{0};
  TimestampNs infer_ns{0};
  TimestampNs receive_ns{0};
  bool valid{false};
  Detection detection{};
  CameraIntrinsics intrinsics{};
  uint32_t image_width{0};
  uint32_t image_height{0};
};

}  // namespace circle::types
