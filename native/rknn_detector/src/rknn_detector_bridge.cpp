#include "rknn_detector_bridge.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "circle/perception/letterbox.hpp"
#include "circle/perception/rknn_engine.hpp"
#include "circle/types/detection.hpp"
#include "circle/vision/detection_filter.hpp"
#include "circle/vision/yolo_postprocess.hpp"

namespace {

using Clock = std::chrono::steady_clock;

struct RawInference {
  std::vector<circle::vision::YoloDetection> detections;
  float preprocess_ms{0.0F};
  float inference_ms{0.0F};
  float postprocess_ms{0.0F};
  float total_ms{0.0F};
};

float elapsedMs(Clock::time_point start, Clock::time_point end) {
  return std::chrono::duration<float, std::milli>(end - start).count();
}

void setError(char* error, size_t error_size, const std::string& message) {
  if (!error || error_size == 0) {
    return;
  }
  std::snprintf(error, error_size, "%s", message.c_str());
}

CircleRknnConfig defaultConfig() {
  CircleRknnConfig config{};
  config.struct_size = sizeof(config);
  config.conf_threshold = 0.20F;
  config.iou_threshold = 0.45F;
  config.min_score = 0.25F;
  config.min_bbox_area = 0.0F;
  config.max_bbox_aspect_ratio = 3.0F;
  config.max_det = 300;
  config.core_mask = 7;
  config.temporal_gating_enabled = 0;
  config.gate_radius_px = 160.0F;
  config.reacquire_area_ratio = 0.4F;
  config.track_hint_max_misses = 30;
  return config;
}

class Detector {
 public:
  Detector(std::string model_path, CircleRknnConfig config)
      : model_path_(std::move(model_path)), config_(config) {}

  bool initialize(std::string& error) {
    if (!engine_.init(model_path_, config_.core_mask,
                      /*zero_copy_input=*/true,
                      /*zero_copy_output=*/false)) {
      error = "RKNN initialization failed for model: " + model_path_;
      return false;
    }
    if (engine_.inputWidth() <= 0 || engine_.inputHeight() <= 0 ||
        engine_.inputChannels() != 3) {
      error = "RKNN model must expose a three-channel image input";
      return false;
    }
    scratch_.resize(static_cast<size_t>(engine_.inputWidth()) *
                    static_cast<size_t>(engine_.inputHeight()) * 3U);
    buildOutputSchema();
    return true;
  }

  bool infer(const uint8_t* rgb, int width, int height, int stride,
             CircleRknnResult& result, std::string& error) {
    if (!rgb || width <= 0 || height <= 0 || stride < width * 3) {
      error = "invalid packed RGB image or stride";
      return false;
    }
    result = {};
    result.struct_size = sizeof(result);
    result.selected_index = -1;

    RawInference raw;
    if (!runModel(rgb, width, height, stride, raw, error)) {
      return false;
    }
    const auto& yolo = raw.detections;
    std::vector<circle::types::Detection> typed;
    typed.reserve(yolo.size());
    for (const auto& item : yolo) {
      circle::types::Detection detection;
      detection.cx = (item.x1 + item.x2) * 0.5F;
      detection.cy = (item.y1 + item.y2) * 0.5F;
      detection.width = item.x2 - item.x1;
      detection.height = item.y2 - item.y1;
      detection.score = item.score;
      detection.class_id = item.class_id;
      detection.class_name = "UAV";
      typed.push_back(std::move(detection));
    }

    circle::vision::DetectionFilterParams params;
    params.min_score = config_.min_score;
    params.min_bbox_area = config_.min_bbox_area;
    params.max_bbox_aspect_ratio = config_.max_bbox_aspect_ratio;
    params.target_class_name = "UAV";
    params.temporal_gating_enabled = config_.temporal_gating_enabled != 0;
    params.gate_radius_px = config_.gate_radius_px;
    params.reacquire_area_ratio = config_.reacquire_area_ratio;
    const auto filtered = circle::vision::filterDetections(
        typed, params, params.temporal_gating_enabled ? &track_hint_ : nullptr);

    int accepted_count = 0;
    for (const auto& item : filtered.results) {
      if (item.status == circle::vision::DetectionFilterStatus::kAccept) {
        ++accepted_count;
      }
    }
    result.raw_count = static_cast<int32_t>(typed.size());
    result.accepted_count = accepted_count;
    result.selected_index = filtered.best_index;
    if (filtered.best_index >= 0) {
      const auto& selected = yolo[static_cast<size_t>(filtered.best_index)];
      if (!track_hint_.valid) {
        ++track_generation_;
      }
      updateTrackHint(&typed[static_cast<size_t>(filtered.best_index)]);
      result.valid = 1;
      result.x1 = selected.x1;
      result.y1 = selected.y1;
      result.x2 = selected.x2;
      result.y2 = selected.y2;
      result.score = selected.score;
      result.class_id = selected.class_id;
      result.track_id = track_generation_;
      result.reject_code = CIRCLE_RKNN_REJECT_NONE;
    } else {
      updateTrackHint(nullptr);
      result.reject_code = typed.empty() ? CIRCLE_RKNN_REJECT_NO_CANDIDATES
                                         : CIRCLE_RKNN_REJECT_FILTERED;
    }
    result.preprocess_ms = raw.preprocess_ms;
    result.inference_ms = raw.inference_ms;
    result.postprocess_ms = raw.postprocess_ms;
    result.total_ms = raw.total_ms;
    return true;
  }

  bool inferAll(const uint8_t* rgb, int width, int height, int stride,
                CircleRknnDetection* detections, int capacity,
                CircleRknnBatchResult& result, std::string& error) {
    if (capacity < 0 || (capacity > 0 && !detections)) {
      error = "invalid detection output buffer or capacity";
      return false;
    }
    RawInference raw;
    if (!runModel(rgb, width, height, stride, raw, error)) {
      return false;
    }
    result = {};
    result.struct_size = sizeof(result);
    result.total_count = static_cast<int32_t>(raw.detections.size());
    result.count = std::min(result.total_count, capacity);
    result.truncated = result.count < result.total_count ? 1 : 0;
    result.preprocess_ms = raw.preprocess_ms;
    result.inference_ms = raw.inference_ms;
    result.postprocess_ms = raw.postprocess_ms;
    result.total_ms = raw.total_ms;
    for (int index = 0; index < result.count; ++index) {
      const auto& source = raw.detections[static_cast<size_t>(index)];
      auto& target = detections[index];
      target = {};
      target.struct_size = sizeof(target);
      target.x1 = source.x1;
      target.y1 = source.y1;
      target.x2 = source.x2;
      target.y2 = source.y2;
      target.score = source.score;
      target.class_id = source.class_id;
      target.candidate_index = index;
    }
    return true;
  }

  const std::string& outputSchema() const { return output_schema_; }

 private:
  bool runModel(const uint8_t* rgb, int width, int height, int stride,
                RawInference& raw, std::string& error) {
    if (!rgb || width <= 0 || height <= 0 || stride < width * 3) {
      error = "invalid packed RGB image or stride";
      return false;
    }
    const auto total_start = Clock::now();
    const auto pre_start = total_start;
    const uint8_t* packed = rgb;
    if (stride != width * 3) {
      packed_input_.resize(static_cast<size_t>(width) * height * 3U);
      for (int row = 0; row < height; ++row) {
        std::memcpy(packed_input_.data() + static_cast<size_t>(row) * width * 3U,
                    rgb + static_cast<size_t>(row) * stride,
                    static_cast<size_t>(width) * 3U);
      }
      packed = packed_input_.data();
    }
    const auto letterbox = circle::perception::computeLetterbox(
        width, height, engine_.inputWidth(), engine_.inputHeight());
    uint8_t* input = engine_.getInputBuffer();
    const bool use_zero_copy =
        input && engine_.getInputBufferSize() >= scratch_.size();
    uint8_t* destination = use_zero_copy ? input : scratch_.data();
    circle::perception::letterboxRgbToBuffer(
        packed, width, height, letterbox, engine_.inputWidth(),
        engine_.inputHeight(), destination);
    const auto pre_end = Clock::now();
    const auto infer_start = pre_end;
    const bool inference_ok = use_zero_copy
                                  ? engine_.runZeroCopy(false)
                                  : engine_.run(scratch_.data(), scratch_.size());
    const auto infer_end = Clock::now();
    if (!inference_ok) {
      error = "RKNN inference failed";
      return false;
    }
    const auto post_start = infer_end;
    raw.detections = postprocess(width, height, letterbox);
    const auto post_end = Clock::now();
    raw.preprocess_ms = elapsedMs(pre_start, pre_end);
    raw.inference_ms = elapsedMs(infer_start, infer_end);
    raw.postprocess_ms = elapsedMs(post_start, post_end);
    raw.total_ms = elapsedMs(total_start, post_end);
    return true;
  }

  std::vector<circle::vision::YoloDetection> postprocess(
      int image_width, int image_height,
      const circle::perception::LetterboxParams& letterbox) {
    const int count = engine_.numOutputs();
    if (count == 1) {
      return circle::vision::yoloPostprocess(
          engine_.getOutputData(0), engine_.getOutputShape(0),
          engine_.getOutputDims(0), image_width, image_height, letterbox.scale,
          letterbox.top_pad, letterbox.left_pad, config_.conf_threshold,
          config_.iou_threshold, config_.max_det);
    }

    std::vector<const float*> outputs(static_cast<size_t>(count));
    std::vector<const uint32_t*> shapes(static_cast<size_t>(count));
    std::vector<int> dimensions(static_cast<size_t>(count));
    std::unique_ptr<bool[]> is_nhwc(new bool[static_cast<size_t>(count)]);
    for (int index = 0; index < count; ++index) {
      outputs[static_cast<size_t>(index)] = engine_.getOutputData(index);
      shapes[static_cast<size_t>(index)] = engine_.getOutputShape(index);
      dimensions[static_cast<size_t>(index)] = engine_.getOutputDims(index);
      is_nhwc[static_cast<size_t>(index)] = engine_.getOutputIsNHWC(index);
    }
    return circle::vision::yoloPostprocessMultihead(
        outputs.data(), shapes.data(), dimensions.data(), is_nhwc.get(), count,
        image_width, image_height, letterbox.scale, letterbox.top_pad,
        letterbox.left_pad, config_.conf_threshold, config_.iou_threshold,
        config_.max_det, engine_.inputWidth(), engine_.inputHeight());
  }

  void updateTrackHint(const circle::types::Detection* best) {
    if (best) {
      track_hint_.valid = true;
      track_hint_.cx = best->cx;
      track_hint_.cy = best->cy;
      track_hint_.area = std::max(1.0F, best->width) *
                         std::max(1.0F, best->height);
      track_hint_misses_ = 0;
      return;
    }
    if (track_hint_.valid &&
        ++track_hint_misses_ > config_.track_hint_max_misses) {
      track_hint_ = {};
      track_hint_misses_ = 0;
    }
  }

  void buildOutputSchema() {
    std::ostringstream stream;
    stream << "{\"input\":[" << engine_.inputWidth() << ','
           << engine_.inputHeight() << ',' << engine_.inputChannels()
           << "],\"outputs\":[";
    for (int index = 0; index < engine_.numOutputs(); ++index) {
      if (index > 0) {
        stream << ',';
      }
      stream << "{\"shape\":[";
      const auto* shape = engine_.getOutputShape(index);
      for (int dimension = 0; dimension < engine_.getOutputDims(index);
           ++dimension) {
        if (dimension > 0) {
          stream << ',';
        }
        stream << shape[dimension];
      }
      stream << "],\"layout\":\""
             << (engine_.getOutputIsNHWC(index) ? "NHWC" : "NCHW")
             << "\",\"zero_point\":" << engine_.getOutputZeroPoint(index)
             << ",\"scale\":" << engine_.getOutputScale(index) << '}';
    }
    stream << "]}";
    output_schema_ = stream.str();
  }

  std::string model_path_;
  CircleRknnConfig config_{};
  circle::perception::RknnEngine engine_;
  std::vector<uint8_t> scratch_;
  std::vector<uint8_t> packed_input_;
  circle::vision::DetectionTrackHint track_hint_{};
  int track_hint_misses_{0};
  int track_generation_{0};
  std::string output_schema_;
};

}  // namespace

extern "C" uint32_t circle_rknn_abi_version(void) {
  return CIRCLE_RKNN_BRIDGE_ABI_VERSION;
}

extern "C" void* circle_rknn_create(const char* model_path,
                                     const CircleRknnConfig* config,
                                     char* error,
                                     size_t error_size) {
  try {
    if (!model_path || model_path[0] == '\0') {
      setError(error, error_size, "model_path is required");
      return nullptr;
    }
    CircleRknnConfig resolved = defaultConfig();
    if (config) {
      if (config->struct_size != sizeof(CircleRknnConfig)) {
        setError(error, error_size, "CircleRknnConfig ABI size mismatch");
        return nullptr;
      }
      resolved = *config;
    }
    auto detector = std::make_unique<Detector>(model_path, resolved);
    std::string message;
    if (!detector->initialize(message)) {
      setError(error, error_size, message);
      return nullptr;
    }
    setError(error, error_size, "");
    return detector.release();
  } catch (const std::exception& exception) {
    setError(error, error_size, exception.what());
    return nullptr;
  }
}

extern "C" int circle_rknn_infer(void* handle,
                                  const uint8_t* rgb,
                                  int32_t width,
                                  int32_t height,
                                  int32_t stride_bytes,
                                  CircleRknnResult* result,
                                  char* error,
                                  size_t error_size) {
  if (!handle || !result || result->struct_size != sizeof(CircleRknnResult)) {
    setError(error, error_size, "invalid detector handle or result ABI size");
    return -1;
  }
  try {
    std::string message;
    CircleRknnResult output{};
    if (!static_cast<Detector*>(handle)->infer(
            rgb, width, height, stride_bytes, output, message)) {
      setError(error, error_size, message);
      return -2;
    }
    *result = output;
    setError(error, error_size, "");
    return 0;
  } catch (const std::exception& exception) {
    setError(error, error_size, exception.what());
    return -3;
  }
}

extern "C" int circle_rknn_infer_all(
    void* handle, const uint8_t* rgb, int32_t width, int32_t height,
    int32_t stride_bytes, CircleRknnDetection* detections,
    int32_t detection_capacity, CircleRknnBatchResult* result, char* error,
    size_t error_size) {
  if (!handle || !result ||
      result->struct_size != sizeof(CircleRknnBatchResult)) {
    setError(error, error_size, "invalid detector handle or batch result ABI size");
    return -1;
  }
  try {
    std::string message;
    CircleRknnBatchResult output{};
    if (!static_cast<Detector*>(handle)->inferAll(
            rgb, width, height, stride_bytes, detections,
            detection_capacity, output, message)) {
      setError(error, error_size, message);
      return -2;
    }
    *result = output;
    setError(error, error_size, "");
    return 0;
  } catch (const std::exception& exception) {
    setError(error, error_size, exception.what());
    return -3;
  }
}

extern "C" const char* circle_rknn_output_schema(void* handle) {
  if (!handle) {
    return "";
  }
  return static_cast<Detector*>(handle)->outputSchema().c_str();
}

extern "C" void circle_rknn_destroy(void* handle) {
  delete static_cast<Detector*>(handle);
}
