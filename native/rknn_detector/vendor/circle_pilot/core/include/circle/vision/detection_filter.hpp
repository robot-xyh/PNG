#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "circle/types/detection.hpp"

namespace circle::vision {

enum class DetectionFilterStatus {
  kAccept,
  kRejectScore,
  kRejectClass,
  kRejectArea,
  kRejectAspectRatio,
};

struct DetectionFilterResult {
  DetectionFilterStatus status{DetectionFilterStatus::kAccept};
  double area{0.0};
  double aspect_ratio{1.0};
  double min_area_threshold{0.0};
  double max_aspect_threshold{0.0};
};

struct DetectionFilterParams {
  double min_score{0.3};
  double min_bbox_area{0.0};
  double max_bbox_aspect_ratio{3.0};
  std::string target_class_name{};
  std::vector<std::string> target_class_names{};

  /**
   * Temporal gating (plan 1a): when a recent target exists, a candidate that is
   * BOTH far from it AND much smaller than it is treated as spurious (e.g. fixed
   * lens/sensor ghost boxes) and not selected as best, instead of letting it
   * hijack the track. In-track candidates are always eligible (picked by area).
   * Disabled by default so unrelated callers keep pure largest-area behavior.
   */
  bool temporal_gating_enabled{false};
  /** Candidate within this pixel distance of the prev target is "in-track". */
  double gate_radius_px{160.0};
  /** Out-of-track candidate must reach this fraction of prev area to steal track. */
  double reacquire_area_ratio{0.4};
};

/** Previous selected target hint for temporal gating (owned by the pipeline). */
struct DetectionTrackHint {
  bool valid{false};
  double cx{0.0};
  double cy{0.0};
  double area{0.0};
};

struct DetectionFilterOutput {
  std::vector<DetectionFilterResult> results;
  int best_index{-1};
};

inline const char* filterStatusToString(DetectionFilterStatus s) {
  switch (s) {
    case DetectionFilterStatus::kAccept:
      return "ACCEPT";
    case DetectionFilterStatus::kRejectScore:
      return "REJ:score";
    case DetectionFilterStatus::kRejectClass:
      return "REJ:class";
    case DetectionFilterStatus::kRejectArea:
      return "REJ:area";
    case DetectionFilterStatus::kRejectAspectRatio:
      return "REJ:ratio";
  }
  return "UNKNOWN";
}

inline bool isAcceptedClass(const std::string& class_name,
                            const std::string& target_class_name,
                            const std::vector<std::string>& target_class_names) {
  if (!target_class_names.empty()) {
    return std::find(target_class_names.begin(), target_class_names.end(),
                     class_name) != target_class_names.end();
  }
  if (!target_class_name.empty()) {
    return class_name == target_class_name;
  }
  return true;
}

inline DetectionFilterOutput filterDetections(
    const std::vector<circle::types::Detection>& detections,
    const DetectionFilterParams& params,
    const DetectionTrackHint* track = nullptr) {
  DetectionFilterOutput out;
  out.results.reserve(detections.size());

  const bool use_gating =
      params.temporal_gating_enabled && track != nullptr && track->valid;

  double best_area = -1.0;
  for (size_t i = 0; i < detections.size(); ++i) {
    const auto& det = detections[i];
    DetectionFilterResult fr;
    const double bw = std::max(1.0, static_cast<double>(det.width));
    const double bh = std::max(1.0, static_cast<double>(det.height));
    fr.area = bw * bh;
    fr.aspect_ratio = std::max(bw, bh) / std::min(bw, bh);
    fr.min_area_threshold = params.min_bbox_area;
    fr.max_aspect_threshold = params.max_bbox_aspect_ratio;

    const bool class_ok = isAcceptedClass(
        det.class_name, params.target_class_name, params.target_class_names);

    if (det.score < params.min_score) {
      fr.status = DetectionFilterStatus::kRejectScore;
    } else if (!class_ok) {
      fr.status = DetectionFilterStatus::kRejectClass;
    } else if (fr.area < params.min_bbox_area) {
      fr.status = DetectionFilterStatus::kRejectArea;
    } else if (fr.aspect_ratio > params.max_bbox_aspect_ratio) {
      fr.status = DetectionFilterStatus::kRejectAspectRatio;
    } else {
      fr.status = DetectionFilterStatus::kAccept;

      bool eligible = true;
      if (use_gating) {
        const double dx = static_cast<double>(det.cx) - track->cx;
        const double dy = static_cast<double>(det.cy) - track->cy;
        const bool in_track =
            (dx * dx + dy * dy) <= (params.gate_radius_px * params.gate_radius_px);
        // A far candidate may only steal the track if it is large enough to
        // plausibly be the same target reappearing (not a tiny ghost box).
        eligible = in_track ||
                   fr.area >= params.reacquire_area_ratio * track->area;
      }

      if (eligible &&
          (fr.area > best_area ||
           (fr.area == best_area &&
            det.score > (out.best_index >= 0
                             ? detections[static_cast<size_t>(out.best_index)].score
                             : 0.0F)))) {
        best_area = fr.area;
        out.best_index = static_cast<int>(i);
      }
    }
    out.results.push_back(fr);
  }
  return out;
}

}  // namespace circle::vision
