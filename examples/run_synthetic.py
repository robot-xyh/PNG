from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision_guidance.attitude_buffer import AttitudeHistoryBuffer
from vision_guidance.fusion import PureVisionGuidancePipeline
from vision_guidance.geometry import camera_to_body_mount
from vision_guidance.types import AttitudeSample, CameraIntrinsics, FrameDetection


def main() -> None:
    intrinsics = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)
    attitude = AttitudeHistoryBuffer(duration_s=2.0)
    for i in range(200):
        ts = i / 100.0
        attitude.push(AttitudeSample(timestamp=ts, R_IB=np.eye(3)))

    pipeline = PureVisionGuidancePipeline(
        intrinsics=intrinsics,
        R_BC=camera_to_body_mount(0.0),
        attitude_buffer=attitude,
    )

    print("frame ts center area ttc valid quality g_eval")
    for i in range(10, 90):
        ts = i / 100.0
        # Synthetic target drifts right while expanding smoothly.
        u = 310.0 + 0.8 * i
        v = 238.0 + 2.0 * math.sin(i * 0.05)
        size = 18.0 + 0.45 * i
        det = FrameDetection(
            frame_id=i,
            exposure_ts=ts,
            bbox_xyxy=(u - size / 2.0, v - size / 2.0, u + size / 2.0, v + size / 2.0),
            track_id=1,
            score=0.9,
        )
        result = pipeline.process(det)
        ttc = None if result.ttc is None else result.ttc.ttc
        print(
            i,
            f"{ts:.2f}",
            f"({u:.1f},{v:.1f})",
            f"{det.area:.1f}",
            "None" if ttc is None else f"{ttc:.2f}",
            result.guidance.valid,
            f"{result.guidance.quality:.2f}",
            np.array2string(result.guidance.g_eval, precision=4),
        )


if __name__ == "__main__":
    main()
