from __future__ import annotations

import argparse
import atexit
import sys
import time
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from examples.run_airsim_gimbal_vision_png import (
    PROJECT_ROOT,
    SETTINGS_EXAMPLE_PATH,
    _airsim_safety_ok,
    _bearing_deg_from_xy,
    _body_yaw_deg,
    _camera_offset_body,
    _command_duration,
    _decode_airsim_image,
    _detection_names,
    _format_names,
    _guidance_velocity,
    _experiment_fields,
    _image_kf_takeover_allowed,
    _intruder_velocity,
    _load_vehicle_origins,
    _los_fallback_allowed,
    _make_display,
    _make_preview_recorder,
    _output_paths,
    _prefer_user_mpl_toolkits,
    _preview_lines,
    _probe_camera_intrinsics,
    _record_detection_preview,
    _require_vehicles,
    _summarize_run,
    _terminal_area,
    _terminal_config_from_args,
    _terminal_image_kf_config_from_args,
    _truth_kinematics,
    _vector_xyz,
    _warmup_detection,
    _wrap_angle_deg,
    _write_run_metadata,
    _write_csv,
    _yaw_deg_from_velocity,
)
from vision_guidance.airsim_adapter import (
    AirSimDetectionConfig,
    airsim_orientation_to_R_IB,
    choose_detection,
    configure_detection_filter,
    detection_to_frame_detection,
    get_vehicle_object_collision,
    get_vehicle_pair_collision,
    get_detections,
)
from vision_guidance.attitude_buffer import AttitudeHistoryBuffer
from vision_guidance.geometry import (
    airsim_fixed_camera_to_body,
    camera_ray_from_pixel,
    los_camera_to_inertial,
    normalize,
    project_perpendicular,
)
from vision_guidance.los_filter import LOSFilterConfig, LOSKalmanFilter6D
from vision_guidance.png_eval import TTCGainSchedule
from vision_guidance.terminal_image_kf import IMAGE_KF_PREDICT, TerminalImageKF
from vision_guidance.terminal_image_kf import center_from_angle_error
from vision_guidance.terminal_extrapolation import TERMINAL_VISUAL, TerminalExtrapolator
from vision_guidance.ttc import ScaleExpansionTTC, TTCConfig
from vision_guidance.types import AttitudeSample
from vision_guidance.truth_png import integrate_velocity_command
from vision_guidance.yolo_bytetrack_detector import add_detector_args, create_detection_provider


GRAVITY_MPS2 = 9.80665


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run AirSim strapdown-camera pure-vision PNG validation.")
    parser.add_argument("--interceptor", default="Interceptor")
    parser.add_argument("--intruder", default="Intruder")
    parser.add_argument(
        "--intruder-actor",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Use a spawned scene Actor as the intruder target instead of a second AirSim vehicle.",
    )
    parser.add_argument("--intruder-actor-name", default="IntruderActor")
    parser.add_argument("--intruder-actor-asset", default="1M_Cube_Chamfer")
    parser.add_argument("--intruder-actor-scale", type=float, default=2.0)
    parser.add_argument("--intruder-actor-scale-x", type=float, default=None)
    parser.add_argument("--intruder-actor-scale-y", type=float, default=None)
    parser.add_argument("--intruder-actor-scale-z", type=float, default=None)
    parser.add_argument("--intruder-actor-physics", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--intruder-actor-blueprint", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--intruder-actor-respawn", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--collision-interceptor-pattern", action="append", default=None)
    parser.add_argument("--collision-intruder-pattern", action="append", default=None)
    parser.add_argument(
        "--collision-max-range-m",
        type=float,
        default=8.0,
        help=(
            "Accept AirSim collision as a hit only when diagnostic truth range is below this value. "
            "This rejects stale has_collided state when running multiple actor cases without reset."
        ),
    )
    parser.add_argument("--camera", default="0")
    parser.add_argument("--mesh", default="Intruder*")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fov-deg", type=float, default=120.0)
    parser.add_argument("--camera-x", type=float, default=0.0)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=-0.5)
    parser.add_argument("--camera-pitch-deg", type=float, default=0.0)
    parser.add_argument("--camera-roll-deg", type=float, default=0.0)
    parser.add_argument("--camera-yaw-deg", type=float, default=0.0)
    parser.add_argument("--intruder-speed", type=float, default=5.0)
    parser.add_argument("--intruder-vx", type=float, default=0.0)
    parser.add_argument("--intruder-vy", type=float, default=None)
    parser.add_argument("--intruder-vz", type=float, default=0.0)
    parser.add_argument("--speed-ratio", type=float, default=2.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--min-command-duration-s", type=float, default=0.10)
    parser.add_argument("--command-duration-margin-s", type=float, default=0.05)
    parser.add_argument("--max-command-duration-s", type=float, default=0.20)
    parser.add_argument("--enable-motion", action="store_true", help="Apply AirSim velocity commands.")
    parser.add_argument("--reset", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--px4-interceptor",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Treat the interceptor as PX4 SITL and use PX4-friendly prep/state estimates.",
    )
    parser.add_argument(
        "--px4-intruder",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Treat the intruder as PX4 SITL for dual-SITL tests.",
    )
    parser.add_argument(
        "--px4-max-vertical-speed",
        type=float,
        default=2.0,
        help="PX4 SITL vertical speed clamp for commanded velocities.",
    )
    parser.add_argument(
        "--px4-command-join",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Join PX4 interceptor velocity futures. Disable for high-rate Offboard setpoint streaming.",
    )
    parser.add_argument(
        "--px4-command-mode",
        choices=("velocity_simple", "velocity_yaw_rate", "mavlink_offboard", "mavlink_body_rate"),
        default="velocity_yaw_rate",
        help=(
            "PX4 SITL velocity command mapping. velocity_simple avoids AirSim "
            "drivetrain/yaw arguments because Blocks 1.8.1 PX4 ignores them in "
            "some Offboard states. mavlink_offboard sends PX4 SET_POSITION_TARGET_LOCAL_NED "
            "through AirSim's QGC UDP forwarding port for hardware HIL. "
            "mavlink_body_rate sends PX4 SET_ATTITUDE_TARGET body rates plus thrust."
        ),
    )
    parser.add_argument(
        "--px4-mavlink-url",
        default="udp:127.0.0.1:14550",
        help="MAVLink endpoint used by --px4-command-mode mavlink_offboard.",
    )
    parser.add_argument(
        "--px4-offboard-prime-s",
        type=float,
        default=2.5,
        help="Seconds of velocity setpoint streaming before requesting PX4 Offboard mode.",
    )
    parser.add_argument(
        "--px4-hil",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Hardware PX4 HIL mode. Unlike --hil-bench-safe this still permits arming and "
            "motion commands, but start geometry is computed from the AirSim/Unreal vehicle "
            "pose so the spawned target is placed in the rendered camera frame."
        ),
    )
    parser.add_argument(
        "--hil-bench-safe",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Hardware-in-the-loop bench mode: do not reset, arm, take off, climb, align, "
            "or send vehicle velocity commands. This is for camera/detector/RPC smoke tests "
            "with a physical PX4 connected."
        ),
    )
    parser.add_argument(
        "--skip-list-vehicles",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Skip AirSim listVehicles() validation; useful when PX4 HIL exposes state RPC but listVehicles() blocks.",
    )
    parser.add_argument(
        "--skip-collision-check",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Skip AirSim collision polling and leave collision log fields empty.",
    )
    parser.add_argument("--climb-to-altitude", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--intercept-altitude-m", type=float, default=50.0)
    parser.add_argument(
        "--intruder-altitude-offset-m",
        type=float,
        default=30.0,
        help="Intruder starts this many meters above the interceptor before interception.",
    )
    parser.add_argument(
        "--start-horizontal-range-m",
        type=float,
        default=None,
        help="If set, teleport vehicles to this initial horizontal range before strapdown PNG starts.",
    )
    parser.add_argument(
        "--start-forward-offset-m",
        type=float,
        default=None,
        help="Alternative to --start-horizontal-range-m: intruder forward offset from interceptor in world X.",
    )
    parser.add_argument(
        "--start-lateral-offset-m",
        type=float,
        default=-20.0,
        help="Intruder lateral offset from interceptor in world Y when start geometry is enabled.",
    )
    parser.add_argument(
        "--start-interceptor-x-m",
        type=float,
        default=0.0,
        help="Interceptor local NED X used when start geometry is enabled.",
    )
    parser.add_argument(
        "--start-interceptor-y-m",
        type=float,
        default=0.0,
        help="Interceptor local NED Y used when start geometry is enabled.",
    )
    parser.add_argument("--start-geometry-settle-s", type=float, default=0.5)
    parser.add_argument("--climb-speed", type=float, default=5.0)
    parser.add_argument("--climb-timeout-s", type=float, default=60.0)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--settle-speed", type=float, default=0.5)
    parser.add_argument("--settle-timeout-s", type=float, default=8.0)
    parser.add_argument("--hit-radius-m", type=float, default=1.0, help="Deprecated; AirSim collision is the success criterion.")
    parser.add_argument("--max-vision-lateral-speed", type=float, default=4.0)
    parser.add_argument("--max-vision-vertical-speed", type=float, default=3.0)
    parser.add_argument("--coast-timeout-s", type=float, default=0.5)
    parser.add_argument("--blind-push-timeout-s", type=float, default=1.0)
    parser.add_argument("--terminal-bbox-area-ratio", type=float, default=0.25)
    parser.add_argument("--terminal-extrapolation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--terminal-enter-area-ratio", type=float, default=0.20)
    parser.add_argument("--terminal-soft-enter-area-ratio", type=float, default=0.05)
    parser.add_argument("--terminal-cutoff-area-ratio", type=float, default=0.60)
    parser.add_argument("--terminal-gimbal-limit-area-ratio", type=float, default=0.05)
    parser.add_argument("--terminal-cutoff-miss-count", type=int, default=3)
    parser.add_argument("--terminal-min-tracking-time-s", type=float, default=0.20)
    parser.add_argument("--terminal-confidence-min-score", type=float, default=0.35)
    parser.add_argument("--terminal-max-measurement-age-s", type=float, default=0.12)
    parser.add_argument("--terminal-blind-duration-s", type=float, default=0.30)
    parser.add_argument("--terminal-command-average-window-s", type=float, default=0.10)
    parser.add_argument("--terminal-command-decay-tau-s", type=float, default=0.18)
    parser.add_argument("--terminal-trend-bias-gain", type=float, default=0.10)
    parser.add_argument("--terminal-trend-bias-max-mps", type=float, default=1.5)
    parser.add_argument("--terminal-pitch-up-bias-mps", type=float, default=0.8)
    parser.add_argument("--terminal-abort-on-tilt-hardcap", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--terminal-image-kf", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--terminal-image-kf-max-predict-s", type=float, default=0.35)
    parser.add_argument("--terminal-image-kf-meas-noise-rad", type=float, default=0.006)
    parser.add_argument("--terminal-image-kf-accel-noise-rad-s2", type=float, default=8.0)
    parser.add_argument("--terminal-image-kf-innovation-reject-rad", type=float, default=0.20)
    parser.add_argument("--terminal-image-kf-max-angle-rad", type=float, default=1.0)
    parser.add_argument("--terminal-image-kf-max-rate-rad-s", type=float, default=8.0)
    parser.add_argument(
        "--terminal-image-kf-guidance",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "When image KF is predicting during short visual loss, rebuild strapdown LOS/LOS-rate "
            "from the predicted image angles and keep velocity guidance active."
        ),
    )
    parser.add_argument(
        "--ignore-pseudo-track-switch",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Do not reset LOS/TTC/KF state when YOLO pseudo track id -1 alternates with a real ByteTrack id.",
    )
    parser.add_argument(
        "--terminal-yaw-rate-extrapolation",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Keep a decaying short-window yaw-rate command during terminal blind push.",
    )
    parser.add_argument("--terminal-yaw-rate-average-window-s", type=float, default=0.10)
    parser.add_argument("--terminal-yaw-rate-decay-tau-s", type=float, default=0.18)
    parser.add_argument("--terminal-yaw-rate-scale", type=float, default=0.70)
    parser.add_argument(
        "--reject-top-clipped-pitch",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Treat top-edge clipped bbox center pitch as invalid and enter terminal prediction/hold.",
    )
    parser.add_argument("--yaw-control", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--yaw-error-gain", type=float, default=2.5)
    parser.add_argument("--max-yaw-rate-deg", type=float, default=90.0)
    parser.add_argument("--initial-truth-align", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--initial-align-timeout-s", type=float, default=8.0)
    parser.add_argument("--initial-align-margin-deg", type=float, default=3.0)
    parser.add_argument("--detection-radius-cm", type=float, default=50000.0)
    parser.add_argument(
        "--diagnostic-truth",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Continuously log intruder truth for offline evaluation only; guidance logic does not consume it.",
    )
    parser.add_argument("--ttc-min-area", type=float, default=0.0)
    parser.add_argument("--ttc-max-s", type=float, default=120.0)
    parser.add_argument(
        "--ttc-soft-guidance",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "For ttc_png, keep LOS/Vm guidance active when TTC is unavailable; "
            "TTC only schedules gain when valid."
        ),
    )
    parser.add_argument(
        "--ttc-soft-min-gain-scale",
        type=float,
        default=0.55,
        help="Minimum LOS+Vm gain scale used by --ttc-soft-guidance when TTC is valid and large.",
    )
    parser.add_argument(
        "--guidance-law",
        choices=("ttc_png", "fixed_vm_png"),
        default="ttc_png",
        help=(
            "Guidance gain source. ttc_png uses scale-expansion TTC scheduling; "
            "fixed_vm_png ignores TTC for guidance and uses N*Vm with Vm=speed_ratio*intruder_speed."
        ),
    )
    parser.add_argument(
        "--navigation-constant",
        type=float,
        default=3.0,
        help="Navigation constant N used by --guidance-law fixed_vm_png.",
    )
    parser.add_argument(
        "--guidance-output-mode",
        choices=("velocity_bias", "accel_integral", "accel_body_rate"),
        default="velocity_bias",
        help=(
            "velocity_bias preserves the legacy behavior that adds PNG correction directly to velocity. "
            "accel_integral treats PNG as acceleration/overload and explicitly integrates it to a velocity setpoint. "
            "accel_body_rate maps PNG acceleration to PX4 body-rate/thrust setpoints."
        ),
    )
    parser.add_argument(
        "--max-guidance-accel-mps2",
        type=float,
        default=15.0,
        help="Norm limit for acceleration-form PNG in --guidance-output-mode accel_integral or accel_body_rate.",
    )
    parser.add_argument(
        "--min-speed-ratio",
        type=float,
        default=0.60,
        help="Minimum speed ratio relative to scheduled speed cap for accel_integral velocity integration.",
    )
    parser.add_argument(
        "--accel-integral-reset-on-invalid",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Reset accel_integral base velocity to measured vehicle velocity when guidance is invalid.",
    )
    parser.add_argument("--body-rate-max-tilt-deg", type=float, default=20.0)
    parser.add_argument("--body-rate-roll-gain", type=float, default=1.0)
    parser.add_argument("--body-rate-pitch-gain", type=float, default=1.0)
    parser.add_argument("--body-rate-attitude-p", type=float, default=4.0)
    parser.add_argument("--body-rate-max-roll-rate-deg", type=float, default=60.0)
    parser.add_argument("--body-rate-max-pitch-rate-deg", type=float, default=60.0)
    parser.add_argument("--body-rate-hover-thrust", type=float, default=0.50)
    parser.add_argument("--body-rate-thrust-gain", type=float, default=0.50)
    parser.add_argument("--body-rate-min-thrust", type=float, default=0.25)
    parser.add_argument("--body-rate-max-thrust", type=float, default=0.75)
    parser.add_argument("--body-rate-speed-hold-gain", type=float, default=1.2)
    parser.add_argument("--body-rate-speed-hold-max-accel-mps2", type=float, default=6.0)
    parser.add_argument("--body-rate-total-accel-limit-mps2", type=float, default=18.0)
    parser.add_argument(
        "--bbox-noise",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Inject repeatable synthetic noise into AirSim detection bbox before LOS/TTC processing.",
    )
    parser.add_argument("--bbox-center-noise-px", type=float, default=3.0)
    parser.add_argument("--bbox-area-noise-ratio", type=float, default=0.08)
    parser.add_argument("--bbox-noise-seed", type=int, default=20260617)
    parser.add_argument(
        "--los-filter",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use the 6D LOS Kalman filter. Disable it for noiseless AirSim detection/pose analysis.",
    )
    parser.add_argument("--los-filter-process-lambda", type=float, default=1e-4)
    parser.add_argument("--los-filter-process-lambda-dot", type=float, default=5e-3)
    parser.add_argument("--los-filter-measurement-noise", type=float, default=5e-3)
    parser.add_argument(
        "--los-filter-innovation-reject",
        type=float,
        default=0.25,
        help="6D LOS Kalman innovation norm gate. Increase for high terminal LOS-rate or noisy YOLO boxes.",
    )
    parser.add_argument("--allow-los-fallback", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--los-fallback-gain", type=float, default=0.5)
    parser.add_argument(
        "--frame-guard",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Prioritize keeping the target in the strapdown camera frame before aggressive PNG correction.",
    )
    parser.add_argument("--frame-guard-enter-error-ratio", type=float, default=0.42)
    parser.add_argument("--frame-guard-exit-error-ratio", type=float, default=0.28)
    parser.add_argument("--frame-guard-area-mid-ratio", type=float, default=0.0012)
    parser.add_argument("--frame-guard-area-ratio", type=float, default=0.004)
    parser.add_argument("--frame-guard-ttc-mid-s", type=float, default=2.5)
    parser.add_argument("--frame-guard-ttc-terminal-s", type=float, default=1.2)
    parser.add_argument("--frame-guard-hold-s", type=float, default=0.65)
    parser.add_argument("--frame-guard-min-speed-ratio", type=float, default=1.30)
    parser.add_argument("--frame-guard-mid-speed-ratio", type=float, default=1.60)
    parser.add_argument("--frame-guard-lateral-scale", type=float, default=0.55)
    parser.add_argument("--frame-guard-vertical-scale", type=float, default=0.85)
    parser.add_argument("--frame-guard-yaw-gain-scale", type=float, default=1.45)
    parser.add_argument("--frame-guard-max-yaw-rate-deg", type=float, default=60.0)
    parser.add_argument("--frame-guard-vertical-error-gain", type=float, default=0.012)
    parser.add_argument(
        "--frame-centering",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Hard strapdown camera frame-keeping state. When the target approaches the image edge "
            "or is briefly lost, reduce speed, suppress lateral PNG velocity, and keep/predict yaw-rate."
        ),
    )
    parser.add_argument("--frame-centering-enter-error-ratio", type=float, default=0.52)
    parser.add_argument("--frame-centering-terminal-error-ratio", type=float, default=0.68)
    parser.add_argument("--frame-centering-area-ratio", type=float, default=0.006)
    parser.add_argument("--frame-centering-loss-hold-s", type=float, default=0.80)
    parser.add_argument("--frame-centering-speed-ratio", type=float, default=1.45)
    parser.add_argument("--terminal-capture-speed-ratio", type=float, default=1.20)
    parser.add_argument("--frame-centering-min-forward-ratio", type=float, default=0.55)
    parser.add_argument("--frame-centering-lateral-scale", type=float, default=0.20)
    parser.add_argument("--terminal-capture-lateral-scale", type=float, default=0.08)
    parser.add_argument("--frame-centering-max-lateral-speed", type=float, default=1.20)
    parser.add_argument("--terminal-capture-max-lateral-speed", type=float, default=0.55)
    parser.add_argument("--frame-centering-yaw-gain-scale", type=float, default=1.20)
    parser.add_argument("--terminal-capture-yaw-gain-scale", type=float, default=1.45)
    parser.add_argument("--frame-centering-max-yaw-rate-deg", type=float, default=70.0)
    parser.add_argument("--terminal-capture-max-yaw-rate-deg", type=float, default=60.0)
    parser.add_argument("--frame-centering-yaw-hold-window-s", type=float, default=0.20)
    parser.add_argument("--frame-centering-yaw-hold-decay-tau-s", type=float, default=0.30)
    parser.add_argument(
        "--frame-centering-loss-hold-last-velocity",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="During short target loss, start from the previous final velocity command before applying frame-centering limits.",
    )
    add_detector_args(parser)
    parser.add_argument(
        "--shadow-airsim-detect",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Also run AirSim built-in detection on the same frame for diagnostics. "
            "The shadow detector is logged only and never feeds guidance."
        ),
    )
    parser.add_argument("--detection-warmup-s", type=float, default=1.0)
    parser.add_argument("--show-window", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--window-scale", type=float, default=0.75)
    parser.add_argument("--record-preview", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--preview-dir", default="")
    parser.add_argument(
        "--preview-every-s",
        type=float,
        default=0.0,
        help="When positive, save one annotated recognition frame per this many simulation seconds.",
    )
    parser.add_argument("--preview-every-n", type=int, default=20)
    parser.add_argument("--preview-max-frames", type=int, default=12)
    parser.add_argument("--preview-near-terminal-only", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--print-every-n", type=int, default=10)
    parser.add_argument("--settings-path", default=str(SETTINGS_EXAMPLE_PATH))
    parser.add_argument("--list-vehicles", action="store_true")
    parser.add_argument("--trajectory-dir", default=str(PROJECT_ROOT / "logs"))
    parser.add_argument("--trajectory-prefix", default="")
    parser.add_argument("--no-plot", action="store_true")
    return parser.parse_args()


def _validate_runtime_args(args) -> None:
    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be positive")
    if args.speed_ratio <= 0.0:
        raise SystemExit("--speed-ratio must be positive")
    if args.navigation_constant <= 0.0:
        raise SystemExit("--navigation-constant must be positive")
    if args.guidance_output_mode == "accel_body_rate":
        if not args.px4_interceptor:
            raise SystemExit("--guidance-output-mode accel_body_rate requires --px4-interceptor")
        if args.px4_command_mode != "mavlink_body_rate":
            raise SystemExit("--guidance-output-mode accel_body_rate requires --px4-command-mode mavlink_body_rate")
    if args.px4_command_mode == "mavlink_body_rate" and args.guidance_output_mode != "accel_body_rate":
        raise SystemExit("--px4-command-mode mavlink_body_rate requires --guidance-output-mode accel_body_rate")
    if args.body_rate_min_thrust > args.body_rate_max_thrust:
        raise SystemExit("--body-rate-min-thrust cannot exceed --body-rate-max-thrust")
    if args.los_filter_process_lambda < 0.0:
        raise SystemExit("--los-filter-process-lambda must be non-negative")
    if args.los_filter_process_lambda_dot < 0.0:
        raise SystemExit("--los-filter-process-lambda-dot must be non-negative")
    if args.los_filter_measurement_noise <= 0.0:
        raise SystemExit("--los-filter-measurement-noise must be positive")
    if args.los_filter_innovation_reject <= 0.0:
        raise SystemExit("--los-filter-innovation-reject must be positive")


def _output_paths_strapdown(args) -> tuple[Path, Path]:
    if args.trajectory_prefix:
        return _output_paths(args)
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    base = Path(args.trajectory_dir)
    return base / f"strapdown_vision_png_{timestamp}.csv", base / f"strapdown_vision_png_{timestamp}.png"


def _empty_collision_snapshot():
    from types import SimpleNamespace

    return SimpleNamespace(
        collided=False,
        reason="collision_check_disabled",
        interceptor_time_stamp_ns=0,
        intruder_time_stamp_ns=0,
        interceptor_object_name="",
        intruder_object_name="",
    )


def _fixed_camera_pose(airsim_module, args):
    return airsim_module.Pose(
        airsim_module.Vector3r(args.camera_x, args.camera_y, args.camera_z),
        airsim_module.to_quaternion(
            -np.deg2rad(float(args.camera_pitch_deg)),
            np.deg2rad(float(args.camera_roll_deg)),
            np.deg2rad(float(args.camera_yaw_deg)),
        ),
    )


def _fixed_camera_R_BC(args) -> np.ndarray:
    return airsim_fixed_camera_to_body(
        yaw_rad=np.deg2rad(float(args.camera_yaw_deg)),
        pitch_rad=np.deg2rad(float(args.camera_pitch_deg)),
        roll_rad=np.deg2rad(float(args.camera_roll_deg)),
    )


def _apply_bbox_noise(frame_detection, intrinsics, rng: np.random.Generator, args):
    if not args.bbox_noise:
        return frame_detection, {
            "dx": 0.0,
            "dy": 0.0,
            "area_scale": 1.0,
            "raw_bbox": frame_detection.bbox_xyxy,
            "noisy_bbox": frame_detection.bbox_xyxy,
        }

    x1, y1, x2, y2 = frame_detection.bbox_xyxy
    width = max(1.0e-6, x2 - x1)
    height = max(1.0e-6, y2 - y1)
    cx = 0.5 * (x1 + x2)
    cy = 0.5 * (y1 + y2)

    center_sigma = max(0.0, float(args.bbox_center_noise_px))
    area_sigma = max(0.0, float(args.bbox_area_noise_ratio))
    dx = float(rng.normal(0.0, center_sigma)) if center_sigma > 0.0 else 0.0
    dy = float(rng.normal(0.0, center_sigma)) if center_sigma > 0.0 else 0.0
    area_scale = 1.0 + (float(rng.normal(0.0, area_sigma)) if area_sigma > 0.0 else 0.0)
    area_scale = max(0.05, area_scale)
    side_scale = float(np.sqrt(area_scale))

    noisy_cx = cx + dx
    noisy_cy = cy + dy
    noisy_w = width * side_scale
    noisy_h = height * side_scale
    nx1 = max(0.0, noisy_cx - 0.5 * noisy_w)
    ny1 = max(0.0, noisy_cy - 0.5 * noisy_h)
    nx2 = min(float(intrinsics.width), noisy_cx + 0.5 * noisy_w)
    ny2 = min(float(intrinsics.height), noisy_cy + 0.5 * noisy_h)
    if nx2 <= nx1:
        nx2 = min(float(intrinsics.width), nx1 + 1.0)
    if ny2 <= ny1:
        ny2 = min(float(intrinsics.height), ny1 + 1.0)

    noisy = type(frame_detection)(
        frame_id=frame_detection.frame_id,
        exposure_ts=frame_detection.exposure_ts,
        bbox_xyxy=(float(nx1), float(ny1), float(nx2), float(ny2)),
        track_id=frame_detection.track_id,
        score=frame_detection.score,
    )
    return noisy, {
        "dx": dx,
        "dy": dy,
        "area_scale": area_scale,
        "raw_bbox": frame_detection.bbox_xyxy,
        "noisy_bbox": noisy.bbox_xyxy,
    }


def _initial_truth_yaw_deg(client, args, origins: dict[str, np.ndarray]) -> float:
    interceptor_kin = _guidance_kinematics(client, args.interceptor, args)
    interceptor_pos = _vehicle_truth_position(client, args.interceptor, origins, kinematics=interceptor_kin)
    intruder_pos = _intruder_truth_position(client, args, origins)
    state = client.getMultirotorState(vehicle_name=args.interceptor)
    R_IB = airsim_orientation_to_R_IB(state.kinematics_estimated.orientation)
    camera_pos = interceptor_pos + R_IB @ _camera_offset_body(args)
    relative = intruder_pos - camera_pos
    yaw_deg = _bearing_deg_from_xy(relative[:2])
    print(
        "Initial truth strapdown yaw alignment: "
        f"camera_offset_body={np.array2string(_camera_offset_body(args), precision=2)}, "
        f"relative_I={np.array2string(relative, precision=2)}, yaw={yaw_deg:.2f}deg"
    )
    return yaw_deg


def _align_body_yaw_to_target(client, airsim_module, args, origins: dict[str, np.ndarray]) -> float:
    yaw_deg = _initial_truth_yaw_deg(client, args, origins)
    if args.px4_interceptor:
        print(f"PX4 initial yaw alignment target={yaw_deg:.2f}deg; rotating with yaw-rate setpoints.")
        deadline = time.monotonic() + max(1.0, float(args.initial_align_timeout_s))
        command_dt = 0.20
        last_print = 0.0
        actual_yaw_deg = float("nan")
        while time.monotonic() < deadline:
            _ensure_px4_api_control(client, args, args.interceptor)
            state = client.getMultirotorState(vehicle_name=args.interceptor)
            actual_yaw_deg = _body_yaw_deg(airsim_module, state.kinematics_estimated.orientation)
            yaw_error_deg = _wrap_angle_deg(yaw_deg - actual_yaw_deg)
            if abs(yaw_error_deg) <= max(0.0, float(args.initial_align_margin_deg)):
                break
            yaw_rate_cmd = float(
                np.clip(
                    float(args.yaw_error_gain) * yaw_error_deg,
                    -float(args.max_yaw_rate_deg),
                    float(args.max_yaw_rate_deg),
                )
            )
            client.moveByVelocityAsync(
                0.0,
                0.0,
                0.0,
                duration=command_dt,
                drivetrain=airsim_module.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim_module.YawMode(is_rate=True, yaw_or_rate=yaw_rate_cmd),
                vehicle_name=args.interceptor,
            )
            now = time.monotonic()
            if now - last_print >= 1.0:
                print(
                    f"px4_initial_yaw_status target={yaw_deg:.2f}deg actual={actual_yaw_deg:.2f}deg "
                    f"error={yaw_error_deg:.2f}deg yaw_rate_cmd={yaw_rate_cmd:.2f}deg/s"
                )
                last_print = now
            time.sleep(command_dt)
        for _ in range(3):
            client.moveByVelocityAsync(
                0.0,
                0.0,
                0.0,
                duration=command_dt,
                drivetrain=airsim_module.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim_module.YawMode(is_rate=True, yaw_or_rate=0.0),
                vehicle_name=args.interceptor,
            )
            time.sleep(0.05)
        state = client.getMultirotorState(vehicle_name=args.interceptor)
        actual_yaw_deg = _body_yaw_deg(airsim_module, state.kinematics_estimated.orientation)
        yaw_error_deg = _wrap_angle_deg(yaw_deg - actual_yaw_deg)
        print(
            f"Initial PX4 body yaw after alignment: commanded={yaw_deg:.2f}deg, "
            f"actual={actual_yaw_deg:.2f}deg, error={yaw_error_deg:.2f}deg"
        )
        return yaw_deg
    try:
        client.rotateToYawAsync(
            yaw_deg,
            timeout_sec=args.initial_align_timeout_s,
            margin=args.initial_align_margin_deg,
            vehicle_name=args.interceptor,
        ).join()
    except Exception as exc:
        if not args.px4_interceptor:
            raise
        print(f"px4_rotate_yaw_warning={exc}; falling back to velocity yaw command")
        deadline = time.monotonic() + max(1.0, float(args.initial_align_timeout_s))
        while time.monotonic() < deadline:
            client.moveByVelocityAsync(
                0.0,
                0.0,
                0.0,
                duration=0.25,
                yaw_mode=airsim_module.YawMode(is_rate=False, yaw_or_rate=yaw_deg),
                vehicle_name=args.interceptor,
            )
            time.sleep(0.25)
    state = client.getMultirotorState(vehicle_name=args.interceptor)
    actual_yaw_deg = _body_yaw_deg(airsim_module, state.kinematics_estimated.orientation)
    print(f"Initial body yaw after alignment: commanded={yaw_deg:.2f}deg, actual={actual_yaw_deg:.2f}deg")
    return yaw_deg


def _start_geometry_enabled(args) -> bool:
    return args.start_horizontal_range_m is not None or args.start_forward_offset_m is not None


def _local_start_z(vehicle: str, args) -> float:
    altitude = abs(float(args.intercept_altitude_m))
    if vehicle == args.intruder:
        altitude += float(args.intruder_altitude_offset_m)
    return -altitude


def _interceptor_start_local(args, origins: dict[str, np.ndarray]) -> np.ndarray:
    local = np.array(
        [float(args.start_interceptor_x_m), float(args.start_interceptor_y_m), _local_start_z(args.interceptor, args)],
        dtype=float,
    )
    if args.px4_interceptor:
        # PX4 kinematics are local to the spawned vehicle origin. Use the settings
        # origin to place the vehicle at the requested world altitude.
        local[2] = -abs(float(args.intercept_altitude_m)) - origins.get(args.interceptor, np.zeros(3, dtype=float))[2]
    return local


def _is_px4_vehicle(vehicle_name: str, args) -> bool:
    if getattr(args, "intruder_actor", False) and vehicle_name == getattr(args, "intruder", ""):
        return False
    return (getattr(args, "px4_interceptor", False) and vehicle_name == args.interceptor) or (
        getattr(args, "px4_intruder", False) and vehicle_name == args.intruder
    )


def _start_geometry_offsets(args) -> tuple[float, float, float]:
    lateral = float(args.start_lateral_offset_m)
    if args.start_horizontal_range_m is not None and args.start_forward_offset_m is not None:
        raise SystemExit("Use either --start-horizontal-range-m or --start-forward-offset-m, not both.")
    if args.start_horizontal_range_m is not None:
        horizontal_range = float(args.start_horizontal_range_m)
        if horizontal_range <= 0.0:
            raise SystemExit("--start-horizontal-range-m must be positive")
        if abs(lateral) > horizontal_range:
            raise SystemExit("--start-lateral-offset-m magnitude cannot exceed --start-horizontal-range-m")
        forward = float(np.sqrt(max(0.0, horizontal_range * horizontal_range - lateral * lateral)))
    elif args.start_forward_offset_m is not None:
        forward = float(args.start_forward_offset_m)
        horizontal_range = float(np.hypot(forward, lateral))
    else:
        forward = 0.0
        horizontal_range = 0.0
    return forward, lateral, horizontal_range


def _guidance_kinematics(client, vehicle_name: str, args):
    if _is_px4_vehicle(vehicle_name, args):
        return client.getMultirotorState(vehicle_name=vehicle_name).kinematics_estimated
    return _truth_kinematics(client, vehicle_name)


def _world_position(kinematics, vehicle_name: str, origins: dict[str, np.ndarray]) -> np.ndarray:
    return origins.get(vehicle_name, np.zeros(3, dtype=float)) + _vector_xyz(kinematics.position)


def _object_world_position(client, object_name: str) -> np.ndarray:
    pose = client.simGetObjectPose(object_name)
    return _vector_xyz(pose.position)


def _vehicle_truth_position(client, vehicle_name: str, origins: dict[str, np.ndarray], kinematics=None) -> np.ndarray:
    try:
        return _object_world_position(client, vehicle_name)
    except Exception:
        if kinematics is None:
            kinematics = _truth_kinematics(client, vehicle_name)
        return _world_position(kinematics, vehicle_name, origins)


def _actor_name(args) -> str:
    return str(getattr(args, "intruder_actor_name", "") or args.intruder)


def _actor_pose(airsim_module, position: np.ndarray, yaw_deg: float = 0.0):
    return airsim_module.Pose(
        airsim_module.Vector3r(float(position[0]), float(position[1]), float(position[2])),
        airsim_module.to_quaternion(0.0, 0.0, np.deg2rad(float(yaw_deg))),
    )


def _actor_position_from_pose(pose) -> np.ndarray:
    return _vector_xyz(pose.position)


def _actor_scale_xyz(args) -> tuple[float, float, float]:
    base = float(getattr(args, "intruder_actor_scale", 1.0))
    return (
        float(getattr(args, "intruder_actor_scale_x", None) if getattr(args, "intruder_actor_scale_x", None) is not None else base),
        float(getattr(args, "intruder_actor_scale_y", None) if getattr(args, "intruder_actor_scale_y", None) is not None else base),
        float(getattr(args, "intruder_actor_scale_z", None) if getattr(args, "intruder_actor_scale_z", None) is not None else base),
    )


def _intruder_truth_position(client, args, origins: dict[str, np.ndarray]) -> np.ndarray:
    if getattr(args, "intruder_actor", False):
        return _object_world_position(client, _actor_name(args))
    intruder_kin = _guidance_kinematics(client, args.intruder, args)
    return _vehicle_truth_position(client, args.intruder, origins, kinematics=intruder_kin)


def _spawn_or_move_intruder_actor(client, airsim_module, args, world_position: np.ndarray, yaw_deg: float) -> None:
    object_name = _actor_name(args)
    pose = _actor_pose(airsim_module, world_position, yaw_deg)
    if args.intruder_actor_respawn:
        try:
            client.simDestroyObject(object_name)
        except Exception:
            pass
    else:
        try:
            moved = client.simSetObjectPose(object_name, pose, teleport=True)
            if moved is None or bool(moved):
                return
        except Exception:
            pass
        try:
            scene_objects = client.simListSceneObjects(f"{object_name}.*")
        except Exception:
            scene_objects = []
        if object_name in scene_objects:
            client.simSetObjectPose(object_name, pose, teleport=True)
            return
    spawned = False
    try:
        scale_x, scale_y, scale_z = _actor_scale_xyz(args)
        scale = airsim_module.Vector3r(
            scale_x,
            scale_y,
            scale_z,
        )
        spawned = bool(
            client.simSpawnObject(
                object_name,
                args.intruder_actor_asset,
                pose,
                scale,
                bool(args.intruder_actor_physics),
                bool(args.intruder_actor_blueprint),
            )
        )
    except Exception as exc:
        print(f"intruder_actor_spawn_warning={exc}; trying simSetObjectPose on existing object")
    if not spawned:
        try:
            client.simSetObjectPose(object_name, pose, teleport=True)
        except Exception as exc:
            raise SystemExit(
                f"Failed to spawn or move intruder actor '{object_name}' with asset "
                f"'{args.intruder_actor_asset}': {exc}"
            ) from exc


def _move_intruder_actor(client, airsim_module, args, world_position: np.ndarray, yaw_deg: float) -> None:
    client.simSetObjectPose(_actor_name(args), _actor_pose(airsim_module, world_position, yaw_deg), teleport=True)


def _actor_collision_patterns(args) -> tuple[str, ...]:
    patterns = list(args.collision_intruder_pattern or [])
    object_name = _actor_name(args)
    asset = str(args.intruder_actor_asset or "")
    patterns.extend([object_name, f"{object_name}*", asset, f"{asset}*"])
    return tuple(pattern for pattern in patterns if pattern)


def _configure_actor_detection_aliases(client, airsim_module, config: AirSimDetectionConfig, args) -> None:
    if not args.intruder_actor:
        return
    image_type = getattr(airsim_module.ImageType, config.image_type_name)
    aliases = {
        str(args.mesh or ""),
        _actor_name(args),
    }
    for alias in sorted(pattern for pattern in aliases if pattern):
        try:
            client.simAddDetectionFilterMeshName(
                config.camera_name,
                image_type,
                alias,
                vehicle_name=config.vehicle_name,
            )
        except Exception as exc:
            print(f"actor_detection_alias_warning alias={alias}: {exc}")


def _shadow_airsim_detection(client, config: AirSimDetectionConfig, active_name: Optional[str], intrinsics) -> dict[str, float | int | str]:
    start = time.monotonic()
    try:
        detections = list(get_detections(client, config))
        selected = choose_detection(detections, preferred_name=active_name)
    except Exception as exc:
        return {
            "shadow_airsim_enabled": 1,
            "shadow_airsim_elapsed_s": time.monotonic() - start,
            "shadow_airsim_fps": 0.0,
            "shadow_airsim_detection_count": 0,
            "shadow_airsim_detected": 0,
            "shadow_airsim_target_name": "",
            "shadow_airsim_bbox_area": 0.0,
            "shadow_airsim_pixel_error_x": 0.0,
            "shadow_airsim_pixel_error_y": 0.0,
            "shadow_airsim_reject_reason": f"shadow_exception:{exc}",
        }

    elapsed = time.monotonic() - start
    fps = 1.0 / max(1.0e-6, elapsed)
    if selected is None:
        return {
            "shadow_airsim_enabled": 1,
            "shadow_airsim_elapsed_s": elapsed,
            "shadow_airsim_fps": fps,
            "shadow_airsim_detection_count": len(detections),
            "shadow_airsim_detected": 0,
            "shadow_airsim_target_name": "",
            "shadow_airsim_bbox_area": 0.0,
            "shadow_airsim_pixel_error_x": 0.0,
            "shadow_airsim_pixel_error_y": 0.0,
            "shadow_airsim_reject_reason": "no_detection",
        }

    box = selected.box2D
    x1 = float(box.min.x_val)
    y1 = float(box.min.y_val)
    x2 = float(box.max.x_val)
    y2 = float(box.max.y_val)
    area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    center_x = 0.5 * (x1 + x2)
    center_y = 0.5 * (y1 + y2)
    return {
        "shadow_airsim_enabled": 1,
        "shadow_airsim_elapsed_s": elapsed,
        "shadow_airsim_fps": fps,
        "shadow_airsim_detection_count": len(detections),
        "shadow_airsim_detected": 1,
        "shadow_airsim_target_name": str(getattr(selected, "name", "") or ""),
        "shadow_airsim_bbox_area": area,
        "shadow_airsim_pixel_error_x": center_x - float(intrinsics.cx),
        "shadow_airsim_pixel_error_y": center_y - float(intrinsics.cy),
        "shadow_airsim_reject_reason": "",
    }


def _empty_shadow_airsim_stats() -> dict[str, float | int | str]:
    return {
        "shadow_airsim_enabled": 0,
        "shadow_airsim_elapsed_s": 0.0,
        "shadow_airsim_fps": 0.0,
        "shadow_airsim_detection_count": "",
        "shadow_airsim_detected": "",
        "shadow_airsim_target_name": "",
        "shadow_airsim_bbox_area": "",
        "shadow_airsim_pixel_error_x": "",
        "shadow_airsim_pixel_error_y": "",
        "shadow_airsim_reject_reason": "",
    }


def _px4_local_target_z(vehicle: str, world_target_z: float, origins: dict[str, np.ndarray]) -> float:
    return float(world_target_z - origins.get(vehicle, np.zeros(3, dtype=float))[2])


def _px4_limited_velocity(v_cmd: np.ndarray, args) -> np.ndarray:
    command = np.array(v_cmd, dtype=float)
    vertical_limit = max(0.0, float(args.px4_max_vertical_speed))
    if vertical_limit > 0.0:
        command[2] = float(np.clip(command[2], -vertical_limit, vertical_limit))
    return command


def _vehicle_euler_rad(airsim_module, orientation) -> tuple[float, float, float]:
    pitch, roll, yaw = airsim_module.to_eularian_angles(orientation)
    return float(roll), float(pitch), float(yaw)


def _body_rate_neutral_thrust(args) -> float:
    return float(np.clip(float(args.body_rate_hover_thrust), 0.0, 1.0))


def _body_rate_command_from_accel(
    acceleration_I: np.ndarray,
    R_IB: np.ndarray,
    roll_rad: float,
    pitch_rad: float,
    yaw_rate_deg_s: float,
    args,
) -> dict[str, float | np.ndarray]:
    accel_I = np.asarray(acceleration_I, dtype=float)
    if not np.all(np.isfinite(accel_I)):
        accel_I = np.zeros(3, dtype=float)
    R_BI = np.asarray(R_IB, dtype=float).T
    accel_B = R_BI @ accel_I

    max_tilt = np.deg2rad(max(0.0, float(args.body_rate_max_tilt_deg)))
    roll_sp = float(
        np.clip(
            float(args.body_rate_roll_gain) * float(accel_B[1]) / GRAVITY_MPS2,
            -max_tilt,
            max_tilt,
        )
    )
    pitch_sp = float(
        np.clip(
            -float(args.body_rate_pitch_gain) * float(accel_B[0]) / GRAVITY_MPS2,
            -max_tilt,
            max_tilt,
        )
    )

    attitude_p = max(0.0, float(args.body_rate_attitude_p))
    max_roll_rate = np.deg2rad(max(0.0, float(args.body_rate_max_roll_rate_deg)))
    max_pitch_rate = np.deg2rad(max(0.0, float(args.body_rate_max_pitch_rate_deg)))
    max_yaw_rate = np.deg2rad(max(0.0, float(args.max_yaw_rate_deg)))
    p_cmd = float(np.clip(attitude_p * (roll_sp - float(roll_rad)), -max_roll_rate, max_roll_rate))
    q_cmd = float(np.clip(attitude_p * (pitch_sp - float(pitch_rad)), -max_pitch_rate, max_pitch_rate))
    r_cmd = float(np.clip(np.deg2rad(float(yaw_rate_deg_s)), -max_yaw_rate, max_yaw_rate))

    thrust = float(args.body_rate_hover_thrust) + float(args.body_rate_thrust_gain) * (-float(accel_I[2]) / GRAVITY_MPS2)
    thrust = float(
        np.clip(
            thrust,
            max(0.0, float(args.body_rate_min_thrust)),
            min(1.0, float(args.body_rate_max_thrust)),
        )
    )
    return {
        "accel_B": accel_B,
        "roll_sp_rad": roll_sp,
        "pitch_sp_rad": pitch_sp,
        "body_rates_rad_s": np.array([p_cmd, q_cmd, r_cmd], dtype=float),
        "thrust": thrust,
    }


class PX4MavlinkOffboard:
    def __init__(self, url: str, *, source_system: int = 134, source_component: int = 1):
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise SystemExit("Install pymavlink before using --px4-command-mode mavlink_offboard.") from exc
        self.mavutil = mavutil
        self.master = mavutil.mavlink_connection(url, source_system=source_system, source_component=source_component)
        self.target_system = 1
        self.target_component = 1
        self.connected = False
        self.armed = False
        self.offboard_requested = False
        self.last_mode_request_s = 0.0
        self.start_monotonic = time.monotonic()

    def connect(self, timeout_s: float = 12.0) -> bool:
        if self.connected:
            return True
        heartbeat = self.master.wait_heartbeat(timeout=timeout_s)
        if heartbeat is None:
            print(f"px4_mavlink_warning=no_heartbeat url={self.master.address}")
            return False
        self.target_system = int(self.master.target_system or 1)
        self.target_component = int(self.master.target_component or 1)
        self.connected = True
        print(
            "px4_mavlink_connected "
            f"target_system={self.target_system} target_component={self.target_component} heartbeat={heartbeat}"
        )
        return True

    def _time_boot_ms(self) -> int:
        return int(max(0.0, time.monotonic() - self.start_monotonic) * 1000.0) & 0xFFFFFFFF

    def send_velocity(self, velocity_ned: np.ndarray, yaw_rate_deg_s: float = 0.0) -> None:
        if not self.connected and not self.connect(timeout_s=0.2):
            return
        velocity = np.asarray(velocity_ned, dtype=float)
        yaw_rate_rad_s = float(np.deg2rad(yaw_rate_deg_s))
        mavlink = self.mavutil.mavlink
        # Ignore position and acceleration; use vx/vy/vz plus yaw-rate. Also ignore yaw.
        type_mask = 0x0007 | 0x01C0 | 0x0400
        self.master.mav.set_position_target_local_ned_send(
            self._time_boot_ms(),
            self.target_system,
            self.target_component,
            mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0.0,
            0.0,
            0.0,
            float(velocity[0]),
            float(velocity[1]),
            float(velocity[2]),
            0.0,
            0.0,
            0.0,
            0.0,
            yaw_rate_rad_s,
        )

    def send_body_rate(self, body_rates_rad_s: np.ndarray, thrust: float) -> None:
        if not self.connected and not self.connect(timeout_s=0.2):
            return
        rates = np.asarray(body_rates_rad_s, dtype=float)
        mavlink = self.mavutil.mavlink
        type_mask = int(getattr(mavlink, "ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE", 128))
        self.master.mav.set_attitude_target_send(
            self._time_boot_ms(),
            self.target_system,
            self.target_component,
            type_mask,
            [1.0, 0.0, 0.0, 0.0],
            float(rates[0]),
            float(rates[1]),
            float(rates[2]),
            float(np.clip(float(thrust), 0.0, 1.0)),
        )

    def arm(self) -> None:
        if not self.connected and not self.connect(timeout_s=1.0):
            return
        if self.armed:
            return
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.armed = True

    def request_offboard(self) -> None:
        if not self.connected and not self.connect(timeout_s=1.0):
            return
        now = time.monotonic()
        if self.offboard_requested and now - self.last_mode_request_s < 2.0:
            return
        self.master.set_mode("OFFBOARD")
        self.offboard_requested = True
        self.last_mode_request_s = now

    def prime_and_request_offboard(self, duration_s: float) -> None:
        if not self.connect(timeout_s=max(2.0, duration_s + 4.0)):
            return
        deadline = time.monotonic() + max(0.0, float(duration_s))
        while time.monotonic() < deadline:
            self.send_velocity(np.zeros(3, dtype=float), 0.0)
            time.sleep(0.05)
        self.arm()
        for _ in range(10):
            self.send_velocity(np.zeros(3, dtype=float), 0.0)
            time.sleep(0.05)
        self.request_offboard()

    def prime_body_rate_and_request_offboard(self, duration_s: float, hover_thrust: float) -> None:
        if not self.connect(timeout_s=max(2.0, duration_s + 4.0)):
            return
        deadline = time.monotonic() + max(0.0, float(duration_s))
        while time.monotonic() < deadline:
            self.send_body_rate(np.zeros(3, dtype=float), hover_thrust)
            time.sleep(0.05)
        self.arm()
        for _ in range(10):
            self.send_body_rate(np.zeros(3, dtype=float), hover_thrust)
            time.sleep(0.05)
        self.request_offboard()

    def stop(self) -> None:
        if not self.connected:
            return
        for _ in range(10):
            self.send_velocity(np.zeros(3, dtype=float), 0.0)
            time.sleep(0.03)

    def stop_body_rate(self, hover_thrust: float) -> None:
        if not self.connected:
            return
        for _ in range(10):
            self.send_body_rate(np.zeros(3, dtype=float), hover_thrust)
            time.sleep(0.03)


def _px4_mavlink_offboard(args) -> Optional[PX4MavlinkOffboard]:
    if getattr(args, "px4_command_mode", "") not in {"mavlink_offboard", "mavlink_body_rate"}:
        return None
    offboard = getattr(args, "_px4_mavlink_offboard", None)
    if offboard is None:
        offboard = PX4MavlinkOffboard(args.px4_mavlink_url)
        setattr(args, "_px4_mavlink_offboard", offboard)
    return offboard


def _ensure_px4_api_control(client, args, vehicle_name: str) -> bool:
    if not _is_px4_vehicle(vehicle_name, args):
        return True
    if args.px4_command_mode in {"mavlink_offboard", "mavlink_body_rate"}:
        offboard = _px4_mavlink_offboard(args)
        if offboard is not None:
            offboard.connect(timeout_s=0.2)
        return True
    try:
        if not client.isApiControlEnabled(vehicle_name=vehicle_name):
            client.enableApiControl(True, vehicle_name=vehicle_name)
        armed_key = f"_px4_armed_once_{vehicle_name}"
        if not getattr(args, armed_key, False):
            client.armDisarm(True, vehicle_name=vehicle_name)
            setattr(args, armed_key, True)
        return True
    except Exception as exc:
        print(f"px4_api_control_warning vehicle={vehicle_name}: {exc}")
        return False


def _command_vehicle_velocity(
    client,
    airsim_module,
    vehicle_name: str,
    velocity: np.ndarray,
    yaw_rate_deg_s: float,
    command_duration: float,
    args,
):
    command = _px4_limited_velocity(velocity, args) if _is_px4_vehicle(vehicle_name, args) else np.asarray(velocity, dtype=float)
    if _is_px4_vehicle(vehicle_name, args):
        if args.px4_command_mode == "mavlink_offboard":
            offboard = _px4_mavlink_offboard(args)
            if offboard is not None:
                offboard.send_velocity(command, yaw_rate_deg_s)
                offboard.arm()
                offboard.request_offboard()
            return None
        if args.px4_command_mode == "mavlink_body_rate":
            _ensure_px4_api_control(client, args, vehicle_name)
            return client.moveByVelocityAsync(
                float(command[0]),
                float(command[1]),
                float(command[2]),
                duration=command_duration,
                drivetrain=airsim_module.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim_module.YawMode(is_rate=True, yaw_or_rate=yaw_rate_deg_s),
                vehicle_name=vehicle_name,
            )
        _ensure_px4_api_control(client, args, vehicle_name)
        if args.px4_command_mode == "velocity_simple":
            return client.moveByVelocityAsync(
                float(command[0]),
                float(command[1]),
                float(command[2]),
                duration=command_duration,
                vehicle_name=vehicle_name,
            )
    return client.moveByVelocityAsync(
        float(command[0]),
        float(command[1]),
        float(command[2]),
        duration=command_duration,
        drivetrain=airsim_module.DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=airsim_module.YawMode(is_rate=True, yaw_or_rate=yaw_rate_deg_s),
        vehicle_name=vehicle_name,
    )


def _px4_keepalive(client, airsim_module, vehicles: Sequence[str], args, duration_s: float = 1.0) -> None:
    if getattr(args, "hil_bench_safe", False):
        return
    px4_vehicles = [vehicle for vehicle in vehicles if _is_px4_vehicle(vehicle, args)]
    if not px4_vehicles:
        return
    command_dt = 0.20
    deadline = time.monotonic() + max(command_dt, float(duration_s))
    while time.monotonic() < deadline:
        for vehicle in px4_vehicles:
            _command_vehicle_velocity(client, airsim_module, vehicle, np.zeros(3, dtype=float), 0.0, command_dt, args)
        time.sleep(command_dt)


def _register_px4_shutdown_stop(client, args) -> None:
    if getattr(args, "hil_bench_safe", False):
        return
    if not (getattr(args, "px4_interceptor", False) or getattr(args, "px4_intruder", False)):
        return

    vehicles = [args.interceptor]
    if getattr(args, "px4_intruder", False) and not getattr(args, "intruder_actor", False):
        vehicles.append(args.intruder)

    def stop_px4_vehicles() -> None:
        offboard = _px4_mavlink_offboard(args)
        if offboard is not None:
            if getattr(args, "px4_command_mode", "") == "mavlink_body_rate":
                offboard.stop_body_rate(_body_rate_neutral_thrust(args))
            else:
                offboard.stop()
        for _ in range(10):
            for vehicle in vehicles:
                try:
                    client.moveByVelocityAsync(0.0, 0.0, 0.0, duration=0.2, vehicle_name=vehicle)
                except Exception:
                    pass
            time.sleep(0.05)

    atexit.register(stop_px4_vehicles)


def _move_px4_vehicle_to_local(
    client,
    airsim_module,
    vehicle_name: str,
    local_position: np.ndarray,
    args,
    *,
    label: str,
) -> None:
    target = np.asarray(local_position, dtype=float)
    speed = max(0.5, float(args.climb_speed))
    deadline = time.monotonic() + max(5.0, float(args.climb_timeout_s))
    command_dt = 0.50
    reached = False
    last_print = 0.0
    while time.monotonic() < deadline:
        _ensure_px4_api_control(client, args, vehicle_name)
        kin = _guidance_kinematics(client, vehicle_name, args)
        position = _vector_xyz(kin.position)
        velocity = _vector_xyz(kin.linear_velocity)
        error = target - position
        distance = float(np.linalg.norm(error))
        if distance <= 1.0 and float(np.linalg.norm(velocity)) <= max(0.2, float(args.settle_speed)):
            reached = True
            break
        direction = error / max(1.0e-6, distance)
        command = direction * min(speed, 1.5 * distance)
        command = _px4_limited_velocity(command, args)
        client.moveByVelocityAsync(
            float(command[0]),
            float(command[1]),
            float(command[2]),
            duration=command_dt,
            drivetrain=airsim_module.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim_module.YawMode(is_rate=True, yaw_or_rate=0.0),
            vehicle_name=vehicle_name,
        )
        for keepalive_vehicle in (args.interceptor, args.intruder):
            if keepalive_vehicle != vehicle_name and _is_px4_vehicle(keepalive_vehicle, args):
                client.moveByVelocityAsync(0.0, 0.0, 0.0, duration=command_dt, vehicle_name=keepalive_vehicle)
        now = time.monotonic()
        if now - last_print >= 2.0:
            print(
                f"px4_{label}_status vehicle={vehicle_name} pos={np.array2string(position, precision=2)} "
                f"target={np.array2string(target, precision=2)} dist={distance:.2f}"
            )
            last_print = now
        time.sleep(command_dt)
    if not reached:
        kin = _guidance_kinematics(client, vehicle_name, args)
        position = _vector_xyz(kin.position)
        raise SystemExit(
            f"PX4 vehicle failed to reach {label}: vehicle={vehicle_name}, "
            f"pos={np.array2string(position, precision=2)}, target={np.array2string(target, precision=2)}."
        )


def _prepare_px4_mixed_intercept_altitude(
    client,
    airsim_module,
    vehicles,
    target_z: dict[str, float],
    args,
    origins: dict[str, np.ndarray],
) -> None:
    px4_vehicles = [vehicle for vehicle in vehicles if _is_px4_vehicle(vehicle, args)]
    simple_vehicles = [vehicle for vehicle in vehicles if not _is_px4_vehicle(vehicle, args)]
    print(f"Using PX4-friendly altitude preparation for: {_format_names(px4_vehicles)}")
    for vehicle in simple_vehicles:
        client.takeoffAsync(timeout_sec=args.climb_timeout_s, vehicle_name=vehicle).join()
        client.moveToZAsync(
            target_z[vehicle],
            velocity=args.climb_speed,
            timeout_sec=args.climb_timeout_s,
            vehicle_name=vehicle,
        ).join()
        client.hoverAsync(vehicle_name=vehicle).join()

    for vehicle in px4_vehicles:
        for _ in range(6):
            _ensure_px4_api_control(client, args, vehicle)
            client.moveByVelocityAsync(0.0, 0.0, 0.0, duration=0.2, vehicle_name=vehicle)
            time.sleep(0.1)

    climb_speed = max(0.3, abs(float(args.climb_speed)))
    settle_speed = max(0.1, float(args.settle_speed))
    deadline = time.monotonic() + max(5.0, float(args.climb_timeout_s))
    command_dt = 0.25
    last_print = 0.0
    reached: set[str] = set()
    while time.monotonic() < deadline:
        statuses = []
        for vehicle in px4_vehicles:
            if vehicle in reached:
                continue
            local_target_z = _px4_local_target_z(vehicle, target_z[vehicle], origins)
            kin = _guidance_kinematics(client, vehicle, args)
            position = _vector_xyz(kin.position)
            velocity = _vector_xyz(kin.linear_velocity)
            z_error = local_target_z - float(position[2])
            if abs(z_error) <= 1.0 and abs(float(velocity[2])) <= settle_speed:
                reached.add(vehicle)
                continue
            vz_cmd = float(np.clip(1.2 * z_error, -climb_speed, climb_speed))
            _command_vehicle_velocity(client, airsim_module, vehicle, np.array([0.0, 0.0, vz_cmd], dtype=float), 0.0, command_dt, args)
            statuses.append(
                f"{vehicle}: z={position[2]:.2f} target={local_target_z:.2f} "
                f"err={z_error:.2f} vz={velocity[2]:.2f} cmd={vz_cmd:.2f}"
            )
        if len(reached) == len(px4_vehicles):
            break
        now = time.monotonic()
        if statuses and now - last_print >= 2.0:
            print("px4_climb_status " + " | ".join(statuses))
            last_print = now
        time.sleep(command_dt)
    missing = [vehicle for vehicle in px4_vehicles if vehicle not in reached]
    if missing:
        details = []
        for vehicle in missing:
            kin = _guidance_kinematics(client, vehicle, args)
            position = _vector_xyz(kin.position)
            details.append(f"{vehicle}: z={position[2]:.2f}, target={_px4_local_target_z(vehicle, target_z[vehicle], origins):.2f}")
        raise SystemExit("PX4 vehicle failed to reach intercept altitude: " + "; ".join(details))

    for vehicle in simple_vehicles:
        client.hoverAsync(vehicle_name=vehicle)
    if args.settle_s > 0.0:
        time.sleep(args.settle_s)
    for _ in range(4):
        for vehicle in px4_vehicles:
            client.moveByVelocityAsync(0.0, 0.0, 0.0, duration=0.25, vehicle_name=vehicle)
        time.sleep(0.1)


def _prepare_intercept_altitude(client, airsim_module, vehicles, args, origins: dict[str, np.ndarray]) -> None:
    if not args.climb_to_altitude:
        return
    target_z = {
        args.interceptor: -abs(float(args.intercept_altitude_m)),
        args.intruder: -(abs(float(args.intercept_altitude_m)) + float(args.intruder_altitude_offset_m)),
    }
    if args.intruder_actor:
        target_z.pop(args.intruder, None)
    target_text = ", ".join(f"{vehicle}: NED_Z={target_z[vehicle]:.1f}" for vehicle in vehicles)
    print(f"Climbing vehicles to intercept start altitudes: {target_text}")
    if args.px4_interceptor or args.px4_intruder:
        _prepare_px4_mixed_intercept_altitude(client, airsim_module, vehicles, target_z, args, origins)
    else:
        for future in [client.takeoffAsync(timeout_sec=args.climb_timeout_s, vehicle_name=v) for v in vehicles]:
            future.join()
        for future in [
            client.moveToZAsync(
                target_z[v],
                velocity=args.climb_speed,
                timeout_sec=args.climb_timeout_s,
                vehicle_name=v,
            )
            for v in vehicles
        ]:
            future.join()
        for vehicle in vehicles:
            client.hoverAsync(vehicle_name=vehicle).join()
        if args.settle_s > 0.0:
            time.sleep(args.settle_s)
        settle_start = time.monotonic()
        while time.monotonic() - settle_start < args.settle_timeout_s:
            speeds = [
                float(np.linalg.norm(_vector_xyz(_guidance_kinematics(client, vehicle, args).linear_velocity)))
                for vehicle in vehicles
            ]
            if speeds and max(speeds) <= args.settle_speed:
                break
            for future in [client.hoverAsync(vehicle_name=vehicle) for vehicle in vehicles]:
                future.join()
            time.sleep(0.2)
    print("Altitude preparation complete; starting strapdown vision loop.")


def _apply_start_geometry(client, airsim_module, args, origins: dict[str, np.ndarray]) -> None:
    if not _start_geometry_enabled(args):
        return

    forward, lateral, horizontal_range = _start_geometry_offsets(args)
    if args.hil_bench_safe or args.px4_hil:
        interceptor_pose = client.simGetObjectPose(args.interceptor)
        interceptor_world = _vector_xyz(interceptor_pose.position)
        interceptor_local = interceptor_world - origins.get(args.interceptor, np.zeros(3, dtype=float))
        R_IU = airsim_orientation_to_R_IB(interceptor_pose.orientation)
        forward_axis = np.array([R_IU[0, 0], R_IU[1, 0], 0.0], dtype=float)
        right_axis = np.array([R_IU[0, 1], R_IU[1, 1], 0.0], dtype=float)
        forward_norm = float(np.linalg.norm(forward_axis))
        right_norm = float(np.linalg.norm(right_axis))
        forward_axis = forward_axis / forward_norm if forward_norm > 1.0e-6 else np.array([1.0, 0.0, 0.0])
        right_axis = right_axis / right_norm if right_norm > 1.0e-6 else np.array([0.0, 1.0, 0.0])
        intruder_world = interceptor_world + forward * forward_axis + lateral * right_axis
        intruder_world[2] = interceptor_world[2] - float(args.intruder_altitude_offset_m)
    elif args.px4_interceptor:
        interceptor_kin = _guidance_kinematics(client, args.interceptor, args)
        interceptor_local = _vector_xyz(interceptor_kin.position)
        interceptor_world = _vehicle_truth_position(client, args.interceptor, origins, kinematics=interceptor_kin)
        state = client.getMultirotorState(vehicle_name=args.interceptor)
        R_IB = airsim_orientation_to_R_IB(state.kinematics_estimated.orientation)
        forward_axis = np.array([R_IB[0, 0], R_IB[1, 0], 0.0], dtype=float)
        right_axis = np.array([R_IB[0, 1], R_IB[1, 1], 0.0], dtype=float)
        forward_norm = float(np.linalg.norm(forward_axis))
        right_norm = float(np.linalg.norm(right_axis))
        forward_axis = forward_axis / forward_norm if forward_norm > 1.0e-6 else np.array([1.0, 0.0, 0.0])
        right_axis = right_axis / right_norm if right_norm > 1.0e-6 else np.array([0.0, 1.0, 0.0])
        intruder_world = interceptor_world + forward * forward_axis + lateral * right_axis
        intruder_world[2] = interceptor_world[2] - float(args.intruder_altitude_offset_m)
    else:
        interceptor_local = _interceptor_start_local(args, origins)
        interceptor_world = origins.get(args.interceptor, np.zeros(3, dtype=float)) + interceptor_local
        intruder_world = interceptor_world + np.array([forward, lateral, -float(args.intruder_altitude_offset_m)], dtype=float)
    intruder_local = intruder_world - origins.get(args.intruder, np.zeros(3, dtype=float))

    if args.intruder_actor:
        _spawn_or_move_intruder_actor(client, airsim_module, args, intruder_world, _yaw_deg_from_velocity(_intruder_velocity(args)))
    else:
        for vehicle, local_position in [(args.intruder, intruder_local)]:
            if _is_px4_vehicle(vehicle, args):
                _move_px4_vehicle_to_local(client, airsim_module, vehicle, local_position, args, label="start")
            else:
                client.moveToPositionAsync(
                    float(local_position[0]),
                    float(local_position[1]),
                    float(local_position[2]),
                    velocity=max(0.5, float(args.climb_speed)),
                    timeout_sec=max(1.0, float(args.climb_timeout_s)),
                    vehicle_name=vehicle,
                ).join()
                client.hoverAsync(vehicle_name=vehicle).join()
    if not args.px4_interceptor:
        client.moveToPositionAsync(
            float(interceptor_local[0]),
            float(interceptor_local[1]),
            float(interceptor_local[2]),
            velocity=max(0.5, float(args.climb_speed)),
            timeout_sec=max(1.0, float(args.climb_timeout_s)),
            vehicle_name=args.interceptor,
        ).join()
        client.hoverAsync(vehicle_name=args.interceptor).join()
    intruder_yaw_deg = _yaw_deg_from_velocity(_intruder_velocity(args))
    if args.intruder_actor:
        pass
    elif _is_px4_vehicle(args.intruder, args):
        _command_vehicle_velocity(client, airsim_module, args.intruder, np.zeros(3), 0.0, 0.5, args)
    else:
        client.rotateToYawAsync(
            intruder_yaw_deg,
            timeout_sec=args.initial_align_timeout_s,
            margin=args.initial_align_margin_deg,
            vehicle_name=args.intruder,
        ).join()

    if args.start_geometry_settle_s > 0.0:
        time.sleep(float(args.start_geometry_settle_s))
    print(
        "Applied strapdown start geometry: "
        f"horizontal_range={horizontal_range:.2f}m, forward={forward:.2f}m, lateral={lateral:.2f}m, "
        f"altitude_offset={float(args.intruder_altitude_offset_m):.2f}m, "
        f"interceptor_local={np.array2string(interceptor_local, precision=2)}, "
        f"intruder_local={np.array2string(intruder_local, precision=2)}"
    )


def _command_interceptor_velocity(client, airsim_module, v_cmd: np.ndarray, yaw_rate_deg_s: float, command_duration: float, args):
    future = _command_vehicle_velocity(client, airsim_module, args.interceptor, v_cmd, yaw_rate_deg_s, command_duration, args)
    if args.px4_interceptor and args.px4_command_join:
        future.join()
    return future


def _command_interceptor_body_rate(body_rates_rad_s: np.ndarray, thrust: float, args) -> None:
    offboard = _px4_mavlink_offboard(args)
    if offboard is not None:
        offboard.send_body_rate(body_rates_rad_s, thrust)
        offboard.arm()
        offboard.request_offboard()


def _yaw_rate_from_pixel_error(pixel_error_x: float, intrinsics, args) -> float:
    if not args.yaw_control:
        return 0.0
    yaw_error_rad = float(np.arctan2(pixel_error_x, intrinsics.fx))
    yaw_rate = float(np.rad2deg(args.yaw_error_gain * yaw_error_rad))
    return float(np.clip(yaw_rate, -args.max_yaw_rate_deg, args.max_yaw_rate_deg))


def _yaw_rate_from_angle_error(theta_x_rad: float, args) -> float:
    if not args.yaw_control:
        return 0.0
    yaw_rate = float(np.rad2deg(args.yaw_error_gain * float(theta_x_rad)))
    return float(np.clip(yaw_rate, -args.max_yaw_rate_deg, args.max_yaw_rate_deg))


def _append_yaw_rate_sample(
    samples: list[tuple[float, float]],
    timestamp: float,
    yaw_rate_deg_s: float,
    valid: bool,
    args,
) -> None:
    if valid and np.isfinite(yaw_rate_deg_s):
        samples.append((float(timestamp), float(yaw_rate_deg_s)))
    keep_s = max(1.0, 4.0 * max(0.01, float(args.terminal_yaw_rate_average_window_s)))
    samples[:] = [(ts, value) for ts, value in samples if timestamp - ts <= keep_s]


def _terminal_yaw_rate_command(
    *,
    current_yaw_rate_deg_s: float,
    samples: list[tuple[float, float]],
    timestamp: float,
    terminal_state: str,
    using_blind_push: bool,
    blind_elapsed_s: float,
    args,
) -> tuple[float, float, int, float]:
    if not args.terminal_yaw_rate_extrapolation:
        return current_yaw_rate_deg_s, 0.0, 0, 0.0

    max_rate = max(0.0, float(args.max_yaw_rate_deg))
    if not using_blind_push:
        if terminal_state == TERMINAL_VISUAL:
            scaled = float(current_yaw_rate_deg_s) * max(0.0, float(args.terminal_yaw_rate_scale))
            return float(np.clip(scaled, -max_rate, max_rate)), 0.0, 0, 1.0
        return current_yaw_rate_deg_s, 0.0, 0, 0.0

    window = max(0.01, float(args.terminal_yaw_rate_average_window_s))
    recent = [value for ts, value in samples if timestamp - ts <= window]
    if not recent and samples:
        recent = [samples[-1][1]]
    if not recent:
        return current_yaw_rate_deg_s, 0.0, 0, 0.0
    base = float(np.mean(recent))
    tau = max(1.0e-6, float(args.terminal_yaw_rate_decay_tau_s))
    decay = float(np.exp(-max(0.0, float(blind_elapsed_s)) / tau))
    command = float(np.clip(decay * base, -max_rate, max_rate))
    return command, base, len(recent), decay


def _kf_yaw_rate_command(image_kf, args) -> float:
    feedback = _yaw_rate_from_angle_error(image_kf.theta_x, args)
    feedforward = float(np.rad2deg(image_kf.theta_dot_x))
    command = feedback + feedforward
    return float(np.clip(command, -args.max_yaw_rate_deg, args.max_yaw_rate_deg))


def _image_kf_los_guidance_estimate(image_kf, intrinsics, R_BC: np.ndarray, R_IB: np.ndarray):
    theta = np.array([image_kf.theta_x, image_kf.theta_y], dtype=float)
    theta_dot = np.array([image_kf.theta_dot_x, image_kf.theta_dot_y], dtype=float)
    if not np.all(np.isfinite(theta)) or not np.all(np.isfinite(theta_dot)):
        return None, None

    center_now = center_from_angle_error(theta, intrinsics)
    los_C_now = camera_ray_from_pixel(*center_now, intrinsics)
    lambda_I = los_camera_to_inertial(los_C_now, R_BC, R_IB)

    dt = 0.02
    center_next = center_from_angle_error(theta + theta_dot * dt, intrinsics)
    los_C_next = camera_ray_from_pixel(*center_next, intrinsics)
    lambda_next_I = los_camera_to_inertial(los_C_next, R_BC, R_IB)
    lambda_dot_I = project_perpendicular((lambda_next_I - lambda_I) / dt, lambda_I)
    omega_los = np.cross(lambda_I, lambda_dot_I)
    if not np.all(np.isfinite(lambda_I)) or not np.all(np.isfinite(omega_los)):
        return None, None
    return lambda_I, omega_los


def _clip_vector_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    limit = max(0.0, float(max_norm))
    norm = float(np.linalg.norm(value))
    if norm <= limit or norm <= 1.0e-9:
        return np.array(value, dtype=float)
    return value * (limit / norm)


def _png_acceleration_command(
    lambda_I: Optional[np.ndarray],
    omega_los: Optional[np.ndarray],
    guidance_gain: float,
    args,
) -> np.ndarray:
    if lambda_I is None or omega_los is None:
        return np.zeros(3, dtype=float)
    accel = float(guidance_gain) * np.cross(np.asarray(omega_los, dtype=float), np.asarray(lambda_I, dtype=float))
    if not np.all(np.isfinite(accel)):
        return np.zeros(3, dtype=float)
    return _clip_vector_norm(accel, float(args.max_guidance_accel_mps2))


def _fallback_direction(lambda_I: Optional[np.ndarray]) -> Optional[np.ndarray]:
    if lambda_I is None:
        return None
    direction = np.asarray(lambda_I, dtype=float)
    norm = float(np.linalg.norm(direction))
    if norm <= 1.0e-9 or not np.all(np.isfinite(direction)):
        return None
    return direction / norm


def _accel_integrated_velocity(
    current_velocity: np.ndarray,
    acceleration: np.ndarray,
    dt: float,
    speed_cap: float,
    lambda_I: Optional[np.ndarray],
    args,
) -> np.ndarray:
    min_speed = max(0.0, min(float(speed_cap), float(args.min_speed_ratio) * float(speed_cap)))
    return integrate_velocity_command(
        current_velocity,
        acceleration,
        max(1.0e-6, float(dt)),
        speed_cap=max(0.0, float(speed_cap)),
        min_speed=min_speed,
        fallback_direction=_fallback_direction(lambda_I),
    )


def _los_base_velocity(lambda_I: Optional[np.ndarray], speed_cap: float) -> np.ndarray:
    direction = _fallback_direction(lambda_I)
    if direction is None:
        direction = np.array([1.0, 0.0, 0.0], dtype=float)
    return float(max(0.0, speed_cap)) * direction


def _body_rate_control_acceleration(
    *,
    png_acceleration_I: np.ndarray,
    current_velocity_I: np.ndarray,
    velocity_reference_I: np.ndarray,
    args,
) -> tuple[np.ndarray, np.ndarray]:
    speed_hold = float(args.body_rate_speed_hold_gain) * (
        np.asarray(velocity_reference_I, dtype=float) - np.asarray(current_velocity_I, dtype=float)
    )
    speed_hold = _clip_vector_norm(speed_hold, float(args.body_rate_speed_hold_max_accel_mps2))
    total = np.asarray(png_acceleration_I, dtype=float) + speed_hold
    total = _clip_vector_norm(total, float(args.body_rate_total_accel_limit_mps2))
    return total, speed_hold


def _candidate_guidance_velocity(
    current_velocity: np.ndarray,
    lambda_I: Optional[np.ndarray],
    omega_los: Optional[np.ndarray],
    guidance_gain: float,
    scheduled_speed_cap: float,
    dt: float,
    valid: bool,
    args,
) -> tuple[np.ndarray, np.ndarray, float]:
    mode = str(getattr(args, "guidance_output_mode", "velocity_bias"))
    if mode not in {"accel_integral", "accel_body_rate"}:
        command = _guidance_velocity(
            current_velocity,
            lambda_I,
            omega_los if valid else None,
            guidance_gain,
            scheduled_speed_cap,
            args,
        )
        accel = _png_acceleration_command(lambda_I, omega_los if valid else None, guidance_gain, args)
        return command, accel, 0.0

    accel = _png_acceleration_command(lambda_I, omega_los if valid else None, guidance_gain, args)
    if not valid and bool(getattr(args, "accel_integral_reset_on_invalid", False)):
        accel = np.zeros(3, dtype=float)
    if mode == "accel_body_rate":
        command = _los_base_velocity(lambda_I, scheduled_speed_cap)
        return command, accel, 0.0
    command = _accel_integrated_velocity(
        current_velocity,
        accel,
        dt,
        scheduled_speed_cap,
        lambda_I,
        args,
    )
    return command, accel, max(1.0e-6, float(dt))


def _image_error_ratio(px_err_x: float, px_err_y: float, intrinsics) -> float:
    half_w = max(1.0, 0.5 * float(intrinsics.width))
    half_h = max(1.0, 0.5 * float(intrinsics.height))
    return float(max(abs(float(px_err_x)) / half_w, abs(float(px_err_y)) / half_h))


def _bbox_area_ratio(bbox_area: float, intrinsics) -> float:
    return float(max(0.0, float(bbox_area)) / max(1.0, float(intrinsics.width * intrinsics.height)))


def _scheduled_speed_cap(
    *,
    base_speed_cap: float,
    intruder_speed: float,
    error_ratio: float,
    area_ratio: float,
    ttc_s: Optional[float],
    args,
) -> tuple[float, str]:
    if not bool(args.frame_guard):
        return float(base_speed_cap), "base"
    min_speed = max(0.1, float(args.frame_guard_min_speed_ratio) * float(intruder_speed))
    mid_speed = max(min_speed, float(args.frame_guard_mid_speed_ratio) * float(intruder_speed))
    base_speed = max(mid_speed, float(base_speed_cap))
    area_mid = max(0.0, float(args.frame_guard_area_mid_ratio))
    area_terminal = max(area_mid, float(args.frame_guard_area_ratio))
    ttc_terminal = max(0.0, float(args.frame_guard_ttc_terminal_s))
    ttc_mid = max(ttc_terminal, float(args.frame_guard_ttc_mid_s))
    ttc_finite = ttc_s is not None and np.isfinite(float(ttc_s)) and float(ttc_s) > 0.0
    terminal_by_ttc = bool(ttc_finite and float(ttc_s) <= ttc_terminal)
    mid_by_ttc = bool(ttc_finite and float(ttc_s) <= ttc_mid)
    if area_ratio >= area_terminal or error_ratio >= max(0.0, float(args.frame_guard_enter_error_ratio)) or terminal_by_ttc:
        return min(base_speed, min_speed), "terminal_capture"
    if area_ratio >= area_mid or error_ratio >= max(0.0, float(args.frame_guard_exit_error_ratio)) or mid_by_ttc:
        return min(base_speed, mid_speed), "frame_guard"
    return float(base_speed_cap), "base"


def _apply_frame_guard_velocity(
    v_cmd: np.ndarray,
    *,
    lambda_I: Optional[np.ndarray],
    scheduled_speed_cap: float,
    error_ratio: float,
    area_ratio: float,
    speed_schedule_mode: str,
    args,
) -> tuple[np.ndarray, int, str]:
    command = np.array(v_cmd, dtype=float)
    if not bool(args.frame_guard):
        return command, 0, "disabled"

    enter_error = max(0.0, float(args.frame_guard_enter_error_ratio))
    exit_error = max(0.0, float(args.frame_guard_exit_error_ratio))
    area_mid = max(0.0, float(args.frame_guard_area_mid_ratio))
    area_terminal = max(area_mid, float(args.frame_guard_area_ratio))
    active = (
        error_ratio >= exit_error
        or area_ratio >= area_mid
        or str(speed_schedule_mode) in {"frame_guard", "terminal_capture"}
    )
    if not active:
        norm = float(np.linalg.norm(command))
        if norm > scheduled_speed_cap > 1.0e-6:
            command *= scheduled_speed_cap / norm
        return command, 0, "inactive"

    if lambda_I is None or float(np.linalg.norm(lambda_I)) <= 1.0e-6:
        forward = normalize(np.array([1.0, 0.0, 0.0], dtype=float))
    else:
        forward = normalize(np.asarray(lambda_I, dtype=float))
    forward_component = float(np.dot(command, forward))
    lateral = command - forward_component * forward
    lateral[2] = 0.0
    vertical = float(command[2])

    forward_component = float(np.clip(forward_component, 0.0, scheduled_speed_cap))
    lateral *= max(0.0, float(args.frame_guard_lateral_scale))
    vertical *= max(0.0, float(args.frame_guard_vertical_scale))
    command = forward_component * forward + lateral
    command[2] = float(np.clip(vertical, -args.max_vision_vertical_speed, args.max_vision_vertical_speed))
    norm = float(np.linalg.norm(command))
    if norm > scheduled_speed_cap > 1.0e-6:
        command *= scheduled_speed_cap / norm
    mode = (
        "terminal_capture"
        if error_ratio >= enter_error or area_ratio >= area_terminal or str(speed_schedule_mode) == "terminal_capture"
        else "frame_guard"
    )
    return command, 1, mode


def _frame_guard_yaw_rate(yaw_rate_deg_s: float, px_err_x: float, intrinsics, args) -> tuple[float, int]:
    if not bool(args.frame_guard):
        return yaw_rate_deg_s, 0
    error_ratio = abs(float(px_err_x)) / max(1.0, 0.5 * float(intrinsics.width))
    if error_ratio < max(0.0, float(args.frame_guard_exit_error_ratio)):
        return yaw_rate_deg_s, 0
    scaled = float(yaw_rate_deg_s) * max(0.0, float(args.frame_guard_yaw_gain_scale))
    limit = max(0.0, min(float(args.max_yaw_rate_deg), float(args.frame_guard_max_yaw_rate_deg)))
    return float(np.clip(scaled, -limit, limit)), 1


def _frame_guard_vertical_bias(px_err_y: float, intrinsics, args) -> float:
    if not bool(args.frame_guard):
        return 0.0
    error_ratio = abs(float(px_err_y)) / max(1.0, 0.5 * float(intrinsics.height))
    if error_ratio < max(0.0, float(args.frame_guard_exit_error_ratio)):
        return 0.0
    # AirSim NED: negative Z is up. Pixel y below center is positive, so command down;
    # pixel y above center is negative, so command up.
    return float(np.clip(float(px_err_y) * float(args.frame_guard_vertical_error_gain), -args.max_vision_vertical_speed, args.max_vision_vertical_speed))


def _yaw_rate_window_average(samples: list[tuple[float, float]], timestamp: float, window_s: float) -> tuple[float, int]:
    window = max(0.01, float(window_s))
    recent = [rate for ts, rate in samples if timestamp - ts <= window and np.isfinite(rate)]
    if not recent:
        return 0.0, 0
    return float(np.mean(recent)), len(recent)


def _frame_centering_state(
    *,
    detected: bool,
    detection_clipped: bool,
    image_error_ratio: float,
    area_ratio: float,
    ttc_s: Optional[float],
    image_kf_valid: bool,
    guard_hold_active: bool,
    guard_hold_error_ratio: float,
    guard_hold_area_ratio: float,
    last_detected_ts: Optional[float],
    timestamp: float,
    args,
) -> tuple[str, int, str]:
    if not bool(args.frame_centering):
        return "disabled", 0, ""

    enter_error = max(0.0, float(args.frame_centering_enter_error_ratio))
    terminal_error = max(enter_error, float(args.frame_centering_terminal_error_ratio))
    terminal_area = max(0.0, float(args.frame_centering_area_ratio))
    loss_hold_s = max(0.0, float(args.frame_centering_loss_hold_s))
    ttc_terminal = max(0.0, float(args.frame_guard_ttc_terminal_s))
    ttc_close = ttc_s is not None and np.isfinite(float(ttc_s)) and float(ttc_s) > 0.0 and float(ttc_s) <= ttc_terminal

    effective_error = max(float(image_error_ratio), float(guard_hold_error_ratio if guard_hold_active else 0.0))
    effective_area = max(float(area_ratio), float(guard_hold_area_ratio if guard_hold_active else 0.0))

    if detected:
        if detection_clipped:
            return "terminal_capture", 1, "bbox_clipped"
        if effective_error >= terminal_error:
            return "terminal_capture", 1, "edge_error_terminal"
        if effective_area >= terminal_area:
            return "terminal_capture", 1, "area_terminal"
        if ttc_close:
            return "terminal_capture", 1, "ttc_terminal"
        if effective_error >= enter_error:
            return "frame_centering", 1, "edge_error"
        if guard_hold_active and effective_error >= max(0.0, float(args.frame_guard_exit_error_ratio)):
            return "frame_centering", 1, "guard_hold"
        return "tracking", 0, ""

    if last_detected_ts is not None and timestamp - last_detected_ts <= loss_hold_s:
        if image_kf_valid:
            return "loss_hold", 1, "kf_loss_hold"
        return "loss_hold", 1, "recent_loss_hold"
    return "tracking", 0, ""


def _apply_frame_centering_velocity(
    v_cmd: np.ndarray,
    *,
    lambda_I: Optional[np.ndarray],
    scheduled_speed_cap: float,
    intruder_speed: float,
    state: str,
    args,
) -> tuple[np.ndarray, float, float, float]:
    command = np.array(v_cmd, dtype=float)
    if not bool(args.frame_centering) or state not in {"frame_centering", "terminal_capture", "loss_hold"}:
        return command, float(np.linalg.norm(command)), 1.0, 0.0

    if lambda_I is None or float(np.linalg.norm(lambda_I)) <= 1.0e-6:
        forward = normalize(np.array([1.0, 0.0, 0.0], dtype=float))
    else:
        forward = normalize(np.asarray(lambda_I, dtype=float))

    if state == "terminal_capture":
        speed_cap = min(
            float(scheduled_speed_cap),
            max(0.1, float(args.terminal_capture_speed_ratio) * float(intruder_speed)),
        )
        lateral_scale = max(0.0, float(args.terminal_capture_lateral_scale))
        lateral_limit = max(0.0, float(args.terminal_capture_max_lateral_speed))
    elif state == "loss_hold":
        speed_cap = min(
            float(scheduled_speed_cap),
            max(0.1, float(args.frame_centering_speed_ratio) * float(intruder_speed)),
        )
        lateral_scale = max(0.0, min(float(args.frame_centering_lateral_scale), float(args.terminal_capture_lateral_scale)))
        lateral_limit = max(0.0, min(float(args.frame_centering_max_lateral_speed), float(args.terminal_capture_max_lateral_speed)))
    else:
        speed_cap = min(
            float(scheduled_speed_cap),
            max(0.1, float(args.frame_centering_speed_ratio) * float(intruder_speed)),
        )
        lateral_scale = max(0.0, float(args.frame_centering_lateral_scale))
        lateral_limit = max(0.0, float(args.frame_centering_max_lateral_speed))

    forward_component = float(np.dot(command, forward))
    min_forward = max(0.0, float(args.frame_centering_min_forward_ratio)) * speed_cap
    forward_component = float(np.clip(forward_component, min_forward, speed_cap))

    lateral = command - forward_component * forward
    lateral_norm = float(np.linalg.norm(lateral))
    if lateral_norm > 1.0e-6:
        lateral *= lateral_scale
        scaled_lateral_norm = float(np.linalg.norm(lateral))
        if lateral_limit > 0.0 and scaled_lateral_norm > lateral_limit:
            lateral *= lateral_limit / scaled_lateral_norm

    command = forward_component * forward + lateral
    command[2] = float(np.clip(command[2], -args.max_vision_vertical_speed, args.max_vision_vertical_speed))
    norm = float(np.linalg.norm(command))
    if norm > speed_cap > 1.0e-6:
        command *= speed_cap / norm
        norm = float(np.linalg.norm(command))
    return command, norm, lateral_scale, lateral_limit


def _frame_centering_yaw_rate(
    *,
    yaw_rate_deg_s: float,
    px_err_x: float,
    intrinsics,
    state: str,
    timestamp: float,
    yaw_rate_samples: list[tuple[float, float]],
    image_kf,
    yaw_rate_source: str,
    args,
) -> tuple[float, str, float, int]:
    if not bool(args.frame_centering) or state not in {"frame_centering", "terminal_capture", "loss_hold"}:
        return yaw_rate_deg_s, yaw_rate_source, 0.0, 0

    if state == "terminal_capture":
        gain_scale = max(0.0, float(args.terminal_capture_yaw_gain_scale))
        limit = max(0.0, min(float(args.max_yaw_rate_deg), float(args.terminal_capture_max_yaw_rate_deg)))
    else:
        gain_scale = max(0.0, float(args.frame_centering_yaw_gain_scale))
        limit = max(0.0, min(float(args.max_yaw_rate_deg), float(args.frame_centering_max_yaw_rate_deg)))

    base = float(yaw_rate_deg_s)
    sample_count = 0
    if state == "loss_hold":
        if image_kf.valid:
            base = _kf_yaw_rate_command(image_kf, args)
            yaw_rate_source = "frame_centering_kf"
        else:
            base, sample_count = _yaw_rate_window_average(
                yaw_rate_samples,
                timestamp,
                float(args.frame_centering_yaw_hold_window_s),
            )
            yaw_rate_source = "frame_centering_hold"
    else:
        # Recompute from the current horizontal image error so this state is explicitly
        # frame-centering driven even if the upstream PNG yaw source is stale.
        base = _yaw_rate_from_pixel_error(px_err_x, intrinsics, args)
        yaw_rate_source = f"{state}_centering"

    decay = 1.0
    if state == "loss_hold" and sample_count > 0:
        newest_ts = max((ts for ts, _ in yaw_rate_samples), default=timestamp)
        tau = max(1.0e-6, float(args.frame_centering_yaw_hold_decay_tau_s))
        decay = float(np.exp(-max(0.0, timestamp - newest_ts) / tau))
    command = float(np.clip(base * gain_scale * decay, -limit, limit))
    return command, yaw_rate_source, decay, sample_count


def _terminal_trigger_strapdown(reason: str, detected: bool, bbox_area: float, intrinsics, args) -> str:
    if reason.startswith("bbox_") and reason.endswith("_clipped"):
        return reason
    if detected and _terminal_area(bbox_area, intrinsics, args):
        return "bbox_area_large"
    return ""


def _is_short_visual_loss_reason(reason: str) -> bool:
    text = str(reason or "")
    return (
        text == "no_detection"
        or text == "los_innovation_reject"
        or text.endswith("_missing")
        or text.startswith("yolo_")
        or text.startswith("bbox_") and text.endswith("_clipped")
    )


def _row_float(row: dict[str, float | int | str], key: str, default: float = 0.0) -> float:
    try:
        value = row.get(key, default)
        if value == "":
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _kinematics_timestamp_s(kinematics) -> Optional[float]:
    for name in ("time_stamp", "timestamp"):
        value = getattr(kinematics, name, None)
        if value is None:
            continue
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if not np.isfinite(number):
            continue
        return number * 1.0e-9 if number > 1.0e6 else number
    return None


def _linear_acceleration_mps2(kinematics) -> np.ndarray:
    acceleration = getattr(kinematics, "linear_acceleration", None)
    if acceleration is None:
        return np.zeros(3, dtype=float)
    return _vector_xyz(acceleration)


def _rows_until_first_hit(rows: list[dict[str, float | int | str]]) -> list[dict[str, float | int | str]]:
    for index, row in enumerate(rows):
        try:
            hit = int(row.get("hit", 0)) == 1
        except (TypeError, ValueError):
            hit = False
        if hit:
            return rows[: index + 1]
    return rows


def _plot_strapdown(rows: list[dict[str, float | int | str]], plot_path: Path) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        _prefer_user_mpl_toolkits()
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except Exception as exc:
        print(f"matplotlib 3D plot unavailable ({exc}); CSV was still saved.")
        return False
    rows = _rows_until_first_hit(rows)
    if not rows:
        return False

    has_truth = all(str(row.get("intruder_x", "")) != "" for row in rows)
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(15, 8))
    ax3d = fig.add_subplot(221, projection="3d")
    ax_range = fig.add_subplot(222)
    ax_pixel = fig.add_subplot(223)
    ax_yaw = fig.add_subplot(224)

    trajectory_specs = [("interceptor", "tab:blue", "Interceptor")]
    if has_truth:
        trajectory_specs.append(("intruder", "tab:red", "Intruder"))
    for prefix, color, label in trajectory_specs:
        xs = [_row_float(row, f"{prefix}_x") for row in rows]
        ys = [_row_float(row, f"{prefix}_y") for row in rows]
        alts = [-_row_float(row, f"{prefix}_z") for row in rows]
        ax3d.plot(xs, ys, alts, color=color, linewidth=2, label=label)
        ax3d.scatter(xs[0], ys[0], alts[0], color=color, marker="o", s=28)
        ax3d.scatter(xs[-1], ys[-1], alts[-1], color=color, marker="x", s=52)
    ax3d.set_xlabel("NED X / m")
    ax3d.set_ylabel("NED Y / m")
    ax3d.set_zlabel("Altitude / m")
    ax3d.legend()
    ax3d.grid(True)

    t = [_row_float(row, "t") for row in rows]
    if has_truth:
        ax_range.plot(t, [_row_float(row, "range", float("nan")) for row in rows], color="tab:green")
    else:
        ax_range.text(0.5, 0.5, "diagnostic truth disabled", transform=ax_range.transAxes, ha="center", va="center")
    ax_range.set_xlabel("Time / s")
    ax_range.set_ylabel("Range / m")
    ax_range.grid(True)

    ax_pixel.plot(t, [_row_float(row, "pixel_error_x") for row in rows], label="u error")
    ax_pixel.plot(t, [_row_float(row, "pixel_error_y") for row in rows], label="v error")
    ax_pixel.set_xlabel("Time / s")
    ax_pixel.set_ylabel("Pixel error / px")
    ax_pixel.legend()
    ax_pixel.grid(True)

    ax_yaw.plot(t, [_row_float(row, "body_yaw_deg") for row in rows], label="body yaw")
    ax_yaw.plot(t, [_row_float(row, "cmd_yaw_deg") for row in rows], label="cmd yaw", linestyle=":")
    ax_yaw.plot(t, [_row_float(row, "target_body_bearing_deg") for row in rows], label="target body bearing", linestyle="--")
    ax_yaw.plot(t, [_row_float(row, "yaw_rate_cmd_deg_s") for row in rows], label="yaw rate cmd", alpha=0.75)
    ax_yaw.set_xlabel("Time / s")
    ax_yaw.set_ylabel("Yaw / deg, rate / deg/s")
    ax_yaw.legend()
    ax_yaw.grid(True)

    fig.subplots_adjust(left=0.05, right=0.98, bottom=0.08, top=0.94, hspace=0.32, wspace=0.25)
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)
    return True


def _draw_detection_window_strapdown(
    display,
    client,
    config: AirSimDetectionConfig,
    detections,
    selected,
    intrinsics,
    yaw_rate_deg_s: float,
    valid: bool,
    reason: str,
    ttc_text: str,
    args,
    image_bgr=None,
) -> bool:
    if display is None or display.get("failed"):
        return False
    cv2 = display["cv2"]
    airsim_module = display["airsim"]
    try:
        if image_bgr is None:
            raw_image = client.simGetImage(
                config.camera_name,
                getattr(airsim_module.ImageType, config.image_type_name),
                vehicle_name=config.vehicle_name,
            )
            if raw_image is None:
                display["image_failures"] += 1
                if display["image_failures"] <= 3:
                    print("OpenCV display: simGetImage returned no image.")
                return False
            image = _decode_airsim_image(cv2, raw_image)
            if image is None:
                display["image_failures"] += 1
                if display["image_failures"] <= 3:
                    print("OpenCV display: failed to decode AirSim image.")
                return False
        else:
            image = np.array(image_bgr, copy=True)
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        cx = int(round(intrinsics.cx))
        cy = int(round(intrinsics.cy))
        cv2.line(image, (cx - 16, cy), (cx + 16, cy), (255, 255, 255), 1)
        cv2.line(image, (cx, cy - 16), (cx, cy + 16), (255, 255, 255), 1)
        for detection in detections:
            box = detection.box2D
            color = (0, 255, 0) if detection is selected else (255, 0, 0)
            p1 = (int(box.min.x_val), int(box.min.y_val))
            p2 = (int(box.max.x_val), int(box.max.y_val))
            cv2.rectangle(image, p1, p2, color, 2)
            cv2.putText(image, getattr(detection, "name", "target"), (p1[0], max(15, p1[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        lines = [
            f"yaw_rate={yaw_rate_deg_s:.1f} deg/s",
            f"detections={len(detections)} selected={getattr(selected, 'name', '-') if selected is not None else '-'}",
            f"valid={valid} reason={reason or 'ok'} ttc={ttc_text or '-'}",
            "q: quit",
        ]
        for i, line in enumerate(lines):
            cv2.putText(image, line, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (36, 255, 12), 1)
        if args.window_scale > 0.0 and abs(args.window_scale - 1.0) > 1.0e-6:
            image = cv2.resize(image, None, fx=args.window_scale, fy=args.window_scale)
        cv2.imshow("Strapdown Vision PNG", image)
        key = cv2.waitKey(1) & 0xFF
        return key == ord("q")
    except Exception as exc:
        display["failed"] = True
        print(f"OpenCV display failed ({exc}); continuing without preview window.")
        return False


def _collision_time_stamp_s(time_stamp_ns: int) -> str | float:
    if not time_stamp_ns:
        return ""
    return float(time_stamp_ns) * 1.0e-9


def _collision_snapshot(client, args):
    if getattr(args, "skip_collision_check", False):
        return _empty_collision_snapshot()
    if args.intruder_actor:
        return get_vehicle_object_collision(
            client,
            args.interceptor,
            _actor_collision_patterns(args),
        )
    return get_vehicle_pair_collision(
        client,
        args.interceptor,
        args.intruder,
        interceptor_object_patterns=args.collision_interceptor_pattern,
        intruder_object_patterns=args.collision_intruder_pattern,
    )


def _collision_is_new(pair_collision, baseline_collision) -> bool:
    interceptor_ts = int(getattr(pair_collision, "interceptor_time_stamp_ns", 0) or 0)
    intruder_ts = int(getattr(pair_collision, "intruder_time_stamp_ns", 0) or 0)
    baseline_interceptor_ts = int(getattr(baseline_collision, "interceptor_time_stamp_ns", 0) or 0)
    baseline_intruder_ts = int(getattr(baseline_collision, "intruder_time_stamp_ns", 0) or 0)
    if interceptor_ts or intruder_ts or baseline_interceptor_ts or baseline_intruder_ts:
        return interceptor_ts > baseline_interceptor_ts or intruder_ts > baseline_intruder_ts
    return True


def main() -> None:
    args = parse_args()
    _validate_runtime_args(args)
    try:
        import airsim
    except ImportError as exc:
        raise SystemExit("Install the AirSim Python package before running this example.") from exc

    client = airsim.MultirotorClient()
    _register_px4_shutdown_stop(client, args)
    try:
        client.confirmConnection()
    except Exception as exc:
        raise SystemExit("Failed to connect to AirSim RPC. Start Blocks first.") from exc
    required_vehicles = [args.interceptor] if args.intruder_actor else [args.interceptor, args.intruder]
    if args.hil_bench_safe:
        args.enable_motion = False
        args.reset = False
        args.climb_to_altitude = False
        args.initial_truth_align = False
        args.skip_list_vehicles = True
        args.skip_collision_check = True
        print("HIL bench-safe mode: disabled reset, arming, climb, initial yaw alignment, collision polling, and motion commands.")
    available = list(required_vehicles) if args.skip_list_vehicles else _require_vehicles(client, required_vehicles)
    print(f"AirSim vehicles: {_format_names(available)}")
    if args.list_vehicles:
        return
    detector = create_detection_provider(args, airsim_module=airsim)

    if args.intruder_actor:
        args.px4_intruder = False
        if args.mesh == "Intruder*":
            args.mesh = _actor_name(args)
    origins = _load_vehicle_origins(args.settings_path, required_vehicles)
    if args.reset and (args.px4_interceptor or args.px4_intruder):
        print("PX4 SITL mode: ignoring AirSim client.reset(); restart PX4/Blocks for a clean SITL session.")
    elif args.reset:
        client.reset()
        time.sleep(1.0)
    if args.hil_bench_safe:
        print("Skipping API control and arming in HIL bench-safe mode.")
    else:
        for vehicle in required_vehicles:
            if _is_px4_vehicle(vehicle, args):
                if args.px4_command_mode != "mavlink_offboard":
                    client.enableApiControl(True, vehicle_name=vehicle)
                _ensure_px4_api_control(client, args, vehicle)
            else:
                client.enableApiControl(True, vehicle_name=vehicle)
                client.armDisarm(True, vehicle_name=vehicle)
    if args.px4_command_mode == "mavlink_offboard":
        offboard = _px4_mavlink_offboard(args)
        if offboard is not None:
            offboard.prime_and_request_offboard(args.px4_offboard_prime_s)
    elif args.px4_command_mode == "mavlink_body_rate":
        offboard = _px4_mavlink_offboard(args)
        if offboard is not None:
            offboard.prime_body_rate_and_request_offboard(args.px4_offboard_prime_s, _body_rate_neutral_thrust(args))
    _prepare_intercept_altitude(client, airsim, required_vehicles, args, origins)
    _px4_keepalive(client, airsim, required_vehicles, args, duration_s=0.8)
    _apply_start_geometry(client, airsim, args, origins)
    _px4_keepalive(client, airsim, required_vehicles, args, duration_s=0.8)
    if args.intruder_actor and not _start_geometry_enabled(args):
        interceptor_kin = _guidance_kinematics(client, args.interceptor, args)
        interceptor_pos = _vehicle_truth_position(client, args.interceptor, origins, kinematics=interceptor_kin)
        actor_pos = interceptor_pos + np.array(
            [
                float(args.start_forward_offset_m or args.start_horizontal_range_m or 100.0),
                float(args.start_lateral_offset_m),
                -float(args.intruder_altitude_offset_m),
            ],
            dtype=float,
        )
        _spawn_or_move_intruder_actor(client, airsim, args, actor_pos, _yaw_deg_from_velocity(_intruder_velocity(args)))
    client.simSetCameraPose(args.camera, _fixed_camera_pose(airsim, args), vehicle_name=args.interceptor)
    print("Fixed strapdown camera pose set once; subsequent alignment rotates interceptor body yaw only.")
    if args.initial_truth_align:
        _align_body_yaw_to_target(client, airsim, args, origins)
    _px4_keepalive(client, airsim, required_vehicles, args, duration_s=0.8)

    config = AirSimDetectionConfig(
        camera_name=args.camera,
        detection_radius_cm=args.detection_radius_cm,
        mesh_name_pattern=args.mesh,
        vehicle_name=args.interceptor,
    )
    if args.detector_source == "airsim" or args.shadow_airsim_detect:
        configure_detection_filter(client, config)
        _configure_actor_detection_aliases(client, airsim, config, args)

    attitude_buffer = AttitudeHistoryBuffer(duration_s=2.0)
    los_filter_config = LOSFilterConfig(
        process_lambda=float(args.los_filter_process_lambda),
        process_lambda_dot=float(args.los_filter_process_lambda_dot),
        measurement_noise=float(args.los_filter_measurement_noise),
        innovation_reject=float(args.los_filter_innovation_reject),
    )
    los_filter = LOSKalmanFilter6D(los_filter_config)
    ttc_filter = ScaleExpansionTTC(
        TTCConfig(min_area=max(0.0, args.ttc_min_area), max_ttc_s=max(0.1, args.ttc_max_s))
    )
    gain_schedule = TTCGainSchedule()
    terminal_extrapolator = TerminalExtrapolator(_terminal_config_from_args(args))
    terminal_image_kf = TerminalImageKF(_terminal_image_kf_config_from_args(args))
    target_dt = 1.0 / args.rate_hz
    intruder_velocity_cmd = _intruder_velocity(args)
    intruder_speed = max(float(np.linalg.norm(intruder_velocity_cmd)), args.intruder_speed)
    speed_cap = args.speed_ratio * intruder_speed
    fixed_vm_gain = float(args.navigation_constant) * float(speed_cap)
    actor_initial_pos: Optional[np.ndarray] = None
    if args.intruder_actor:
        actor_initial_pos = _intruder_truth_position(client, args, origins)
    R_BC = _fixed_camera_R_BC(args)
    intrinsics = _probe_camera_intrinsics(client, airsim, config, args)
    experiment_fields = _experiment_fields(
        args=args,
        experiment_type="strapdown_vision_png",
        speed_cap=speed_cap,
        intruder_velocity_cmd=intruder_velocity_cmd,
        intrinsics=intrinsics,
    )
    if args.detector_source == "airsim":
        _warmup_detection(client, config, args)
    else:
        yolo_device_text = "unknown"
        try:
            yolo_device_text = str(getattr(detector, "runtime_info", {}))
        except Exception:
            pass
        print(
            "YOLO ByteTrack detector active: "
            f"model={args.yolo_model}, class_id={args.yolo_class_id}, tracker={args.yolo_tracker}, "
            f"device_arg={args.yolo_device or 'auto'}, runtime={yolo_device_text}, "
            f"untracked_fallback={args.yolo_allow_untracked_fallback}"
        )
    _px4_keepalive(client, airsim, required_vehicles, args, duration_s=0.8)
    display = _make_display(airsim, args.show_window)

    active_name: Optional[str] = None
    active_track_id: Optional[int] = None
    last_valid_ts: Optional[float] = None
    last_lambda_I: Optional[np.ndarray] = None
    last_omega_los: Optional[np.ndarray] = None
    last_raw_lambda_I: Optional[np.ndarray] = None
    last_raw_lambda_ts: Optional[float] = None
    last_v_cmd: Optional[np.ndarray] = None
    last_wall_t: Optional[float] = None
    last_kin_t: Optional[float] = None
    last_interceptor_vel: Optional[np.ndarray] = None
    yaw_rate_samples: list[tuple[float, float]] = []
    rows: list[dict[str, float | int | str]] = []
    csv_path, plot_path = _output_paths_strapdown(args)
    preview_recorder = _make_preview_recorder(airsim, args, csv_path.with_name(f"{csv_path.stem}_preview"))
    start = time.monotonic()
    last_loop_start = start
    sim_start_t: Optional[float] = None
    frame_id = 0
    hit = False
    bbox_noise_rng = np.random.default_rng(int(args.bbox_noise_seed))
    last_frame_guard_ts: Optional[float] = None
    last_frame_guard_error_ratio = 0.0
    last_frame_guard_area_ratio = 0.0
    last_frame_guard_ttc: Optional[float] = None
    last_detected_ts: Optional[float] = None

    print(
        "frame,t,loop_dt,command_duration,detection_count,detected,range,px_err_x,px_err_y,bbox_area,"
        "ttc,valid,guidance_mode,reason,yaw_rate,body_yaw,cmd_yaw,v_cmd,hit"
    )
    baseline_collision = _collision_snapshot(client, args)
    while True:
        loop_start = time.monotonic()
        loop_dt = target_dt if frame_id == 0 else max(1.0e-6, loop_start - last_loop_start)
        last_loop_start = loop_start
        command_duration = _command_duration(loop_dt, target_dt, args)
        if args.enable_motion and not hit and not args.intruder_actor:
            _command_vehicle_velocity(client, airsim, args.intruder, intruder_velocity_cmd, 0.0, command_duration, args)

        state = client.getMultirotorState(vehicle_name=args.interceptor)
        kin_t_abs = _kinematics_timestamp_s(state) or _kinematics_timestamp_s(state.kinematics_estimated)
        if kin_t_abs is not None and sim_start_t is None:
            sim_start_t = kin_t_abs
        sim_t = (
            kin_t_abs - sim_start_t
            if kin_t_abs is not None and sim_start_t is not None
            else loop_start - start
        )
        if sim_t >= args.duration_s:
            break
        if args.enable_motion and args.intruder_actor and actor_initial_pos is not None and not hit:
            actor_pos = actor_initial_pos + intruder_velocity_cmd * float(sim_t)
            _move_intruder_actor(client, airsim, args, actor_pos, _yaw_deg_from_velocity(intruder_velocity_cmd))
        interceptor_kin = _guidance_kinematics(client, args.interceptor, args)
        interceptor_pos = _vehicle_truth_position(client, args.interceptor, origins, kinematics=interceptor_kin)
        interceptor_vel = _vector_xyz(interceptor_kin.linear_velocity)
        interceptor_accel = _linear_acceleration_mps2(interceptor_kin)
        interceptor_accel_norm = float(np.linalg.norm(interceptor_accel))
        load_factor_g = interceptor_accel_norm / GRAVITY_MPS2
        kin_t = kin_t_abs or _kinematics_timestamp_s(interceptor_kin)
        wall_fps = 0.0 if last_wall_t is None else 1.0 / max(1.0e-6, loop_start - last_wall_t)
        sim_sample_fps = ""
        sim_clock_ratio = ""
        accel_fd_norm = 0.0
        load_factor_fd_g = 0.0
        guidance_dt = target_dt
        if kin_t is not None and last_kin_t is not None:
            kin_dt = max(1.0e-6, kin_t - last_kin_t)
            guidance_dt = kin_dt
            sim_sample_fps = 1.0 / kin_dt
            sim_clock_ratio = kin_dt / max(1.0e-6, loop_start - float(last_wall_t or loop_start))
            if last_interceptor_vel is not None:
                accel_fd = (interceptor_vel - last_interceptor_vel) / kin_dt
                accel_fd_norm = float(np.linalg.norm(accel_fd))
                load_factor_fd_g = accel_fd_norm / GRAVITY_MPS2
        last_wall_t = loop_start
        if kin_t is not None:
            last_kin_t = kin_t
        last_interceptor_vel = np.array(interceptor_vel, dtype=float)
        intruder_pos = np.full(3, np.nan)
        rel = np.full(3, np.nan)
        range_m = float("nan")
        if args.diagnostic_truth:
            intruder_pos = _intruder_truth_position(client, args, origins)
            rel = intruder_pos - interceptor_pos
            range_m = float(np.linalg.norm(rel))
        pair_collision = _collision_snapshot(client, args)
        raw_collision_hit = bool(pair_collision.collided)
        collision_new = _collision_is_new(pair_collision, baseline_collision)
        collision_range_gate_m = max(0.0, float(args.collision_max_range_m))
        collision_range_ok = (
            not args.diagnostic_truth
            or not np.isfinite(range_m)
            or range_m <= collision_range_gate_m
        )
        hit = raw_collision_hit and collision_new and collision_range_ok
        if raw_collision_hit and not collision_new:
            pair_collision_reason = f"{pair_collision.reason}:stale_time_reject"
        elif raw_collision_hit and not collision_range_ok:
            pair_collision_reason = f"{pair_collision.reason}:range_gate_reject"
        else:
            pair_collision_reason = pair_collision.reason

        R_IB = airsim_orientation_to_R_IB(state.kinematics_estimated.orientation)
        roll_rad, pitch_rad, yaw_rad = _vehicle_euler_rad(airsim, state.kinematics_estimated.orientation)
        attitude_buffer.push(AttitudeSample(timestamp=sim_t, R_IB=R_IB))
        body_yaw_deg = _body_yaw_deg(airsim, state.kinematics_estimated.orientation)
        camera_world_pos = interceptor_pos + R_IB @ _camera_offset_body(args)

        detector_start = time.monotonic()
        detector_frame = detector.detect(
            client=client,
            config=config,
            frame_id=frame_id,
            exposure_ts=sim_t,
            active_name=active_name,
            active_track_id=active_track_id,
        )
        detector_elapsed_s = time.monotonic() - detector_start
        detector_fps = 1.0 / max(1.0e-6, detector_elapsed_s)
        detections = detector_frame.detections
        detection_count = len(detections)
        detection_names = _detection_names(detections)
        detection = detector_frame.selected
        detected = detection is not None
        detector_stats = detector_frame.stats
        detector_image = detector_frame.image_bgr
        shadow_airsim_stats = (
            _shadow_airsim_detection(client, config, active_name, intrinsics)
            if args.shadow_airsim_detect
            else _empty_shadow_airsim_stats()
        )
        px_err_x = 0.0
        px_err_y = 0.0
        yaw_rate_cmd_deg_s = 0.0
        ttc_value = ""
        reason = ""
        valid = False
        g_eval = np.zeros(3)
        a_cmd = np.zeros(3)
        accel_integral_dt = 0.0
        v_cmd_pre_guard = np.zeros(3)
        v_cmd_integrated = np.zeros(3)
        body_rate_command_active = 0
        body_rate_accel_B = np.zeros(3)
        body_rate_roll_sp_rad = 0.0
        body_rate_pitch_sp_rad = 0.0
        body_rate_cmd_rad_s = np.zeros(3)
        body_rate_thrust = _body_rate_neutral_thrust(args)
        body_rate_speed_hold_accel = np.zeros(3)
        body_rate_control_accel = np.zeros(3)
        los_quality = 0.0
        ttc_quality = 0.0
        ttc_area = 0.0
        ttc_area_dot = 0.0
        lambda_I = last_lambda_I
        omega_los = last_omega_los
        guidance_gain = 0.0
        bbox_area = 0.0
        guidance_mode = "invalid"
        detection_score = 0.0
        center: Optional[tuple[float, float]] = None
        frame_detection = None
        bbox_noise_dx_px = 0.0
        bbox_noise_dy_px = 0.0
        bbox_noise_area_scale = 1.0
        bbox_raw_xyxy = (0.0, 0.0, 0.0, 0.0)
        bbox_noisy_xyxy = (0.0, 0.0, 0.0, 0.0)
        detection_clipped = False
        bbox_left_clipped = False
        bbox_right_clipped = False
        bbox_top_clipped = False
        bbox_bottom_clipped = False
        pitch_measurement_rejected = False
        lambda_I_measured: Optional[np.ndarray] = None
        lambda_raw: Optional[np.ndarray] = None
        omega_raw: Optional[np.ndarray] = None
        los_dt_s: float | str = ""
        los_angle_step_deg: float | str = ""
        los_source = "none"
        track_switched = False

        if detection is None:
            reason = str(detector_stats.get("detector_reject_reason") or "no_detection")
        else:
            active_name = getattr(detection, "name", None) or active_name
            frame_detection = detector_frame.frame_detection
            if frame_detection is None:
                reason = str(detector_stats.get("detector_reject_reason") or "no_detection")
            else:
                previous_track_id = active_track_id
                current_track_id = int(frame_detection.track_id)
                pseudo_track_switch = (
                    bool(args.ignore_pseudo_track_switch)
                    and previous_track_id is not None
                    and (int(previous_track_id) < 0 or current_track_id < 0)
                )
                track_switched = (
                    previous_track_id is not None
                    and current_track_id != int(previous_track_id)
                    and not pseudo_track_switch
                )
                if track_switched:
                    los_filter.reset()
                    ttc_filter.reset()
                    terminal_image_kf.reset()
                    terminal_extrapolator = TerminalExtrapolator(_terminal_config_from_args(args))
                    yaw_rate_samples.clear()
                    last_lambda_I = None
                    last_omega_los = None
                    last_raw_lambda_I = None
                    last_raw_lambda_ts = None
                    last_valid_ts = None
                    last_detected_ts = None
                    last_frame_guard_ts = None
                    lambda_I = None
                    omega_los = None
                active_track_id = current_track_id
        if frame_detection is not None:
            frame_detection, bbox_noise = _apply_bbox_noise(frame_detection, intrinsics, bbox_noise_rng, args)
            bbox_noise_dx_px = float(bbox_noise["dx"])
            bbox_noise_dy_px = float(bbox_noise["dy"])
            bbox_noise_area_scale = float(bbox_noise["area_scale"])
            bbox_raw_xyxy = tuple(float(value) for value in bbox_noise["raw_bbox"])
            bbox_noisy_xyxy = tuple(float(value) for value in bbox_noise["noisy_bbox"])
            detection_score = frame_detection.score
            bbox_area = frame_detection.area
            center = frame_detection.center
            px_err_x = center[0] - intrinsics.cx
            px_err_y = center[1] - intrinsics.cy
            clip_flags = frame_detection.clip_flags(intrinsics.width, intrinsics.height)
            bbox_left_clipped = bool(clip_flags["left"])
            bbox_right_clipped = bool(clip_flags["right"])
            bbox_top_clipped = bool(clip_flags["top"])
            bbox_bottom_clipped = bool(clip_flags["bottom"])
            detection_clipped = any(clip_flags.values())
            pitch_measurement_rejected = bool(args.reject_top_clipped_pitch and bbox_top_clipped)
            yaw_rate_cmd_deg_s = _yaw_rate_from_pixel_error(px_err_x, intrinsics, args)

            lookup = attitude_buffer.lookup(frame_detection.exposure_ts)
            if not lookup.valid or lookup.sample is None:
                reason = lookup.reason or "attitude_lookup_failed"
            elif pitch_measurement_rejected:
                reason = "bbox_top_clipped"
            else:
                los_C = camera_ray_from_pixel(*center, intrinsics)
                lambda_I_measured = los_camera_to_inertial(los_C, R_BC, lookup.sample.R_IB)
                lambda_raw = normalize(lambda_I_measured)
                if last_raw_lambda_I is not None and last_raw_lambda_ts is not None:
                    raw_dt = max(1.0e-3, frame_detection.exposure_ts - last_raw_lambda_ts)
                    los_dt_s = raw_dt
                    raw_delta = project_perpendicular((lambda_raw - last_raw_lambda_I) / raw_dt, lambda_raw)
                    omega_raw = np.cross(lambda_raw, raw_delta)
                    dot_raw = float(np.clip(np.dot(last_raw_lambda_I, lambda_raw), -1.0, 1.0))
                    los_angle_step_deg = float(np.degrees(np.arccos(dot_raw)))
                else:
                    omega_raw = np.zeros(3, dtype=float)
                last_raw_lambda_I = np.array(lambda_raw, dtype=float)
                last_raw_lambda_ts = frame_detection.exposure_ts

                if args.los_filter:
                    los = los_filter.update(frame_detection.exposure_ts, lambda_I_measured)
                    los_source = "kalman"
                    los_valid = los.valid
                    los_lambda = los.lambda_I
                    los_omega = los.omega_los
                    los_quality = los.quality
                    los_reject_reason = los.reject_reason
                else:
                    los_source = "raw_fd"
                    los_valid = True
                    los_lambda = lambda_raw
                    los_omega = omega_raw
                    los_quality = 1.0
                    los_reject_reason = None
                if args.guidance_law == "ttc_png":
                    ttc = ttc_filter.update(frame_detection, intrinsics.width, intrinsics.height)
                    ttc_quality = ttc.quality
                    ttc_area = ttc.area_filtered
                    ttc_area_dot = ttc.area_dot_filtered
                    if ttc.ttc is not None:
                        ttc_value = f"{ttc.ttc:.3f}"
                if not los_valid:
                    reason = los_reject_reason or "los_invalid"
                elif args.guidance_law == "fixed_vm_png":
                    guidance_gain = fixed_vm_gain
                    omega_los = los_omega
                    lambda_I = los_lambda
                    g_eval = guidance_gain * np.cross(omega_los, lambda_I)
                    valid = True
                    guidance_mode = "fixed_vm_png"
                    last_valid_ts = sim_t
                    last_lambda_I = lambda_I
                    last_omega_los = omega_los
                elif not ttc.valid or ttc.ttc is None:
                    lambda_I = los_lambda
                    omega_los = los_omega
                    reason = ttc.reject_reason or "ttc_invalid"
                    if bool(args.ttc_soft_guidance):
                        guidance_gain = fixed_vm_gain
                        g_eval = guidance_gain * np.cross(omega_los, lambda_I)
                        valid = True
                        guidance_mode = "ttc_soft_vm"
                        last_valid_ts = sim_t
                        last_lambda_I = lambda_I
                        last_omega_los = omega_los
                    elif _los_fallback_allowed(reason, args):
                        guidance_gain = max(0.0, args.los_fallback_gain)
                        g_eval = guidance_gain * omega_los
                        valid = True
                        guidance_mode = "los_fallback"
                        last_valid_ts = sim_t
                        last_lambda_I = lambda_I
                        last_omega_los = omega_los
                else:
                    gain = gain_schedule.gain(ttc.ttc)
                    omega_los = los_omega
                    lambda_I = los_lambda
                    if bool(args.ttc_soft_guidance):
                        gain_scale = gain / max(1.0e-6, float(gain_schedule.max_gain))
                        min_scale = float(np.clip(float(args.ttc_soft_min_gain_scale), 0.0, 1.0))
                        guidance_gain = fixed_vm_gain * float(np.clip(gain_scale, min_scale, 1.0))
                        g_eval = guidance_gain * np.cross(omega_los, lambda_I)
                    else:
                        guidance_gain = gain
                        g_eval = gain * omega_los
                    valid = True
                    guidance_mode = "ttc_png"
                    last_valid_ts = sim_t
                    last_lambda_I = lambda_I
                    last_omega_los = omega_los

        image_kf = terminal_image_kf.update(
            timestamp=sim_t,
            center=center,
            intrinsics=intrinsics,
            detected=detected,
            measurement_valid=detected,
            clipped=detection_clipped or pitch_measurement_rejected,
            track_id=(
                None
                if (
                    not detected
                    or active_track_id is None
                    or (bool(args.ignore_pseudo_track_switch) and int(active_track_id) < 0)
                )
                else active_track_id
            ),
        )

        image_kf_guidance_used = False
        if (
            not valid
            and bool(args.terminal_image_kf_guidance)
            and image_kf.valid
            and image_kf.mode == IMAGE_KF_PREDICT
            and _is_short_visual_loss_reason(reason)
        ):
            predicted_lambda_I, predicted_omega_los = _image_kf_los_guidance_estimate(image_kf, intrinsics, R_BC, R_IB)
            if predicted_lambda_I is not None and predicted_omega_los is not None:
                lambda_I = predicted_lambda_I
                omega_los = predicted_omega_los
                if args.guidance_law == "fixed_vm_png" or bool(args.ttc_soft_guidance):
                    guidance_gain = fixed_vm_gain
                    g_eval = guidance_gain * np.cross(omega_los, lambda_I)
                    guidance_mode = "fixed_vm_png_kf_predict" if args.guidance_law == "fixed_vm_png" else "ttc_soft_vm_kf_predict"
                else:
                    gain = gain_schedule.gain(max(1.0, min(6.0, image_kf.age_s + 1.0)))
                    guidance_gain = gain
                    g_eval = gain * omega_los
                    guidance_mode = "ttc_png_kf_predict"
                valid = True
                los_source = "image_kf_predict"
                los_quality = image_kf.quality
                reason = "image_kf_predict"
                last_valid_ts = sim_t
                last_lambda_I = lambda_I
                last_omega_los = omega_los
                image_kf_guidance_used = True

        if not valid and last_valid_ts is not None and sim_t - last_valid_ts <= args.coast_timeout_s:
            lambda_I = last_lambda_I
            omega_los = last_omega_los

        image_error_ratio = _image_error_ratio(px_err_x, px_err_y, intrinsics) if detected else 0.0
        area_ratio = _bbox_area_ratio(bbox_area, intrinsics) if detected else 0.0
        ttc_numeric: Optional[float] = None
        if ttc_value != "":
            try:
                ttc_numeric = float(ttc_value)
            except (TypeError, ValueError):
                ttc_numeric = None
        if bool(args.frame_guard):
            ttc_close = ttc_numeric is not None and ttc_numeric > 0.0 and ttc_numeric <= max(0.0, float(args.frame_guard_ttc_mid_s))
            current_guard_risk = (
                detected
                and (
                    image_error_ratio >= max(0.0, float(args.frame_guard_exit_error_ratio))
                    or area_ratio >= max(0.0, float(args.frame_guard_area_mid_ratio))
                    or ttc_close
                    or detection_clipped
                    or pitch_measurement_rejected
                )
            )
            if current_guard_risk:
                last_frame_guard_ts = sim_t
                last_frame_guard_error_ratio = image_error_ratio
                last_frame_guard_area_ratio = area_ratio
                last_frame_guard_ttc = ttc_numeric
        if image_kf.valid:
            try:
                kf_center = center_from_angle_error(np.array([image_kf.theta_x, image_kf.theta_y], dtype=float), intrinsics)
                image_error_ratio_kf = _image_error_ratio(kf_center[0] - intrinsics.cx, kf_center[1] - intrinsics.cy, intrinsics)
            except Exception:
                image_error_ratio_kf = 0.0
        else:
            image_error_ratio_kf = 0.0
        guard_hold_age_s: float | str = ""
        guard_hold_active = False
        if (
            bool(args.frame_guard)
            and last_frame_guard_ts is not None
            and sim_t - last_frame_guard_ts <= max(0.0, float(args.frame_guard_hold_s))
        ):
            guard_hold_active = True
            guard_hold_age_s = float(sim_t - last_frame_guard_ts)
        effective_image_error_ratio = image_error_ratio
        effective_area_ratio = area_ratio
        effective_ttc = ttc_numeric
        if guard_hold_active:
            effective_image_error_ratio = max(effective_image_error_ratio, last_frame_guard_error_ratio)
            effective_area_ratio = max(effective_area_ratio, last_frame_guard_area_ratio)
            if effective_ttc is None:
                effective_ttc = last_frame_guard_ttc
        if image_kf.valid and not detected:
            effective_image_error_ratio = max(effective_image_error_ratio, image_error_ratio_kf)
        if detected and frame_detection is not None:
            last_detected_ts = sim_t
        frame_guard_yaw_px_err_x = px_err_x
        frame_guard_px_err_y = px_err_y
        if not detected and image_kf.valid:
            try:
                frame_guard_yaw_px_err_x = float(intrinsics.fx * np.tan(float(image_kf.theta_x)))
                frame_guard_px_err_y = float(intrinsics.fy * np.tan(float(image_kf.theta_y)))
            except Exception:
                frame_guard_yaw_px_err_x = 0.0
                frame_guard_px_err_y = 0.0
        scheduled_speed_cap, speed_schedule_mode = _scheduled_speed_cap(
            base_speed_cap=speed_cap,
            intruder_speed=intruder_speed,
            error_ratio=effective_image_error_ratio,
            area_ratio=effective_area_ratio,
            ttc_s=effective_ttc,
            args=args,
        )
        frame_centering_state, frame_centering_active, frame_centering_reason = _frame_centering_state(
            detected=detected,
            detection_clipped=detection_clipped or pitch_measurement_rejected,
            image_error_ratio=effective_image_error_ratio,
            area_ratio=effective_area_ratio,
            ttc_s=effective_ttc,
            image_kf_valid=bool(image_kf.valid),
            guard_hold_active=guard_hold_active,
            guard_hold_error_ratio=last_frame_guard_error_ratio,
            guard_hold_area_ratio=last_frame_guard_area_ratio,
            last_detected_ts=last_detected_ts,
            timestamp=sim_t,
            args=args,
        )
        raw_yaw_rate_cmd_deg_s = yaw_rate_cmd_deg_s
        _append_yaw_rate_sample(yaw_rate_samples, sim_t, raw_yaw_rate_cmd_deg_s, valid, args)
        candidate_v_cmd, a_cmd, accel_integral_dt = _candidate_guidance_velocity(
            interceptor_vel,
            lambda_I,
            omega_los,
            guidance_gain,
            scheduled_speed_cap,
            guidance_dt,
            valid,
            args,
        )
        v_cmd_integrated = np.array(candidate_v_cmd, dtype=float)
        v_cmd_pre_guard = np.array(candidate_v_cmd, dtype=float)
        frame_centering_used_last_v_cmd = 0
        frame_centering_loss_hold_velocity_source = "guidance"
        if (
            frame_centering_state == "loss_hold"
            and bool(args.frame_centering_loss_hold_last_velocity)
            and last_v_cmd is not None
        ):
            candidate_v_cmd = np.array(last_v_cmd, dtype=float)
            frame_centering_used_last_v_cmd = 1
            frame_centering_loss_hold_velocity_source = "last_v_cmd"
        frame_guard_vertical_bias = (
            _frame_guard_vertical_bias(frame_guard_px_err_y, intrinsics, args)
            if (detected or (image_kf.valid and not detected))
            else 0.0
        )
        if frame_guard_vertical_bias:
            candidate_v_cmd[2] = float(
                np.clip(
                    candidate_v_cmd[2] + frame_guard_vertical_bias,
                    -args.max_vision_vertical_speed,
                    args.max_vision_vertical_speed,
                )
            )
            v_cmd_pre_guard = np.array(candidate_v_cmd, dtype=float)
        candidate_v_cmd, frame_guard_active, frame_guard_mode = _apply_frame_guard_velocity(
            candidate_v_cmd,
            lambda_I=lambda_I,
            scheduled_speed_cap=scheduled_speed_cap,
            error_ratio=effective_image_error_ratio,
            area_ratio=effective_area_ratio,
            speed_schedule_mode=speed_schedule_mode,
            args=args,
        )
        candidate_v_cmd, frame_centering_candidate_speed, frame_centering_lateral_scale, frame_centering_lateral_limit = _apply_frame_centering_velocity(
            candidate_v_cmd,
            lambda_I=lambda_I,
            scheduled_speed_cap=scheduled_speed_cap,
            intruder_speed=intruder_speed,
            state=frame_centering_state,
            args=args,
        )
        terminal_result = terminal_extrapolator.update(
            timestamp=sim_t,
            detected=detected,
            measurement_valid=valid,
            measurement_score=detection_score,
            bbox_area=bbox_area,
            image_width=intrinsics.width,
            image_height=intrinsics.height,
            reject_reason=reason,
            v_cmd=candidate_v_cmd,
            lambda_I=lambda_I,
            omega_los=omega_los if valid else None,
            speed_cap=scheduled_speed_cap,
            max_vertical_speed=args.max_vision_vertical_speed,
            gimbal_at_limit=False,
            safety_ok=_airsim_safety_ok(client, args.interceptor),
            soft_measurement_valid=image_kf.valid,
        )
        v_cmd = terminal_result.v_cmd
        if frame_centering_state == "loss_hold" and frame_centering_used_last_v_cmd:
            v_cmd = np.array(last_v_cmd, dtype=float)
            frame_centering_loss_hold_velocity_source = "last_v_cmd_final"
        v_cmd, frame_guard_active_post, frame_guard_mode_post = _apply_frame_guard_velocity(
            v_cmd,
            lambda_I=lambda_I,
            scheduled_speed_cap=scheduled_speed_cap,
            error_ratio=effective_image_error_ratio,
            area_ratio=effective_area_ratio,
            speed_schedule_mode=speed_schedule_mode,
            args=args,
        )
        if frame_guard_active_post:
            frame_guard_active = frame_guard_active_post
            frame_guard_mode = frame_guard_mode_post
        v_cmd, frame_centering_final_speed, frame_centering_lateral_scale, frame_centering_lateral_limit = _apply_frame_centering_velocity(
            v_cmd,
            lambda_I=lambda_I,
            scheduled_speed_cap=scheduled_speed_cap,
            intruder_speed=intruder_speed,
            state=frame_centering_state,
            args=args,
        )
        if args.px4_interceptor:
            v_cmd = _px4_limited_velocity(v_cmd, args)
        if terminal_result.using_blind_push:
            guidance_mode = "blind_push"
            reason = terminal_result.reason
        elif valid:
            last_v_cmd = np.array(v_cmd, dtype=float)
        (
            yaw_rate_cmd_deg_s,
            yaw_rate_blind_base_deg_s,
            yaw_rate_blind_sample_count,
            yaw_rate_blind_decay,
        ) = _terminal_yaw_rate_command(
            current_yaw_rate_deg_s=raw_yaw_rate_cmd_deg_s,
            samples=yaw_rate_samples,
            timestamp=sim_t,
            terminal_state=terminal_result.state,
            using_blind_push=terminal_result.using_blind_push,
            blind_elapsed_s=terminal_result.blind_elapsed_s,
            args=args,
        )
        yaw_rate_source = "measurement" if detected else "none"
        image_kf_takeover_allowed, image_kf_takeover_reason = _image_kf_takeover_allowed(
            image_kf,
            terminal_result,
            reason,
            detected,
            terminal_result.area_ratio,
            args,
            profile="strapdown",
        )
        if image_kf_takeover_allowed:
            yaw_rate_cmd_deg_s = _kf_yaw_rate_command(image_kf, args)
            yaw_rate_blind_base_deg_s = yaw_rate_cmd_deg_s
            yaw_rate_blind_sample_count = 0
            yaw_rate_blind_decay = image_kf.quality
            yaw_rate_source = "kf"
        if image_kf_guidance_used and yaw_rate_source == "none":
            yaw_rate_cmd_deg_s = _kf_yaw_rate_command(image_kf, args)
            yaw_rate_source = "kf_guidance"
        elif terminal_result.using_blind_push:
            yaw_rate_source = "window_hold"
        elif terminal_result.state == TERMINAL_VISUAL:
            yaw_rate_source = "measurement_scaled"
        yaw_rate_cmd_deg_s, frame_guard_yaw_active = _frame_guard_yaw_rate(
            yaw_rate_cmd_deg_s,
            frame_guard_yaw_px_err_x,
            intrinsics,
            args,
        )
        if frame_guard_yaw_active and yaw_rate_source in {"measurement", "none"}:
            yaw_rate_source = "frame_guard"
        (
            yaw_rate_cmd_deg_s,
            yaw_rate_source,
            frame_centering_yaw_decay,
            frame_centering_yaw_sample_count,
        ) = _frame_centering_yaw_rate(
            yaw_rate_deg_s=yaw_rate_cmd_deg_s,
            px_err_x=frame_guard_yaw_px_err_x,
            intrinsics=intrinsics,
            state=frame_centering_state,
            timestamp=sim_t,
            yaw_rate_samples=yaw_rate_samples,
            image_kf=image_kf,
            yaw_rate_source=yaw_rate_source,
            args=args,
        )

        stop_requested = _draw_detection_window_strapdown(
            display,
            client,
            config,
            detections,
            detection,
            intrinsics,
            yaw_rate_cmd_deg_s,
            valid,
            reason,
            ttc_value,
            args,
            image_bgr=detector_image,
        )
        preview_lines = _preview_lines(
            profile="strapdown",
            detections=detections,
            selected=detection,
            valid=valid,
            reason=reason,
            ttc_text=ttc_value,
            terminal_state=terminal_result.state,
            image_kf_mode=image_kf.mode,
            range_m=range_m,
            control_line=f"yaw_rate={yaw_rate_cmd_deg_s:.1f} deg/s body_yaw={body_yaw_deg:.1f}",
            hit=hit,
        )
        _record_detection_preview(
            preview_recorder,
            client,
            config,
            detections,
            detection,
            intrinsics,
            preview_lines,
            args,
            frame_id=frame_id,
            terminal_state=terminal_result.state,
            guidance_mode=guidance_mode,
            reason=reason,
            hit=hit,
            image_bgr=detector_image,
            timestamp=sim_t,
        )

        cmd_yaw_deg = _yaw_deg_from_velocity(v_cmd)
        yaw_error_deg = _wrap_angle_deg(cmd_yaw_deg - body_yaw_deg)
        target_bearing_deg = ""
        target_body_bearing_deg = ""
        if args.diagnostic_truth and np.all(np.isfinite(rel[:2])) and float(np.linalg.norm(rel[:2])) > 1.0e-6:
            target_bearing = _bearing_deg_from_xy(rel[:2])
            target_bearing_deg = target_bearing
            target_body_bearing_deg = _wrap_angle_deg(target_bearing - body_yaw_deg)
        if args.enable_motion:
            if args.guidance_output_mode == "accel_body_rate":
                body_rate_control_accel, body_rate_speed_hold_accel = _body_rate_control_acceleration(
                    png_acceleration_I=a_cmd,
                    current_velocity_I=interceptor_vel,
                    velocity_reference_I=v_cmd,
                    args=args,
                )
                body_rate_result = _body_rate_command_from_accel(
                    body_rate_control_accel,
                    R_IB,
                    roll_rad,
                    pitch_rad,
                    yaw_rate_cmd_deg_s,
                    args,
                )
                body_rate_accel_B = np.asarray(body_rate_result["accel_B"], dtype=float)
                body_rate_roll_sp_rad = float(body_rate_result["roll_sp_rad"])
                body_rate_pitch_sp_rad = float(body_rate_result["pitch_sp_rad"])
                body_rate_cmd_rad_s = np.asarray(body_rate_result["body_rates_rad_s"], dtype=float)
                body_rate_thrust = float(body_rate_result["thrust"])
                body_rate_command_active = 1
                _command_interceptor_body_rate(body_rate_cmd_rad_s, body_rate_thrust, args)
            else:
                _command_interceptor_velocity(client, airsim, v_cmd, yaw_rate_cmd_deg_s, command_duration, args)
        post_command_vel = np.full(3, np.nan)
        post_command_pos = np.full(3, np.nan)
        if args.enable_motion and args.px4_interceptor:
            try:
                post_command_kin = _guidance_kinematics(client, args.interceptor, args)
                post_command_vel = _vector_xyz(post_command_kin.linear_velocity)
                post_command_pos = _vehicle_truth_position(client, args.interceptor, origins, kinematics=post_command_kin)
            except Exception:
                pass

        frame_elapsed = time.monotonic() - loop_start
        vertical_error_sign = ""
        vertical_command_consistent = ""
        if args.diagnostic_truth and np.all(np.isfinite(rel)):
            vertical_error_sign = float(np.sign(rel[2]))
            if abs(float(rel[2])) <= 1.0e-6 or abs(float(v_cmd[2])) <= 1.0e-6:
                vertical_command_consistent = 1
            else:
                vertical_command_consistent = int(np.sign(rel[2]) == np.sign(v_cmd[2]))
        rows.append(
            {
                "frame": frame_id,
                **experiment_fields,
                "t": sim_t,
                "wall_t": loop_start - start,
                "loop_dt": loop_dt,
                "frame_elapsed": frame_elapsed,
                "wall_fps": wall_fps,
                "detector_elapsed_s": detector_elapsed_s,
                "detector_fps": detector_fps,
                "sim_time_s": "" if kin_t is None else kin_t,
                "sim_sample_fps": sim_sample_fps,
                "sim_clock_ratio": sim_clock_ratio,
                "command_duration": command_duration,
                "target_hz": args.rate_hz,
                "guidance_law": args.guidance_law,
                "guidance_output_mode": args.guidance_output_mode,
                "navigation_constant": float(args.navigation_constant),
                "vm_png_gain": fixed_vm_gain,
                "max_guidance_accel_mps2": float(args.max_guidance_accel_mps2),
                "min_speed_ratio": float(args.min_speed_ratio),
                "accel_integral_reset_on_invalid": int(args.accel_integral_reset_on_invalid),
                "ttc_used_for_guidance": int(args.guidance_law == "ttc_png" and not args.ttc_soft_guidance),
                "ttc_soft_guidance": int(args.ttc_soft_guidance),
                "ttc_soft_min_gain_scale": float(args.ttc_soft_min_gain_scale),
                "scheduled_speed_cap": scheduled_speed_cap,
                "speed_schedule_mode": speed_schedule_mode,
                "frame_guard_enabled": int(args.frame_guard),
                "frame_guard_active": frame_guard_active,
                "frame_guard_mode": frame_guard_mode,
                "frame_guard_yaw_active": frame_guard_yaw_active,
                "frame_guard_vertical_bias": frame_guard_vertical_bias,
                "frame_guard_enter_error_ratio": float(args.frame_guard_enter_error_ratio),
                "frame_guard_exit_error_ratio": float(args.frame_guard_exit_error_ratio),
                "frame_guard_area_mid_ratio": float(args.frame_guard_area_mid_ratio),
                "frame_guard_area_ratio": float(args.frame_guard_area_ratio),
                "frame_guard_ttc_mid_s": float(args.frame_guard_ttc_mid_s),
                "frame_guard_ttc_terminal_s": float(args.frame_guard_ttc_terminal_s),
                "frame_guard_hold_s": float(args.frame_guard_hold_s),
                "frame_guard_hold_active": int(guard_hold_active),
                "frame_guard_hold_age_s": guard_hold_age_s,
                "frame_guard_effective_error_ratio": effective_image_error_ratio,
                "frame_guard_effective_area_ratio": effective_area_ratio,
                "frame_guard_effective_ttc": "" if effective_ttc is None else float(effective_ttc),
                "frame_guard_image_kf_error_ratio": image_error_ratio_kf,
                "frame_guard_yaw_px_err_x": frame_guard_yaw_px_err_x,
                "frame_guard_px_err_y": frame_guard_px_err_y,
                "frame_guard_min_speed_ratio": float(args.frame_guard_min_speed_ratio),
                "frame_guard_mid_speed_ratio": float(args.frame_guard_mid_speed_ratio),
                "frame_centering_enabled": int(args.frame_centering),
                "frame_centering_active": int(frame_centering_active),
                "frame_centering_state": frame_centering_state,
                "frame_centering_reason": frame_centering_reason,
                "frame_centering_enter_error_ratio": float(args.frame_centering_enter_error_ratio),
                "frame_centering_terminal_error_ratio": float(args.frame_centering_terminal_error_ratio),
                "frame_centering_area_ratio": float(args.frame_centering_area_ratio),
                "frame_centering_loss_hold_s": float(args.frame_centering_loss_hold_s),
                "frame_centering_speed_ratio": float(args.frame_centering_speed_ratio),
                "terminal_capture_speed_ratio": float(args.terminal_capture_speed_ratio),
                "frame_centering_candidate_speed": frame_centering_candidate_speed,
                "frame_centering_final_speed": frame_centering_final_speed,
                "frame_centering_lateral_scale": frame_centering_lateral_scale,
                "frame_centering_lateral_limit": frame_centering_lateral_limit,
                "frame_centering_yaw_decay": frame_centering_yaw_decay,
                "frame_centering_yaw_sample_count": frame_centering_yaw_sample_count,
                "frame_centering_loss_hold_last_velocity": int(args.frame_centering_loss_hold_last_velocity),
                "frame_centering_used_last_v_cmd": frame_centering_used_last_v_cmd,
                "frame_centering_loss_hold_velocity_source": frame_centering_loss_hold_velocity_source,
                "last_detected_age_s": "" if last_detected_ts is None else float(sim_t - last_detected_ts),
                "detection_count": detection_count,
                "detection_names": detection_names,
                "detected": int(detected),
                "target_name": active_name or "",
                "target_track_id": "" if active_track_id is None else int(active_track_id),
                "track_switched": int(track_switched),
                "detector_source": str(detector_stats.get("detector_source", args.detector_source)),
                "detector_reject_reason": str(detector_stats.get("detector_reject_reason", "")),
                "detector_raw_count": detector_stats.get("detector_raw_count", ""),
                "detector_class_filtered_count": detector_stats.get("detector_class_filtered_count", ""),
                "detector_track_filtered_count": detector_stats.get("detector_track_filtered_count", ""),
                "yolo_raw_count": detector_stats.get("yolo_raw_count", ""),
                "yolo_class_filtered_count": detector_stats.get("yolo_class_filtered_count", ""),
                "yolo_track_filtered_count": detector_stats.get("yolo_track_filtered_count", ""),
                "yolo_track_missing_count": detector_stats.get("yolo_track_missing_count", ""),
                "yolo_selected_track_id": detector_stats.get("yolo_selected_track_id", ""),
                "yolo_selected_class_id": detector_stats.get("yolo_selected_class_id", ""),
                "yolo_selected_score": detector_stats.get("yolo_selected_score", ""),
                "yolo_selected_source": detector_stats.get("yolo_selected_source", ""),
                "yolo_requested_device": detector_stats.get("yolo_requested_device", ""),
                "yolo_runtime_device": detector_stats.get("yolo_runtime_device", ""),
                "yolo_cuda_available": detector_stats.get("yolo_cuda_available", ""),
                "yolo_gpu_name": detector_stats.get("yolo_gpu_name", ""),
                "yolo_allow_untracked_fallback": detector_stats.get("yolo_allow_untracked_fallback", ""),
                "yolo_used_untracked_fallback": detector_stats.get("yolo_used_untracked_fallback", ""),
                "yolo_single_target_mode": detector_stats.get("yolo_single_target_mode", ""),
                "yolo_single_target_selected": detector_stats.get("yolo_single_target_selected", ""),
                "yolo_single_target_distance_px": detector_stats.get("yolo_single_target_distance_px", ""),
                **shadow_airsim_stats,
                "image_width": intrinsics.width,
                "image_height": intrinsics.height,
                "range": range_m,
                "horizontal_range": float(np.linalg.norm(rel[:2])) if args.diagnostic_truth else "",
                "vertical_error": float(rel[2]) if args.diagnostic_truth else "",
                "vertical_error_sign": vertical_error_sign,
                "v_cmd_z_sign": float(np.sign(v_cmd[2])),
                "vertical_command_consistent": vertical_command_consistent,
                "hit": int(hit),
                "collision_reason": pair_collision_reason,
                "collision_raw_hit": int(raw_collision_hit),
                "collision_new": int(collision_new),
                "collision_range_gate_m": collision_range_gate_m,
                "collision_range_ok": int(collision_range_ok),
                "collision_accepted": int(hit),
                "collision_interceptor_time_s": _collision_time_stamp_s(pair_collision.interceptor_time_stamp_ns),
                "collision_intruder_time_s": _collision_time_stamp_s(pair_collision.intruder_time_stamp_ns),
                "collision_baseline_interceptor_time_s": _collision_time_stamp_s(baseline_collision.interceptor_time_stamp_ns),
                "collision_baseline_intruder_time_s": _collision_time_stamp_s(baseline_collision.intruder_time_stamp_ns),
                "interceptor_collision_object": pair_collision.interceptor_object_name,
                "intruder_collision_object": pair_collision.intruder_object_name,
                "intruder_actor_scale_x": _actor_scale_xyz(args)[0],
                "intruder_actor_scale_y": _actor_scale_xyz(args)[1],
                "intruder_actor_scale_z": _actor_scale_xyz(args)[2],
                "pixel_error_x": px_err_x,
                "pixel_error_y": px_err_y,
                "bbox_area": bbox_area,
                "bbox_area_ratio": area_ratio,
                "image_error_ratio": image_error_ratio,
                "bbox_noise_enabled": int(args.bbox_noise),
                "bbox_noise_center_sigma_px": float(args.bbox_center_noise_px),
                "bbox_noise_area_sigma_ratio": float(args.bbox_area_noise_ratio),
                "bbox_noise_seed": int(args.bbox_noise_seed),
                "bbox_noise_dx_px": bbox_noise_dx_px,
                "bbox_noise_dy_px": bbox_noise_dy_px,
                "bbox_noise_area_scale": bbox_noise_area_scale,
                "bbox_raw_x1": bbox_raw_xyxy[0],
                "bbox_raw_y1": bbox_raw_xyxy[1],
                "bbox_raw_x2": bbox_raw_xyxy[2],
                "bbox_raw_y2": bbox_raw_xyxy[3],
                "bbox_noisy_x1": bbox_noisy_xyxy[0],
                "bbox_noisy_y1": bbox_noisy_xyxy[1],
                "bbox_noisy_x2": bbox_noisy_xyxy[2],
                "bbox_noisy_y2": bbox_noisy_xyxy[3],
                "bbox_left_clipped": int(bbox_left_clipped),
                "bbox_right_clipped": int(bbox_right_clipped),
                "bbox_top_clipped": int(bbox_top_clipped),
                "bbox_bottom_clipped": int(bbox_bottom_clipped),
                "bbox_clipped": int(detection_clipped),
                "pitch_measurement_rejected": int(pitch_measurement_rejected),
                "yaw_rate_raw_deg_s": raw_yaw_rate_cmd_deg_s,
                "yaw_rate_cmd_deg_s": yaw_rate_cmd_deg_s,
                "yaw_rate_source": yaw_rate_source,
                "yaw_rate_blind_base_deg_s": yaw_rate_blind_base_deg_s,
                "yaw_rate_blind_sample_count": yaw_rate_blind_sample_count,
                "yaw_rate_blind_decay": yaw_rate_blind_decay,
                "image_kf_takeover_allowed": int(image_kf_takeover_allowed),
                "image_kf_takeover_reason": image_kf_takeover_reason,
                "image_kf_valid": int(image_kf.valid),
                "image_kf_mode": image_kf.mode,
                "image_kf_theta_x": image_kf.theta_x,
                "image_kf_theta_y": image_kf.theta_y,
                "image_kf_theta_dot_x": image_kf.theta_dot_x,
                "image_kf_theta_dot_y": image_kf.theta_dot_y,
                "image_kf_age_s": image_kf.age_s,
                "image_kf_quality": image_kf.quality,
                "image_kf_reject_reason": image_kf.reject_reason,
                "image_kf_guidance_used": int(image_kf_guidance_used),
                "body_yaw_deg": body_yaw_deg,
                "cmd_yaw_deg": cmd_yaw_deg,
                "yaw_error_deg": yaw_error_deg,
                "target_bearing_deg": target_bearing_deg,
                "target_body_bearing_deg": target_body_bearing_deg,
                "los_quality": los_quality,
                "ttc_quality": ttc_quality,
                "ttc": "" if ttc_value == "" else float(ttc_value),
                "ttc_area": ttc_area,
                "ttc_area_dot": ttc_area_dot,
                "valid": int(valid),
                "guidance_mode": guidance_mode,
                "reject_reason": reason,
                "blind_push_reason": terminal_result.reason if terminal_result.using_blind_push else "",
                "terminal_state": terminal_result.state,
                "terminal_reason": terminal_result.reason,
                "terminal_arm_source": terminal_result.terminal_arm_source,
                "terminal_cutoff_source": terminal_result.terminal_cutoff_source,
                "terminal_area_ratio": terminal_result.area_ratio,
                "terminal_miss_count": terminal_result.miss_count,
                "terminal_profile": "strapdown",
                "blind_elapsed_s": terminal_result.blind_elapsed_s,
                "blind_decay": terminal_result.blind_decay,
                "blind_sample_count": terminal_result.blind_sample_count,
                "los_filter_enabled": int(args.los_filter),
                "los_filter_process_lambda": float(args.los_filter_process_lambda),
                "los_filter_process_lambda_dot": float(args.los_filter_process_lambda_dot),
                "los_filter_measurement_noise": float(args.los_filter_measurement_noise),
                "los_filter_innovation_reject": float(args.los_filter_innovation_reject),
                "los_source": los_source,
                "lambda_measured_x": 0.0 if lambda_I_measured is None else float(lambda_I_measured[0]),
                "lambda_measured_y": 0.0 if lambda_I_measured is None else float(lambda_I_measured[1]),
                "lambda_measured_z": 0.0 if lambda_I_measured is None else float(lambda_I_measured[2]),
                "lambda_raw_x": 0.0 if lambda_raw is None else float(lambda_raw[0]),
                "lambda_raw_y": 0.0 if lambda_raw is None else float(lambda_raw[1]),
                "lambda_raw_z": 0.0 if lambda_raw is None else float(lambda_raw[2]),
                "omega_raw_x": 0.0 if omega_raw is None else float(omega_raw[0]),
                "omega_raw_y": 0.0 if omega_raw is None else float(omega_raw[1]),
                "omega_raw_z": 0.0 if omega_raw is None else float(omega_raw[2]),
                "omega_raw_norm_rad_s": 0.0 if omega_raw is None else float(np.linalg.norm(omega_raw)),
                "omega_effective_norm_rad_s": 0.0 if omega_los is None else float(np.linalg.norm(omega_los)),
                "los_dt_s": los_dt_s,
                "los_angle_step_deg": los_angle_step_deg,
                "lambda_x": 0.0 if lambda_I is None else float(lambda_I[0]),
                "lambda_y": 0.0 if lambda_I is None else float(lambda_I[1]),
                "lambda_z": 0.0 if lambda_I is None else float(lambda_I[2]),
                "omega_x": 0.0 if omega_los is None else float(omega_los[0]),
                "omega_y": 0.0 if omega_los is None else float(omega_los[1]),
                "omega_z": 0.0 if omega_los is None else float(omega_los[2]),
                "g_eval_x": float(g_eval[0]),
                "g_eval_y": float(g_eval[1]),
                "g_eval_z": float(g_eval[2]),
                "a_cmd_x": float(a_cmd[0]),
                "a_cmd_y": float(a_cmd[1]),
                "a_cmd_z": float(a_cmd[2]),
                "a_cmd_norm_mps2": float(np.linalg.norm(a_cmd)),
                "n_cmd_g": float(np.linalg.norm(a_cmd) / GRAVITY_MPS2),
                "guidance_dt": guidance_dt,
                "accel_integral_dt": accel_integral_dt,
                "body_rate_control_active": int(body_rate_command_active),
                "body_rate_max_tilt_deg": float(args.body_rate_max_tilt_deg),
                "body_rate_roll_gain": float(args.body_rate_roll_gain),
                "body_rate_pitch_gain": float(args.body_rate_pitch_gain),
                "body_rate_attitude_p": float(args.body_rate_attitude_p),
                "body_rate_hover_thrust": float(args.body_rate_hover_thrust),
                "body_rate_thrust_gain": float(args.body_rate_thrust_gain),
                "body_rate_speed_hold_gain": float(args.body_rate_speed_hold_gain),
                "body_rate_speed_hold_max_accel_mps2": float(args.body_rate_speed_hold_max_accel_mps2),
                "body_rate_total_accel_limit_mps2": float(args.body_rate_total_accel_limit_mps2),
                "roll_deg": float(np.rad2deg(roll_rad)),
                "pitch_deg": float(np.rad2deg(pitch_rad)),
                "a_cmd_body_x": float(body_rate_accel_B[0]),
                "a_cmd_body_y": float(body_rate_accel_B[1]),
                "a_cmd_body_z": float(body_rate_accel_B[2]),
                "body_rate_speed_hold_accel_x": float(body_rate_speed_hold_accel[0]),
                "body_rate_speed_hold_accel_y": float(body_rate_speed_hold_accel[1]),
                "body_rate_speed_hold_accel_z": float(body_rate_speed_hold_accel[2]),
                "body_rate_control_accel_x": float(body_rate_control_accel[0]),
                "body_rate_control_accel_y": float(body_rate_control_accel[1]),
                "body_rate_control_accel_z": float(body_rate_control_accel[2]),
                "body_rate_control_accel_norm_mps2": float(np.linalg.norm(body_rate_control_accel)),
                "roll_sp_deg": float(np.rad2deg(body_rate_roll_sp_rad)),
                "pitch_sp_deg": float(np.rad2deg(body_rate_pitch_sp_rad)),
                "body_rate_p_rad_s": float(body_rate_cmd_rad_s[0]),
                "body_rate_q_rad_s": float(body_rate_cmd_rad_s[1]),
                "body_rate_r_rad_s": float(body_rate_cmd_rad_s[2]),
                "body_rate_p_deg_s": float(np.rad2deg(body_rate_cmd_rad_s[0])),
                "body_rate_q_deg_s": float(np.rad2deg(body_rate_cmd_rad_s[1])),
                "body_rate_r_deg_s": float(np.rad2deg(body_rate_cmd_rad_s[2])),
                "body_rate_thrust": float(body_rate_thrust),
                "v_cmd_pre_guard_x": float(v_cmd_pre_guard[0]),
                "v_cmd_pre_guard_y": float(v_cmd_pre_guard[1]),
                "v_cmd_pre_guard_z": float(v_cmd_pre_guard[2]),
                "v_cmd_integrated_x": float(v_cmd_integrated[0]),
                "v_cmd_integrated_y": float(v_cmd_integrated[1]),
                "v_cmd_integrated_z": float(v_cmd_integrated[2]),
                "px4_command_mode": args.px4_command_mode if args.px4_interceptor else "",
                "v_cmd_base_x": float(terminal_result.v_cmd_base[0]),
                "v_cmd_base_y": float(terminal_result.v_cmd_base[1]),
                "v_cmd_base_z": float(terminal_result.v_cmd_base[2]),
                "v_cmd_trend_bias_x": float(terminal_result.v_cmd_trend_bias[0]),
                "v_cmd_trend_bias_y": float(terminal_result.v_cmd_trend_bias[1]),
                "v_cmd_trend_bias_z": float(terminal_result.v_cmd_trend_bias[2]),
                "v_cmd_pitch_up_bias_x": float(terminal_result.v_cmd_pitch_up_bias[0]),
                "v_cmd_pitch_up_bias_y": float(terminal_result.v_cmd_pitch_up_bias[1]),
                "v_cmd_pitch_up_bias_z": float(terminal_result.v_cmd_pitch_up_bias[2]),
                "v_cmd_x": float(v_cmd[0]),
                "v_cmd_y": float(v_cmd[1]),
                "v_cmd_z": float(v_cmd[2]),
                "command_sent": int(args.enable_motion),
                "interceptor_vel_x": float(interceptor_vel[0]),
                "interceptor_vel_y": float(interceptor_vel[1]),
                "interceptor_vel_z": float(interceptor_vel[2]),
                "post_cmd_vel_x": "" if not np.isfinite(post_command_vel[0]) else float(post_command_vel[0]),
                "post_cmd_vel_y": "" if not np.isfinite(post_command_vel[1]) else float(post_command_vel[1]),
                "post_cmd_vel_z": "" if not np.isfinite(post_command_vel[2]) else float(post_command_vel[2]),
                "post_cmd_x": "" if not np.isfinite(post_command_pos[0]) else float(post_command_pos[0]),
                "post_cmd_y": "" if not np.isfinite(post_command_pos[1]) else float(post_command_pos[1]),
                "post_cmd_z": "" if not np.isfinite(post_command_pos[2]) else float(post_command_pos[2]),
                "interceptor_accel_x": float(interceptor_accel[0]),
                "interceptor_accel_y": float(interceptor_accel[1]),
                "interceptor_accel_z": float(interceptor_accel[2]),
                "interceptor_accel_norm_mps2": interceptor_accel_norm,
                "load_factor_g": load_factor_g,
                "interceptor_accel_fd_norm_mps2": accel_fd_norm,
                "load_factor_fd_g": load_factor_fd_g,
                "interceptor_x": float(interceptor_pos[0]),
                "interceptor_y": float(interceptor_pos[1]),
                "interceptor_z": float(interceptor_pos[2]),
                "camera_world_x": float(camera_world_pos[0]),
                "camera_world_y": float(camera_world_pos[1]),
                "camera_world_z": float(camera_world_pos[2]),
                "intruder_x": float(intruder_pos[0]) if args.diagnostic_truth else "",
                "intruder_y": float(intruder_pos[1]) if args.diagnostic_truth else "",
                "intruder_z": float(intruder_pos[2]) if args.diagnostic_truth else "",
            }
        )
        if args.print_every_n > 0 and (frame_id % args.print_every_n == 0 or hit):
            print(
                f"{frame_id},{sim_t:.3f},{loop_dt:.3f},{command_duration:.3f},"
                f"{detection_count},{int(detected)},{range_m:.3f},"
                f"{px_err_x:.1f},{px_err_y:.1f},{bbox_area:.2f},"
                f"{ttc_value},{valid},{guidance_mode},{reason},"
                f"{yaw_rate_cmd_deg_s:.1f},{body_yaw_deg:.1f},{cmd_yaw_deg:.1f},"
                f"{np.array2string(v_cmd, precision=2)},{int(hit)}"
            )
        if hit:
            print(
                f"hit=True collision=True reason={pair_collision_reason} "
                f"range={range_m:.3f}m t={sim_t:.3f}s"
            )
            break
        if stop_requested:
            print("preview_window_quit=True")
            break
        sleep_s = max(0.0, target_dt - frame_elapsed)
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        frame_id += 1

    _write_csv(rows, csv_path)
    print(f"strapdown_vision_csv={csv_path}")
    metadata_path = _write_run_metadata(
        args=args,
        csv_path=csv_path,
        script_name=Path(__file__).name,
        experiment_type="strapdown_vision_png",
        intrinsics=intrinsics,
        speed_cap=speed_cap,
        intruder_velocity_cmd=intruder_velocity_cmd,
        rows=rows,
        hit=hit,
    )
    print(f"strapdown_vision_meta={metadata_path}")
    if not args.no_plot and _plot_strapdown(rows, plot_path):
        print(f"strapdown_vision_plot={plot_path}")
    _summarize_run(rows)
    if not hit and rows:
        print(f"hit=False final_range={rows[-1]['range']:.3f}m")
    if display is not None:
        display["cv2"].destroyAllWindows()


if __name__ == "__main__":
    main()
