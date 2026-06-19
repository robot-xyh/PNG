# Pure Vision Guidance Evaluation

This workspace contains a lightweight implementation of the pure-vision path
described in `拦截方案.md`.

It intentionally stops at logged/supervised evaluation quantities. It does not
run object detection and does not send MAVLink or flight-control commands.

## Implemented

- Pixel center to camera/inertial LOS conversion.
- Attitude history buffer with timestamp lookup.
- 6D constrained LOS Kalman filter:
  `[lambda_x, lambda_y, lambda_z, lambda_dot_x, lambda_dot_y, lambda_dot_z]`.
- Scale expansion / TTC channel from bbox area.
- Track-continuity, timestamp, LOS and TTC quality gating.
- PNG-style evaluation quantity:
  `g_eval = K(TTC) * lambda_dot_I`.
- Synthetic example and unit tests.

## Detection Input Contract

The AirSim validation scripts default to the built-in AirSim detection API, but
the gimbal and strapdown paths can also use YOLOv8 + ByteTrack. Both detector
sources are converted into the same contract:

```text
FrameDetection:
  frame_id
  exposure_ts
  bbox_xyxy
  track_id
  score
```

## Run

```bash
python3 -m unittest discover -s tests -v
python3 examples/run_synthetic.py
```

## AirSim Blocks Example

The AirSim path uses the built-in detection API by default. It only converts
detected `box2D` into `FrameDetection`; it does not read the intruder's true
pose, `relative_pose`, `geo_point`, or true velocity.

The gimbal and strapdown validation scripts can replace AirSim detection with
YOLOv8 + ByteTrack:

```bash
python3 -m pip install torch ultralytics lap opencv-python

python3 examples/run_airsim_strapdown_vision_png.py \
  --enable-motion \
  --detector-source yolo_bytetrack \
  --yolo-model /path/to/uav_yolov8.pt \
  --yolo-class-id 0
```

YOLO mode is fail-fast: if the model path, dependencies, or `--yolo-class-id`
are missing, the script exits instead of silently falling back to AirSim
detection. ByteTrack `track_id` is logged and used for target continuity.

Expected AirSim setup:

- Blocks is running.
- Multi-vehicle settings define `Interceptor` and `Intruder` under `Vehicles`.
- The default scenario initializes vehicles normally, then climbs both vehicles
  to 50 m altitude before starting the intercept loop.
- The intruder mesh name matches `Intruder*`, or pass a different pattern with
  `--mesh`.

Run logging/evaluation only:

```bash
python3 examples/run_airsim_blocks.py --duration-s 30
```

Run with bounded AirSim-only motion commands:

```bash
python3 examples/run_airsim_blocks.py --enable-motion --intruder-speed 5 --speed-ratio 2
```

Trajectory debug output is written to `logs/` by default:

- `airsim_trajectory_*.csv`
- `airsim_trajectory_*.png`

`--enable-motion` is still simulation-only. It does not send MAVLink commands.
For the three PNG validation scripts, success is defined by AirSim reporting a
collision between `Interceptor` and `Intruder`; range thresholds are diagnostic
only. Each run stops immediately after the collision frame, so CSV and plot
outputs do not include post-collision samples.
The collision object name must match the other vehicle's glob pattern
(`Intruder*` or `Interceptor*` by default), so unrelated wall/ground collisions
are not counted as intercept success.

## AirSim Truth PNG Baseline

For guidance-law validation, the project also includes a separate baseline that
intentionally uses AirSim ground-truth position and velocity for both vehicles.
This is only an upper-bound simulation check; it is not a deployable perception
path.

```bash
python3 examples/run_airsim_truth_png.py --enable-motion --intruder-speed 5 --speed-ratio 2
```

Default truth-PNG settings:

- Classic 3D PNG with `N=3.0`.
- Both vehicles climb to 50 m before the intercept loop.
- Intruder velocity defaults to `+Y` at `5 m/s`.
- Interceptor velocity command is capped at `speed_ratio * intruder_speed`.
- Hit is based on AirSim pair collision, not range radius.

Debug output is written to `logs/`:

- `truth_png_trajectory_*.csv`
- `truth_png_trajectory_*_meta.json`
- `truth_png_trajectory_*.png`

Use this baseline to separate guidance/control issues from pure-vision sensing
issues. If truth PNG misses, tune the scenario, velocity cap, acceleration cap,
or control mapping before debugging the visual LOS/TTC path.

## AirSim Gimbal Vision PNG Validation

The gimbal-camera validation script is a separate experiment from the final
delivery path. It uses AirSim built-in detection boxes and keeps the intruder
near the image center by commanding the interceptor camera pose. Guidance input
still comes only from `box2D`, bbox area, track continuity, interceptor attitude,
and camera gimbal angles.
The interceptor camera is mounted 0.5 m above the body center by default
(`--camera-z -0.5` in AirSim NED coordinates) to reduce frame occlusion.
Interceptor motion commands explicitly use `ForwardOnly` with an absolute yaw
angle derived from the current velocity command, so the airframe heading follows
the guidance direction instead of holding yaw rate zero. Near terminal range,
large/clipped boxes, gimbal-limit conditions, or short terminal target loss
trigger a short terminal extrapolation blind push. The blind command uses a
short-window average of recent 3D velocity commands plus bounded LOS-trend and
pitch-up bias; pass `--no-terminal-extrapolation` to disable it. The gimbal
adapter is intentionally different from the fixed-camera adapter: terminal
visual tracking reduces gimbal centering gain, clipped boxes stop updating the
gimbal, and blind push can freeze the last reliable gimbal yaw/pitch
(`--terminal-freeze-gimbal-on-blind`). A terminal image-plane constant-velocity
Kalman filter is enabled by default; during clipped boxes or blind push it
predicts short-horizon angular error so the gimbal can continue sweeping for up
to `--terminal-image-kf-max-predict-s` instead of only holding still.

```bash
python3 examples/run_airsim_gimbal_vision_png.py --enable-motion --intruder-speed 5 --speed-ratio 2
```

After the vehicles climb to the intercept altitude, the script may use intruder
truth once to pre-point the gimbal so the target starts inside the image. The
guidance loop does not use intruder truth. Diagnostic truth logging is enabled
by default for offline range, hit and trajectory evaluation; use
`--no-diagnostic-truth` to disable it. The OpenCV preview window is disabled by
default to keep the AirSim loop rate higher; pass `--show-window` only when
visual inspection is needed.

It writes `gimbal_vision_png_*.csv`, `gimbal_vision_png_*_meta.json`, and
`gimbal_vision_png_*.png` to `logs/`. The metadata JSON records run arguments
and derived values such as `speed_ratio`, `intruder_altitude_offset_m`, and
`speed_cap`. Ground-truth positions in those logs are not used by the visual
guidance calculation.

## AirSim Strapdown Vision PNG Validation

The strapdown-camera validation script tests the fixed forward camera path. It
does not command a gimbal. After climbing to the start altitudes, it may use
truth once to rotate the interceptor body yaw so the intruder starts inside the
camera FOV. The guidance loop then uses only AirSim `box2D`, interceptor
attitude, fixed camera extrinsics, 6D LOS and TTC.
By default the intruder starts 30 m above the interceptor before interception.

```bash
python3 examples/run_airsim_strapdown_vision_png.py --enable-motion --intruder-speed 5 --speed-ratio 2
```

Yaw is controlled from bbox horizontal error with a bounded yaw-rate command.
Velocity guidance remains 3D and preserves `v_cmd_z`, including during coast and
terminal extrapolation blind push. Because the fixed camera must keep the
airframe pointed at the target, strapdown blind push also keeps a decaying
short-window average of recent yaw-rate commands. When the terminal image-plane
Kalman filter is valid, its predicted angular error and angular rate override
the window hold to generate yaw rate during clipped boxes or blind push; pass
`--no-terminal-image-kf` to disable that predictive layer, and pass
`--no-terminal-yaw-rate-extrapolation` to restore the older behavior where
target loss immediately returns yaw rate to the current visual command. It
writes `strapdown_vision_png_*.csv`, `strapdown_vision_png_*_meta.json`, and
`strapdown_vision_png_*.png` to `logs/`.

## Safety Boundary

`GuidanceEval.g_eval` is an evaluation/logging quantity for replay, simulation,
HIL, and supervised experiments. It is not a flight-control command.
