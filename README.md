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

The detector/tracker is intentionally skipped for now. Future YOLO/ByteTrack
integration only needs to produce:

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

## Safety Boundary

`GuidanceEval.g_eval` is an evaluation/logging quantity for replay, simulation,
HIL, and supervised experiments. It is not a flight-control command.
