# Vendored ByteTrack core

The tracker, base-track, Kalman filter, matching, and track-list helpers are
copied from `ultralytics==8.4.71` under the upstream AGPL-3.0 license.

Integration patches remove imports of the full Ultralytics/PyTorch runtime,
provide the upstream axis-aligned IoU helper locally, and fall back to SciPy's
linear assignment when `lap` is unavailable. The ByteTrack lifecycle,
high/low-confidence association, Kalman model, and matching thresholds are
otherwise retained. Oriented boxes are intentionally unsupported because the
RKNN detector emits axis-aligned boxes only.
