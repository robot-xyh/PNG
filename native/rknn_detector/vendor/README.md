# Imported circle_pilot recognition sources

These files are copied from `/home/orangepi/src/circle_pilot` on the RK3588
test board. They preserve the modified model's RKNN tensor handling, multi-head
DFL decoding, letterbox reversal, NMS, and temporal target filtering.

Do not replace the postprocessor with standard Ultralytics YOLO decoding. The
deployed `drone_v8n_v21_kd_relu_lambda008_640_640-rk3588.rknn` model has a
modified KD/ReLU architecture and emits quantized multi-head tensors.

Imported on 2026-07-11 from the board-side working tree. Local integration
changes belong in `native/rknn_detector/src/`, not in these vendor files.
