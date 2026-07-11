# RK3588 Native Recognition Bridge

This bridge reuses the modified `circle_pilot` RKNN recognition path from the
Orange Pi. It is deliberately separate from Betaflight MSP and cannot send RC
commands.

Build on the RK3588 board:

```bash
cmake -S native/rknn_detector -B native/rknn_detector/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build native/rknn_detector/build -j2
```

The build requires `/usr/local/include/rknn_api.h` and `librknnrt.so`. The
Python logger loads `build/librknn_detector_bridge.so` through `ctypes`. ABI v2
keeps the selected-box call and adds a bounded batch call that returns every
post-NMS candidate required by ByteTrack.

The model receives packed RGB, applies pad value 114 letterboxing, and uses the
vendor multi-head DFL decoder. Its output must not be decoded with standard
Ultralytics assumptions. The bridge also preserves score/area/aspect filters
and temporal target gating from `circle_pilot`.
