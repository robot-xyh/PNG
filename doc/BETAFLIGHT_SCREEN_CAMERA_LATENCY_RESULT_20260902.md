# Betaflight 屏幕-相机延迟测试结果（2026-09-02）

## 测试范围

本轮使用 PC 60 Hz 屏幕进行 40 次伪随机黑白翻转，Orange Pi 5 Max 复用实机
`OpenCvCameraSource` 和 `betaflight.rk3588.kinematics_log_only.example.json` 采集相机
中央 ROI。无人机未进入控制流程，不运行 YOLO、ByteTrack、PNG 或 MSP RC 输出。

测量起点是 PC 中 `cv2.waitKey()` 完成显示事件处理的 Unix 时间，终点是 Orange Pi
中 `capture.read()` 返回后的 `camera_capture_ts`。因此结果包含屏幕刷新/合成、USB 相机、
驱动和取帧缓冲，但不是传感器真实曝光时间，也不包含浏览器预览延迟。

## 时钟与采集

| 项目 | 结果 |
|---|---:|
|Orange Pi 时钟减 PC 时钟|+69.617 ms|
|四时间戳半最小 RTT 不确定度|2.941 ms|
|最佳 20% 样本偏差跨度|1.659 ms|
|有效相机帧 / 失败帧|1905 / 0|
|相机中位帧周期 / 帧率|23.722 ms / 42.154 FPS|
|帧周期 P95 / P99 / max|24.738 / 25.168 / 28.097 ms|
|亮度 P10 / P90 / 对比度|76.091 / 148.966 / 72.875|

ROI 下半部分未被屏幕覆盖，但屏幕翻转仍产生 72.875 的稳定亮度差。共检测到 43 个
亮度边沿，其中 40 个与计划翻转一一匹配；另外 3 个来自进入/退出全屏前后的基线变化。

## 延迟结果

| 指标 | P50 | P95 | P99 | Max |
|---|---:|---:|---:|---:|
|屏幕事件到相机返回延迟（ms）|52.339|69.583|71.706|72.947|
|等效相机帧数|2.206|2.933|3.023|3.075|
|亮到暗延迟（ms）|54.439|69.925|72.342|72.947|
|暗到亮延迟（ms）|49.717|60.960|63.158|63.708|

40/40 匹配、相机失败帧为 0、两端时钟漂移均小于 0.002 ms，自动质量门通过。亮到暗
比暗到亮中位值慢约 4.72 ms，但该差值小于本轮 43.331 ms 的名义时间分辨率，不能据此
断言相机存在确定的亮暗非对称响应。

## 结论与使用边界

当前生产 OpenCV 路径的前端延迟点估计为 P50 52.34 ms、P95 69.58 ms、P99 71.71 ms。
工程预算可暂用 73 ms 作为本轮观测上界，但控制参数更新前还应把该值与 RKNN/ByteTrack
结果年龄联合测量，避免把两个不同时间起点的统计量直接相加或重复计入相机等待时间。

60 Hz 屏幕刷新、23.72 ms 相机帧周期和 2.94 ms 时钟不确定度共同限制了测量精度。
若需小于 10 ms 的曝光时刻标定，仍需硬件 LED/GPIO、光电传感器或 V4L2 驱动曝光时间戳。

## 证据索引

- `logs/camera_latency/test_20260902T075959Z/clock_probe.json`
- `logs/camera_latency/test_20260902T075959Z/display/display_transitions.csv`
- `logs/camera_latency/test_20260902T075959Z/camera/camera_brightness.csv`
- `logs/camera_latency/test_20260902T075959Z/camera/camera_metadata.json`
- `logs/camera_latency/test_20260902T075959Z/camera/camera_roi_preview.jpg`
- `logs/camera_latency/test_20260902T075959Z/analysis/screen_camera_latency_summary.json`
- `logs/camera_latency/test_20260902T075959Z/analysis/screen_camera_latency_matches.csv`
