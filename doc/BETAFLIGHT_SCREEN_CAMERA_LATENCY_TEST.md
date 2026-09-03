# Betaflight 屏幕-相机延迟测试

## 目的与边界

该流程不使用 GPIO 或 LED。PC 屏幕按伪随机间隔进行黑白翻转，RK3588 使用实机
`OpenCvCameraSource` 和当前生产相机配置采集画面，离线匹配显示翻转与 ROI 亮度边沿。
结果表示“PC 显示事件处理完成到 `capture.read()` 返回”的延迟，并包含屏幕刷新、
合成器、USB 相机及驱动缓冲。它不是精确曝光时间，也不包含 YOLO、ByteTrack、PNG、
MSP 或电机响应；浏览器 MJPEG 预览不参与测量。

## 准备

1. 拆桨、保持 DISARM；本测试不需要功率电池、GPS、遥控接管或飞控串口。
2. 让上视相机对准屏幕中央，屏幕色块应覆盖相机中央 ROI。可先用
   `--no-fullscreen` 检查摆放。
3. PC 和 Orange Pi 接入同一局域网。可用下面的四时间戳探针直接测量两机相对时钟
   偏差，不要求两端安装同一种 NTP 客户端。
4. 停止飞行 runner、相机预览和其他占用相机的进程。

## 采集步骤

先在 Orange Pi 的终端 A 启动临时时钟服务：

```bash
cd /home/orangepi/png_betaflight_python
python3 tools/probe_remote_clock_offset.py serve \
  --bind 0.0.0.0 --port 8099 --duration-s 90
```

在 PC 上立即采集 50 个四时间戳样本；JSON 已按分析器所需符号输出“Orange Pi 减
PC”的时钟偏差：

```bash
python3 tools/probe_remote_clock_offset.py probe \
  --host 192.168.124.42 --port 8099 --samples 50 \
  --output logs/camera_latency/clock_probe.json
```

然后在 Orange Pi 的终端 B 启动 45 秒相机采集：

```bash
cd /home/orangepi/png_betaflight_python
python3 tools/capture_camera_flash_probe.py \
  --config config/betaflight.rk3588.kinematics_log_only.example.json \
  --duration-s 45
```

看到 `frames=` 持续增长后，在 PC 仓库目录启动 40 次翻转：

```bash
python3 tools/run_screen_flash_probe.py \
  --transitions 40 --display-refresh-hz 60
```

运行完成后检查 Orange Pi 输出的 `camera_roi_preview.jpg`：绿色框必须完全位于屏幕
翻转区域内。然后将板端整个 `camera_<UTC>` 目录复制到 PC，例如：

```bash
scp -r orangepi@192.168.124.42:/home/orangepi/png_betaflight_python/logs/camera_latency/camera_<UTC> \
  logs/camera_latency/
```

## 离线分析

优先直接使用时钟探针 JSON：

```bash
python3 tools/analyze_camera_flash_latency.py \
  --display-csv logs/camera_latency/display_<UTC>/display_transitions.csv \
  --camera-csv logs/camera_latency/camera_<UTC>/camera_brightness.csv \
  --clock-probe-json logs/camera_latency/clock_probe.json
```

若没有运行探针，也可在两机已经由同一可靠时间源同步后手工填写
`--camera-minus-display-clock-ms` 和 `--clock-uncertainty-ms`。不能确认时钟误差时不要把
偏差写成 0 来绕过质量门。

核心输出为 `screen_camera_latency_summary.json` 和
`screen_camera_latency_matches.csv`。合格条件为：亮度对比度不低于 20、匹配率至少
99%、相机失败帧为 0、时钟不确定度及最佳样本偏差跨度不超过 5 ms。四时间戳法假设
局域网上下行延迟近似对称，`clock_uncertainty_ms` 是半个最小 RTT 的乐观界限；应同时
检查 `best_20_percent_offset_span_ms`。60 Hz 屏幕单次刷新量化约 16.7 ms，报告中的
名义分辨率还会叠加相机中位帧周期和时钟不确定度；不能把小于该量级的差异解释为
相机改进。

## 失败处理

- `screen contrast is too low`：增大屏幕亮度、遮挡环境光或缩小 ROI。
- 匹配率低：确认绿色 ROI 位于色块内，重新采集；不要放宽到超过 350 ms 后强行匹配。
- 延迟整体为负：优先检查时钟偏差符号，不要直接取绝对值。
- 两机无法可靠同步：报告只能用于缓冲帧数和抖动诊断，不能作为绝对端到端延迟证据。
