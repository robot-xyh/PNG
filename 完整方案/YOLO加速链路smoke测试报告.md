# YOLO 加速链路 smoke 测试报告

日期：2026-06-27

## 测试目的

本轮测试验证新增 YOLO 提速链路是否能进入完整 PX4 SITL + AirSim Blocks 闭环：

- `yolo_bytetrack_async` 异步检测
- `--yolo-half` FP16 推理
- `--yolo-image-transport raw` AirSim raw 图像传输
- 新增检测耗时日志字段是否可用

对照基线采用历史稳定配置：

- 控制链路：`velocity_bias + velocity_yaw_rate`
- 检测：`yolo_bytetrack`
- 图像传输：`compressed`
- FP32
- 50m、20m 高度差、目标 `Quadrotor1` actor、`SHADOW_AIRSIM_DETECT=0`

## 本地离线 benchmark

使用 `tools/benchmark_yolo_detector.py`，输入为空白 640x480 图像，仅测试模型调用链路，不包含 AirSim RPC / 图像获取。

|模式|mean FPS|mean elapsed|mean inference|
|---|---:|---:|---:|
|YOLO FP32|174.74|5.91 ms|5.85 ms|
|YOLO FP16|203.79|5.37 ms|5.31 ms|

离线结果说明模型本身不是主要瓶颈；完整仿真中更主要的瓶颈在 AirSim 图像获取、RPC、PX4/仿真循环以及末端检测连续性。

## 完整仿真结果

|case|控制链路|transport|half|async|hit|min range|final range|detected|valid|avg detector FPS|capture|inference|async age|
|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|`yolo_async_half_raw_smoke_20260627_003542`|`accel_attitude / mavlink_attitude`|raw|1|1|0|35.555|56.190|176/364|265/364|26.36|40.4 ms|9.3 ms|83.8 ms|
|`yolo_async_half_raw_velocity_smoke_20260627_004018`|`velocity_bias / velocity_yaw_rate`|raw|1|1|0|38.941|138.337|166/356|233/356|27.45|41.1 ms|10.2 ms|85.8 ms|
|`yolo_sync_compressed_velocity_smoke_20260627_004201`|`velocity_bias / velocity_yaw_rate`|compressed|0|0|1|1.624|1.624|92/96|95/96|6.86|133.7 ms|18.3 ms|-|
|`yolo_sync_raw_half_velocity_smoke_20260627_004344`|`velocity_bias / velocity_yaw_rate`|raw|1|0|0|45.725|135.862|67/237|126/237|16.73|51.4 ms|13.6 ms|-|
|`yolo_sync_compressed_half_velocity_smoke_20260627_004809`|`velocity_bias / velocity_yaw_rate`|compressed|1|0|0|2.004|75.013|107/220|115/220|7.91|118.6 ms|12.4 ms|-|
|`yolo_async_compressed_fp32_velocity_smoke_20260627_005028`|`velocity_bias / velocity_yaw_rate`|compressed|0|1|0|2.274|22.101|149/224|153/224|8.19|124.1 ms|12.6 ms|136.3 ms|

## 结论

1. 新增检测链路可以运行，日志字段有效。异步模式无线程错误，`raw+half` 下检测外层 FPS 能提高到约 26-27 FPS。

2. 只有同步 `compressed + FP32` 命中。该组在 50m 工况下 `detected=92/96`、`valid=95/96`，碰撞时间 15.261s，最小距离 1.624m。

3. `raw` 是当前最大风险项。同步 `raw+half` 检测率只有 `67/237`，最小距离 45.725m 后飞离。说明 raw 图像路径虽然取图更快，但 YOLO 连续识别显著变差，需要进一步确认 AirSim raw 返回的真实通道顺序、gamma/色彩格式、BGRA/RGBA 解释和模型训练输入是否一致。

4. `half` 也会影响闭环稳定性。同步 `compressed+half` 最小距离 2.004m，但未碰撞，检测率低于 FP32 基线。当前权重/ByteTrack 在 FP16 下可能存在近距边界框或置信度细微变化，足以影响末端控制。

5. 异步 `compressed+FP32` 接近成功但未命中，最小距离 2.274m。日志显示末端连续 `no_detection/invalid`，且出现 9 帧 `async_detection_stale`。默认不复用旧异步结果避免重复 LOS 时间戳，但末端会降低视觉保持率；后续应加入专门的 sample-hold/KF 输出，而不是直接把旧 bbox 当新测量。

## 建议

- 当前闭环默认仍使用 `YOLO_IMAGE_TRANSPORT=compressed`、`YOLO_HALF=0`、`DETECTOR_SOURCE=yolo_bytetrack`。
- 异步检测可以继续保留为实验开关，但进入闭环前需要：
  - 末端允许使用 image KF / tracker prediction 输出保持量；
  - stale 阈值结合仿真 `rate_hz` 调到约 0.20-0.25s；
  - 区分“旧检测测量”和“预测/保持输出”，避免污染 LOS 差分。
- raw 传输暂不建议用于闭环。下一步应在同一帧同时保存 compressed 和 raw，分别跑 YOLO，确认通道顺序和检测框差异后再启用。
- FP16/TensorRT 应先用离线真实图像集做检测一致性评估，再进入闭环测试。

## 相关输出

- `完整方案/YOLO_sync_compressed_velocity_smoke测试报告.md`
- `完整方案/YOLO_sync_raw_half_velocity_smoke测试报告.md`
- `完整方案/YOLO_sync_compressed_half_velocity_smoke测试报告.md`
- `完整方案/YOLO_async_compressed_fp32_velocity_smoke测试报告.md`
- `完整方案/YOLO_async_half_raw_smoke测试报告.md`
- `完整方案/YOLO_async_half_raw_velocity_smoke测试报告.md`
