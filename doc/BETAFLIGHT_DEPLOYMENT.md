# Betaflight 实机部署流程与参数确认

本文档描述当前仓库已实现的 Betaflight 实机部署路径。当前实现是
`vision_guidance -> Betaflight MSP telemetry / MSP_SET_RAW_RC` 的独立适配层，
不是 PX4 Offboard 复用路径。默认运行模式为只读日志；任何 RC 输出都必须显式使用
`--control-mode msp_raw_rc --allow-control`。

## 当前能力边界

- 已实现 Betaflight MSP v1 帧收发、常用遥测读取和 `MSP_SET_RAW_RC` 输出。
- 已实现 8 通道 RC 映射、限幅、斜率限制、watchdog、AUX gate 和安全状态机。
- 已实现 `examples/run_betaflight_log_only.py`，可记录 MSP、视觉、导引、候选 setpoint、
  输出 RC 和安全状态。
- 已实现运行 meta JSON：每次 CSV 日志旁边保存参数、配置、字段列表和 FC identity。
- 未做硬件实机验证；未实现 CRSF/SBUS/ELRS 外部注入；未自动 ARM 或切换飞行模式。
- 当前实机相机入口只是 `cv2.VideoCapture(<整数设备号>)`，没有设置或校验分辨率、FPS、
  像素格式、曝光、增益和缓冲深度，也没有 V4L2/GStreamer 硬件时间戳。
- 当前 YOLO 路径加载 Ultralytics `.pt` 模型并调用 `model.track()`；尚无 RKNN/NPU
  推理后端，`--yolo-device` 不能把 PyTorch 模型自动切换到 RK3588 NPU。
- 当前 MSP 轮询、图像采集、YOLO/ByteTrack、导引、RC 发送和 CSV 写入位于同一同步
  循环，尚未完成 RK3588 上的实时性、热稳定性和进程故障验证。

## Betaflight 专用算法优化

当前仓库没有把 PX4 的 `mavlink_body_rate`、姿态四元数或 thrust 控制链路直接移植到
Betaflight。已实现的 Betaflight 优化集中在 `vision_guidance/flight_control.py`、
`vision_guidance/betaflight_msp.py` 和 `examples/run_betaflight_log_only.py`：

```text
PureVisionGuidancePipeline
  -> GuidanceEval(g_eval)
  -> guidance_command.rate_gain_matrix
  -> GuidanceSetpoint(roll_rate, pitch_rate, yaw_rate, thrust)
  -> RcCommandMapper
  -> MSP_SET_RAW_RC
```

### 1. 导引量到 Betaflight rate setpoint

`guidance_eval_to_setpoint()` 使用 3x3 矩阵把视觉导引评估量映射为 Betaflight 可接收的
roll/pitch/yaw rate 候选量：

```text
[roll_rate_deg_s, pitch_rate_deg_s, yaw_rate_deg_s]^T
    = rate_gain_matrix * g_eval + [0, 0, yaw_rate_bias_deg_s]^T
thrust = hover_thrust
```

这里的 `g_eval` 来自 LOS/TTC 纯视觉链路，不是 PX4 的 `p/q/r`。示例配置
`config/betaflight.example.json` 中 `rate_gain_matrix` 默认全零，目的是让未标定系统
只产生日志和中性 RC，不产生实际姿态命令。上机前必须通过无桨台架确认矩阵符号、轴向
和增益。

### 2. RC 通道映射、限幅和斜率限制

`RcCommandMapper` 将 `GuidanceSetpoint` 转为 8 通道 RC microseconds：

```text
rate_norm = rate_deg_s / rate_limit_deg_s
rc_rate_us = mid_us + rate_norm * (max_us - min_us) / 2

thrust <= hover:
  throttle_us = throttle_min_us
              + (thrust - thrust_min) / (thrust_hover - thrust_min)
                * (throttle_hover_us - throttle_min_us)

thrust > hover:
  throttle_us = throttle_hover_us
              + (thrust - thrust_hover) / (thrust_max - thrust_hover)
                * (throttle_max_us - throttle_hover_us)
```

优化点：

- 支持 `AETR1234` 等通道顺序，按角色查找 roll/pitch/throttle/yaw。
- 对 rate 和 throttle 先产生 `rc_raw_ch*`，再裁剪到 `min_us/max_us`，记录
  `rc_clipped_ch*`。
- `max_delta_us_per_s` 对最终通道做斜率限制，记录 `rc_slew_limited_ch*`，避免视觉
  抖动或矩阵调参错误直接形成阶跃 RC。
- inactive 或 setpoint 无效时走中性命令，throttle 使用 `neutral_throttle_us`，
  不继续保持上一帧控制量。
- `aux_values_us` 可固定输出额外 AUX 通道值，但仍经过 RC us 裁剪。

### 3. 安全状态机和 watchdog

`BetaflightSafetyStateMachine` 只在所有 gate 满足时允许 `rc_active=1`：

```text
control_requested = (--control-mode msp_raw_rc)
allow_control     = (--allow-control)
telemetry_fresh   = telemetry_age_s <= safety.telemetry_timeout_s
attitude_synced   = attitude_age_s <= safety.attitude_timeout_s
voltage_ok        = vbat_v >= safety.min_vbat_v，或阈值为 0
watchdog_ok       = 最近有效 GuidanceEval 未超过 safety.watchdog_timeout_s
aux_enabled       = rc_in_ch[aux_enable.channel_index] 在配置区间内
target_valid      = guidance.valid
```

状态转移是单向 gate 逻辑：`LOG_ONLY`、`DISABLED`、`READY`、`DEGRADED` 和
`FAILSAFE` 都不激活 RC；只有 `ACTIVE` 会发送有效 setpoint。若 `send_neutral_when_inactive`
为真，非 active 状态仍发送中性 RC，用于让 Receiver tab 和日志确认失效策略。

### 4. MSP 遥测和姿态同步

`BetaflightMSPAdapter` 使用 MSP v1：

- 读取 `MSP_API_VERSION`、`MSP_FC_VARIANT`、`MSP_FC_VERSION` 写入 meta。
- 每轮读取 `MSP_STATUS`、`MSP_ATTITUDE`、`MSP_ANALOG`、`MSP_RC`。
- `MSP_ATTITUDE` 的 roll/pitch/yaw 转为 `R_IB`，写入 `AttitudeHistoryBuffer`，供
  `PureVisionGuidancePipeline` 按图像曝光时间查姿态。
- `MSP_SET_RAW_RC` 只在 `--control-mode msp_raw_rc --allow-control` 下使用。

因此 Betaflight 侧优化的核心不是重新设计 PNG，而是把纯视觉导引约束成“可审计、
可限幅、可随时退回中性 RC”的飞控输入。

## 日志记录代码实现

日志实现位于 `examples/run_betaflight_log_only.py`：

- `_log_fields(channel_count)` 定义 CSV 字段。
- `_log_row(...)` 将 MSP、检测、LOS/TTC、setpoint、RC 和安全 gate 展开到同一行。
- `_write_run_meta(...)` 保存运行参数、配置、字段列表和 FC identity。
- `_send_neutral_stop(...)` 在允许控制的运行结束时连续发送 5 次中性 RC。

主循环每帧执行：

```text
read_telemetry()
  -> push MSP attitude into AttitudeHistoryBuffer
read_detection()
  -> PureVisionGuidancePipeline.process()
guidance_eval_to_setpoint()
  -> SafetyStateMachine.update()
  -> RcCommandMapper.map_setpoint()
  -> maybe MSP_SET_RAW_RC
  -> writer.writerow(_log_row(...))
```

CSV 字段按用途分组如下。

|类别|字段|
|---|---|
|运行与安全|`timestamp`, `elapsed_s`, `safety_state`, `safety_reason`, `control_requested`, `allow_control`|
|gate 诊断|`telemetry_fresh`, `attitude_synced`, `watchdog_ok`, `voltage_ok`, `aux_enabled`|
|age 与错误|`telemetry_age_s`, `attitude_age_s`, `watchdog_age_s`, `telemetry_error`, `send_error`|
|MSP status|`cycle_time_us`, `i2c_error_count`, `sensor_flags`, `mode_flags`, `profile`|
|电源与姿态|`vbat_v`, `mah_drawn`, `rssi`, `amperage_a`, `roll_deg`, `pitch_deg`, `yaw_deg`|
|输入 RC|`rc_in_count`, `rc_in_ch1..rc_in_ch8`|
|检测|`detector_source`, `detector_reject_reason`, `frame_id`, `detection_exposure_ts`, `detection_score`, `track_id`|
|bbox|`bbox_x1..bbox_y2`, `bbox_width`, `bbox_height`, `bbox_area`, `bbox_area_ratio`, `bbox_clip_*`, `bbox_clipped`|
|LOS|`los_valid`, `los_reject_reason`, `los_quality`, `los_innovation_norm`, `lambda_I_*`, `lambda_dot_I_*`, `omega_los_*`|
|TTC|`ttc_valid`, `ttc_reject_reason`, `ttc_quality`, `ttc_s`, `ttc_area_filtered`, `ttc_area_dot_filtered`|
|导引|`guidance_valid`, `guidance_reject_reason`, `guidance_quality`, `g_eval_x/y/z`|
|setpoint|`sp_valid`, `sp_source`, `sp_reject_reason`, `sp_roll_rate_deg_s`, `sp_pitch_rate_deg_s`, `sp_yaw_rate_deg_s`, `sp_thrust`|
|输出 RC|`rc_active`, `rc_reason`, `rc_raw_ch*`, `rc_ch*`, `rc_clipped_ch*`, `rc_slew_limited_ch*`|

这些字段足以离线回答三类问题：视觉导引是否有效、Betaflight gate 为什么允许或拒绝控制、
实际发出的 RC 是否被限幅或斜率限制。

## RK3588 上机前仍需完成的工作

当前仓库可以作为 RK3588 无桨 `log_only` 联调起点，但还不是可直接上桨的机载程序。
下表区分已有代码和仍需实现/验证的边界。

| 工作项 | 当前状态 | 完成标准 |
|---|---|---|
| MSP 遥测与 RC 帧 | 代码已实现，未接实机 | 目标飞控连续运行 60 min，无不可解释超时、错帧或串口重连 |
| 视觉与导引 | 算法已实现 | 真实镜头完成内外参、畸变、延迟标定，日志可复现 LOS/TTC |
| RK3588 推理 | 未适配 | 确定 CPU/GPU/NPU 后端并测得持续帧率、P95/P99 延迟和温度 |
| 相机采集 | 仅 OpenCV 基础入口 | 固定采集格式和曝光，获得可信曝光/取帧时间戳并拒绝陈旧帧 |
| 控制映射 | 候选映射已实现，矩阵默认为零 | 无桨确认轴向，并按 Betaflight 实际 rate 曲线标定 RC 到角速度 |
| 人工接管 | 只有 AUX 软件 gate | 发射机接管和急停不依赖 RK3588 进程，断电/死机时飞控进入已验证状态 |
| 机载运行管理 | 未实现 | systemd、udev、日志轮转、异常重启和默认 `log_only` 均验证 |

### 1. 固化 RK3588 硬件和系统基线

- 确定板卡型号、Linux 发行版、内核、Python 版本、摄像头接口、存储介质和散热方案；
  将 `uname -a`、`python3 --version`、板卡镜像版本写入每次运行 meta。
- 使用独立、足额且滤波良好的 BEC 给 RK3588 供电，记录典型/峰值功耗和欠压行为。
  飞控 UART 必须确认 3.3 V TTL 电平和共地，不把不确定的 5 V 电源线直接接到 UART。
- 确认满载时 CPU/NPU 不热降频，相机和串口不会因供电、振动或电磁干扰掉线。至少做
  30 min YOLO 满载和 60 min MSP+相机联合老化。
- 仓库目前没有 ARM64 依赖锁定文件或 RK3588 安装脚本；需要增加可重复安装清单，锁定
  `numpy`、`opencv`、`pyserial`、检测后端和 ByteTrack 的实际版本。

首轮检查命令：

```bash
uname -m
python3 --version
ls -l /dev/video* /dev/ttyUSB* /dev/ttyACM*
python3 -m unittest discover -s tests -v
```

预期 `uname -m` 为 `aarch64`。串口用户需要具备对应设备权限；应使用 udev 创建稳定的
飞控符号链接，避免重启后 `/dev/ttyUSB0` 编号变化。当前 `--camera-device` 只接受整数，
若相机也需要稳定符号链接、CSI GStreamer pipeline 或 MIPI 专用 API，需要先扩展参数和
采集类。

### 2. 选择并实现 RK3588 推理路径

当前 `YoloByteTrackDetector` 依赖 Ultralytics/PyTorch，适合先做正确性验证，但在 RK3588
上通常只能按 ARM CPU 后端验证，不能据此认为已使用 NPU。必须在以下两条路径中明确选择：

1. **短期台架路径**：安装可用的 ARM64 PyTorch/Ultralytics，使用 `--yolo-device cpu`，
   先验证模型类别、bbox、ByteTrack ID 和算法日志；达不到实时要求时不得进入控制测试。
2. **正式机载路径**：在匹配板卡驱动版本的工具链中把模型导出并转换为 RKNN，在板端使用
   RKNN Runtime/Lite。仓库需新增与 `YoloByteTrackDetector` 等价的 RKNN detector backend。

RKNN 后端必须与 `.pt` 基线逐项对齐：letterbox 尺寸与填充值、BGR/RGB 顺序、归一化、
输出张量解释、坐标反缩放、class filter、confidence、IoU/NMS。当前 ByteTrack 由
`model.track()` 内部驱动；改为 RKNN 后，必须把 NMS 后检测框送入独立 ByteTrack 更新器，
并保持 `track_id` 连续性和现有 `FrameDetection`/日志接口。使用录制视频比较 `.pt` 与
RKNN 的逐帧框中心、面积、漏检、误检和 ID switch，不能只比较单帧图片。

性能验收至少记录：采集耗时、预处理、NPU/CPU 推理、NMS、ByteTrack、导引、MSP 请求、
RC 发送、CSV 写入、循环周期、丢帧数、CPU/NPU 温度和是否降频。根据实测 P99 延迟再确定
`--rate-hz` 和 watchdog，不能直接沿用仿真帧率或默认 20 Hz。

### 3. 完成真实相机采集和标定

- 为 USB/CSI 相机实现可配置采集后端，启动时设置并回读 width、height、FPS、FOURCC、
  exposure、gain 和 buffer size；设置失败应退出，不能只使用 JSON 中的期望值继续运行。
- 当前 JSON 的 `camera.width/height` 只用于内参和 bbox 边界，`OpenCvYoloSource` 没有把它们
  写入摄像头。必须检查实际帧尺寸与标定分辨率一致，否则 LOS 和 TTC 均不可信。
- 标定 `fx/fy/cx/cy`、畸变系数和完整 `R_BC`。现有流水线没有使用畸变系数；需要在送入
  检测/几何前去畸变，或在像素射线计算中显式反畸变。固定上视安装应通过多姿态静态目标
  验证 body/camera 轴方向，而不是只填写 `pitch_up_deg=90`。
- 锁定短曝光和增益上限，量化运动模糊、滚动快门和振动影响；记录相机掉帧、重复帧和
  自动曝光造成的检测置信度变化。

### 4. 修正时间戳和实时流水线

当前 runner 在 MSP 轮询前记录 `loop_start`，随后才调用 `capture.read()` 和 YOLO，却把
`loop_start` 写为 `detection_exposure_ts`。因此该字段目前只是近似循环时间，不是真实曝光
时间；姿态插值、LOS rate 和 TTC 都可能带入一个“MSP 轮询 + 取帧”量级的时差。

需要实施以下改造：

- 从 V4L2/相机驱动取得单调时钟域的帧时间戳；无法取得曝光时刻时，至少记录 dequeue
  时刻，并单独标明 `timestamp_source`，不能命名为真实曝光时间。
- 记录 `capture_ts`、`inference_start/end_ts`、`command_ts` 和 `send_done_ts`，计算帧龄和
  端到端命令延迟；姿态样本和图像必须落在同一个 `CLOCK_MONOTONIC` 时钟域。
- 将相机采集、推理、MSP telemetry、RC 发送和日志拆为有界队列/独立任务。图像队列采用
  latest-frame 策略，明确丢弃旧帧，禁止积压后仍输出过期导引。
- MSP v1 当前每轮串行请求 status、attitude、analog、RC，单次超时可阻塞主循环。需要给
  各消息设置独立频率，测量请求往返时间，并由一个串口所有者串行化读写，避免多线程抢占
  同一 UART。
- 进程级 watchdog 必须独立于 YOLO 主循环；推理阻塞、Python 异常或 RK3588 断电时，应由
  Betaflight 的 RX/failsafe 机制收敛到已验证状态，不能依赖 `finally` 中的 5 帧中性命令。

### 5. 补齐 Betaflight 控制和人工接管闭环

- `RcCommandMapper` 目前按 `rate/rate_limit` 线性换算 RC us，但 Betaflight 实际目标角速度
  还由 Rates Type、RC Rate、Super Rate、Expo、deadband 和飞行模式决定。需要锁定 profile，
  实测“RC us -> 期望角速度/实测角速度”，实现对应 rate 曲线反算或查表；未经标定不能把
  `sp_*_rate_deg_s` 当作飞控实际收到的角速度。
- `guidance_command.rate_gain_matrix` 当前示例为全零。先用日志回放确定轴映射和符号，再做
  小幅阶跃/扫频，标定增益、斜率限制、姿态响应延迟和饱和边界。
- 标定整机质量、电池、桨和 hover throttle。当前 setpoint 始终使用固定
  `hover_thrust`，没有高度/垂向速度闭环，不能直接承担自动起飞、降落或高度保持。
- 明确 MSP RC 的接管架构。当前程序发送完整 8 通道，`send_neutral_when_inactive=true` 的
  “中性 RC”不是人工遥控透传，可能仍影响飞控输入。必须在所用 Betaflight 版本上验证
  MSP RX/MSP Override、AUX 范围和物理接收机优先级；必要时增加外部 RC mux 或逐通道合并。
- 人工发射机的 DISARM/接管开关必须在 RK3588 卡死、串口拔出和电源掉电时仍有效。依次做
  AUX 关闭、目标丢失、相机断开、串口断开、进程 `SIGKILL`、RK3588 断电和低电压测试。
- 当前没有地理围栏、高度限制、显式 armed 状态解析、电机状态或 GPIO 急停输入；若测试
  方案要求这些 gate，需要增加代码和日志后才能扩大飞行包线。

### 6. 增加 RK3588 运行诊断日志

现有 CSV 覆盖算法、MSP、RC 和安全 gate，但不足以定位板端掉帧、调度抖动和热降频。
至少补充以下字段：

- 帧链路：`camera_seq`、`capture_ts`、`timestamp_source`、`frame_age_ms`、实际 width/height/
  FPS、丢帧/重复帧计数、队列深度。
- 分段耗时：capture、preprocess、inference、NMS、ByteTrack、LOS/TTC、MSP telemetry、
  RC send、CSV write 和总 loop 的 P50/P95/P99。
- MSP 质量：每类消息请求数、超时数、校验错误数、往返时间、最近成功时间、RC 实际发送
  周期和连续失败次数。
- 平台健康：CPU/NPU 使用率、内存、温度、频率/降频状态、供电告警和剩余磁盘空间。
- 对时证据：相机、RK3588 单调时钟、Blackbox 时间轴的偏移/同步事件。建议在每轮开始做
  可识别的静态姿态或 AUX 边沿事件，用于 CSV 与 Blackbox 离线对齐。

这些字段目前尚未由 `run_betaflight_log_only.py` 写出，属于需要新增的代码，不应在测试
报告中标记为“已记录”。高带宽原始视频建议采用独立环形文件并以 `frame_id/capture_ts`
关联 CSV，设置磁盘配额，不能让视频写盘阻塞 RC 发送。

### 7. 产品化启动和故障恢复

- 增加 `systemd` 服务，固定工作目录、Python 环境、配置路径和日志目录；默认启动必须是
  `log_only`，控制许可不能仅靠服务自动重启后永久携带 `--allow-control`。
- 增加 udev 规则固定飞控串口名称和权限；相机路径也要稳定。启动前检查模型哈希、配置
  schema、可写磁盘、相机实际格式、飞控 identity 和 Betaflight profile，不匹配则拒绝控制。
- 实现日志轮转、断电后 CSV 恢复或分段写入，并记录软件 git commit、模型 SHA256、配置
  SHA256、RKNN runtime/driver、启动原因和上次退出原因。
- 稳定性测试应覆盖重复开关机、服务崩溃重启、日志盘满、相机热插拔和串口重连。飞行时
  是否允许自动重连必须作为显式策略；默认应退出控制许可并要求人工重新使能。

### 建议实施顺序

1. **P0，只读联调**：锁定 ARM64 环境，打通稳定串口和真实相机，保持 `rate_gain_matrix`
   全零；先增加实际采集格式、真实取帧时间和分段耗时日志。
2. **P0，感知基准**：用 `.pt` 路径在录制视频上建立精度/时延基线，再实现 RKNN detector
   和独立 ByteTrack；通过同一视频逐帧回归后才切换板端默认后端。
3. **P0，运行架构**：重构 `run_betaflight_log_only.py` 的同步主循环，分离采集、推理、
   MSP/RC 和写盘，增加陈旧帧拒绝、串口统计、平台健康日志与进程 watchdog。
4. **P1，无桨控制**：锁定 Betaflight rate/PID profile，实现 rate 曲线反算或标定 LUT，
   验证八通道映射、AUX、接管、failsafe 和所有断链/断电故障注入。
5. **P1，机载固化**：增加配置校验、依赖锁定、udev、systemd、日志轮转和模型/配置哈希；
   完成 30 min 推理满载和 60 min 联合老化。
6. **P2，受限飞行**：按“无桨 -> 系留 -> 人工主控悬停 -> 非碰撞移动目标”逐级放开，
   每一级均用 CSV 与 Blackbox 对齐结果决定是否进入下一级。

### RK3588 放桨前阻断条件

以下任一项未完成时，只允许无桨 `log_only`：

- 相机实际分辨率、内外参、畸变和时间戳来源未确认。
- `.pt`/RKNN 后端的真实视频精度和持续 P99 延迟未验收。
- MSP 超时、进程崩溃、RK3588 断电后的 Betaflight failsafe 未实测。
- 人工遥控优先级、DISARM、AUX gate 和中性 RC 行为未实测。
- rate/expo/PID profile、RC 到角速度曲线、hover throttle 和命令限幅未标定。
- CSV 与 Blackbox 不能通过公共事件对齐，或缺少帧龄、RC 发送周期和热状态证据。
- `rate_gain_matrix` 仍为全零、轴向不确定，或测试中出现持续 clipping/slew limiting。

## 部署前准备

1. 在 Betaflight Configurator 中配置飞控：
   - UART 开启 MSP，并确认 baud rate。
   - Receiver、通道顺序、ARM/PREARM、MSP Override 或等效 AUX 使能逻辑。
   - Failsafe、低电压告警、Blackbox 记录。
   - Receiver tab 中确认 `roll/pitch/yaw/throttle/AUX` 通道方向正确。

2. 安装运行依赖。以下命令只适用于当前 PyTorch/Ultralytics 路径；RKNN 路径需要使用与
   板卡系统镜像匹配的 Rockchip runtime，不能通过 `--yolo-device` 替代：

   ```bash
   python3 -m pip install pyserial
   ```

   如果使用机载摄像头和 YOLO：

   ```bash
   python3 -m pip install torch ultralytics lap opencv-contrib-python
   ```

   在 RK3588 上必须先确认这些包存在兼容的 `aarch64` wheel 或可重复构建方式，再写入
   项目依赖清单。不要在外场测试前临时升级 NumPy、OpenCV、Ultralytics 或 RKNN runtime。

3. 基于 `config/betaflight.example.json` 创建本机配置，至少确认：
   - `serial.port` 和 `serial.baud`
   - `camera` 内参和固定上视外参
   - `rc_mapping.channel_map`
   - throttle 标定、最大 roll/pitch/yaw rate、RC 斜率限制
   - `safety.aux_enable`、watchdog、低电压阈值
   - `guidance_command.rate_gain_matrix`

   注意：示例配置中的 `rate_gain_matrix` 默认为全零，未标定前不会产生非中性
   roll/pitch/yaw 命令。

## 实际部署流程

### 1. 无桨只读 MSP 验证

```bash
python3 examples/run_betaflight_log_only.py \
  --config config/betaflight.example.json \
  --serial-port /dev/ttyUSB0 \
  --duration-s 60 \
  --detector-source none
```

验收标准：
- CSV 中 `telemetry_error` 为空或偶发可解释。
- `roll_deg/pitch_deg/yaw_deg` 随机体姿态变化。
- `vbat_v`、`mode_flags`、`sensor_flags`、`rc_in_ch*` 有合理值。
- `*_meta.json` 中记录到 FC variant/version/API 信息，或明确记录读取错误。

RK3588 首轮建议把配置另存为未跟踪的 `config/betaflight.rk3588.local.json`，串口使用
udev 稳定名称。此阶段保持 `rate_gain_matrix` 全零，不传 `--allow-control`。

### 2. 无桨视觉链路验证

使用摄像头和 YOLO：

```bash
python3 examples/run_betaflight_log_only.py \
  --config config/betaflight.example.json \
  --serial-port /dev/ttyUSB0 \
  --duration-s 60 \
  --detector-source yolo_bytetrack \
  --camera-device 0 \
  --yolo-model /path/to/model.pt \
  --yolo-class-id 0
```

验收标准：
- `detector_reject_reason` 可解释，目标出现时有 bbox 和 `track_id`。
- `bbox_area_ratio`、裁切标志、LOS、TTC、`g_eval_*` 字段连续合理。
- 未启用控制时 `safety_state=LOG_ONLY`，`rc_active=0`。
- 实际采集分辨率必须等于标定分辨率；在完成时间戳改造前，将
  `detection_exposure_ts` 视为近似值，不得据此宣称完成了曝光时刻姿态同步。
- 连续满载运行时无热降频、内存持续增长或帧队列积压，并保存分阶段延迟统计。

### 3. 无桨 RC 注入验证

仅在确认 Betaflight AUX/failsafe/通道方向后执行。建议先用 CSV 检测回放，避免真实视觉
抖动进入控制：

```bash
python3 examples/run_betaflight_log_only.py \
  --config config/betaflight.example.json \
  --serial-port /dev/ttyUSB0 \
  --control-mode msp_raw_rc \
  --allow-control \
  --detector-source csv \
  --detections-csv /path/to/detections.csv
```

验收标准：
- AUX 未使能时不进入 `ACTIVE`。
- AUX 使能、目标有效、遥测新鲜、watchdog 正常时才允许 `rc_active=1`。
- Betaflight Receiver tab 或 `rc_in_ch*`/`rc_ch*` 显示方向和通道顺序正确。
- `rc_raw_ch*`、`rc_clipped_ch*`、`rc_slew_limited_ch*` 可解释，无持续饱和。

### 4. 系留或低速悬停

- 人工遥控优先，伴随计算机只输出小幅辅助命令。
- 逐步扩大 `rate_gain_matrix` 和 RC 限幅，不直接沿用 AirSim/PX4 参数。
- 同步记录仓库 CSV/meta 和 Betaflight Blackbox。
- 重点检查 `telemetry_age_s`、`attitude_age_s`、`watchdog_age_s`、LOS/TTC 连续性、
  RC 饱和、姿态响应和电压电流。

### 5. 受控移动目标测试

- 先做非碰撞近距通过，不直接做撞击类测试。
- 目标距离、侧向偏置、速度和机动幅度逐步扩大。
- 任何丢目标、遥测超时、低电压、watchdog 超时或人工接管异常都停止扩大包线。

## 当前日志产物

每次运行生成：
- `logs/betaflight_log_<stamp>.csv`
- `logs/betaflight_log_<stamp>_meta.json`

CSV 已包含：
- Betaflight：姿态、电压、电流、RSSI、mode flags、sensor flags、profile、完整输入 RC。
- 视觉：bbox、面积、面积比例、裁切标志、track id、检测拒绝原因。
- 导引：LOS、LOS rate、LOS omega、TTC、`g_eval`。
- 控制：setpoint、raw RC、最终 RC、限幅标志、斜率限制标志。
- 安全：state/reason、telemetry/attitude/watchdog age、AUX、电压和控制许可 gate。

## 上机前必须确认并写入的参数

上机前分两类写入：Betaflight 飞控配置和仓库 JSON 配置。可以先保持零增益做
`log_only`，但进入 RC 注入前必须完成通道方向、AUX gate、failsafe、hover throttle、
rate limit 和 `rate_gain_matrix` 标定。

### Betaflight 飞控配置

必须在 Betaflight Configurator 或 CLI 中确认并写入：

- MSP UART：UART 编号、MSP 开关、baud rate。
- Receiver：协议、通道顺序、各通道方向、endpoint、mid。
- Modes：ARM/PREARM、人工接管 AUX、MSP Override 或等效使能 AUX。
- Failsafe：遥控丢失、MSP/RC 丢失后的动作。
- Blackbox：开启状态、记录频率、存储介质。
- Rate/PID profile：当前使用的 roll/pitch/yaw rate、expo、PID profile。
- 电池告警：cell 数、低电压阈值、严重低电压阈值。

### 仓库 JSON 配置

建议复制 `config/betaflight.example.json` 为本机文件，例如
`config/betaflight.local.json`，并至少写入：

- `serial.port`
- `serial.baud`
- `camera.width` / `camera.height`
- `camera.fx` / `camera.fy` / `camera.cx` / `camera.cy`
- `camera.pitch_up_deg` 或 `camera.R_BC`
- `rc_mapping.channel_map`
- `rc_mapping.roll_rate_limit_deg_s`
- `rc_mapping.pitch_rate_limit_deg_s`
- `rc_mapping.yaw_rate_limit_deg_s`
- `rc_mapping.thrust_min`
- `rc_mapping.thrust_hover`
- `rc_mapping.thrust_max`
- `rc_mapping.throttle_min_us`
- `rc_mapping.throttle_hover_us`
- `rc_mapping.throttle_max_us`
- `rc_mapping.neutral_throttle_us`
- `rc_mapping.max_delta_us_per_s`
- `safety.aux_enable.channel_index`
- `safety.aux_enable.min_us` / `safety.aux_enable.max_us`
- `safety.telemetry_timeout_s`
- `safety.attitude_timeout_s`
- `safety.watchdog_timeout_s`
- `safety.min_vbat_v`

### RC 输出前额外必填

只读日志阶段可以让 `guidance_command.rate_gain_matrix` 保持全零。准备执行
`--control-mode msp_raw_rc --allow-control` 前必须写入并台架确认：

- `guidance_command.rate_gain_matrix`
- `guidance_command.hover_thrust`
- roll/pitch/yaw 正负方向确认结果
- 最大允许命令幅度
- 丢目标时 throttle 策略

### YOLO 和相机运行参数

使用真实相机和 YOLO 时，运行命令中必须明确：

- `--camera-device`
- `--yolo-model`
- `--yolo-class-id`
- `--yolo-conf`
- `--yolo-iou`
- `--yolo-imgsz`
- `--yolo-device`

## 需要确定的内容

### 飞机与动力

- 机架尺寸、质量、电池、电机、桨、推重比。
- hover throttle、最小安全 throttle、最大可用 throttle。
- 最大 roll/pitch/yaw rate、Betaflight rate/expo/PID profile。
- AirMode、Angle/Acro 模式选择。
- 电压阈值、电流限制、可接受温升。

### Betaflight I/O

- 飞控型号、Betaflight 固件版本。
- MSP UART 编号、baud rate、是否与 Blackbox/telemetry 功能冲突。
- Receiver 协议、通道顺序、AUX 分配。
- ARM/PREARM/MSP Override、Failsafe、人工接管逻辑。
- Blackbox 记录频率、字段和存储介质。

### 相机与感知

- 机载计算平台、摄像头设备号、分辨率、FPS、曝光策略。
- 相机内参、畸变、固定上视外参 `R_BC` 或 `pitch_up_deg`。
- 端到端延迟、时间戳来源、安装刚性和振动隔离。
- YOLO 模型、class id、conf/iou/imgsz、有效检测距离、漏检率。
- ByteTrack 或单目标模式参数。

### 导引与控制映射

- `guidance_command.rate_gain_matrix` 的符号、轴向和增益。
- `rc_mapping.channel_map` 是否与 Betaflight Receiver tab 一致。
- roll/pitch/yaw 命令方向和最大幅度。
- throttle 标定：`thrust_min/thrust_hover/thrust_max` 到 RC us 的映射。
- RC 斜率限制、watchdog 超时、inactive 状态是否发送中性 RC。
- 丢目标时的降级策略和恢复条件。

### 测试包线与安全

- 初始距离、高度差、侧向偏置、目标速度。
- 目标 S 机动幅度/周期、风、场地边界。
- 非碰撞近距通过标准和终止条件。
- 急停、人工接管、围挡/软目标、无桨和系留验收流程。
- 每轮测试后必须检查 CSV/meta 与 Blackbox 是否一致。

## 不应直接沿用的参数

- AirSim/PX4 的 `mavlink_body_rate`、thrust、速度上限和命中判据。
- AirSim 中的视觉帧率、延迟、推力模型、碰撞判据。
- 未经台架确认的 `rate_gain_matrix`、hover throttle、最大角速度和 RC 斜率限制。
