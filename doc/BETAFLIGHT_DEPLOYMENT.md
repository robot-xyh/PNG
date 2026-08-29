# Betaflight 实机部署流程与参数确认

本文档描述当前仓库已实现的 Betaflight 实机部署路径。当前实现是
`vision_guidance -> Betaflight MSP telemetry / MSP_SET_RAW_RC` 的独立适配层，
不是 PX4 Offboard 复用路径。默认运行模式为只读日志；任何 RC 输出都必须显式使用
`--control-mode msp_raw_rc --allow-control`。

完整飞行前需要专家确认的控制入口、RC/Rate/油门、供电、视觉、日志和分阶段放行标准见
`doc/BETAFLIGHT_FULL_FLIGHT_EXPERT_CHECKLIST.md`；不得以 bench 程序能运行替代飞行验收。

## 当前能力边界

- 已实现 Betaflight MSP v1 帧收发、常用遥测读取和 `MSP_SET_RAW_RC` 输出。
- 已实现单 UART 异步 MSP pipeline：SET_RAW_RC 不等待同步 ACK，响应由持久缓冲解析，SET 写入
  始终先于每周期最多一个遥测请求；同步接口仅保留给启动 identity/BOXID 探测和离线工具。
- 已实现 8 通道 RC 映射、限幅、斜率限制、watchdog、AUX gate 和安全状态机。
- 已实现 `examples/run_betaflight_log_only.py`，可记录 MSP、视觉、导引、候选 setpoint、
  输出 RC 和安全状态。
- 已新增 `config/betaflight.rk3588.noprop.example.json` 无桨受限配置：程序在 RC7 切入前
  预填物理 RC，拒绝 885 us 初始值，roll/pitch 限制为 3 deg/s、yaw 为 0、throttle 限制为
  1000--1100 us。该配置只用于拆桨台架，不是带桨飞行参数。
- 已实现运行 meta JSON：每次 CSV 日志旁边保存参数、配置、字段列表和 FC identity。
- 已在 Orange Pi 5 Max + Betaflight 上完成无功率电池、无桨的 MSP、USB 相机和 RKNN NPU
  `log_only` 联调；未实现 CRSF/SBUS/ELRS 外部注入，未自动 ARM 或切换飞行模式。
- 当前实机相机入口支持整数索引或设备路径，可设置并回读分辨率、FPS、FOURCC 和缓冲深度，
  并可缩放/去畸变到标定分辨率；尚未设置曝光/增益，也没有 V4L2 硬件曝光时间戳。
- 已新增 `rknn_native` 后端，复用板端 `src/circle_pilot` 的修改模型、RKNN 引擎、多头
  DFL 后处理和时序目标门控；原 `yolo_bytetrack` 仍加载 Ultralytics `.pt` 并调用
  `model.track()`，`--yolo-device` 不能把 PyTorch 模型自动切换到 NPU。
- 已新增 `rknn_bytetrack`：C ABI v2 返回全部 NMS 候选，Python 运行固定版本完整 ByteTrack
  和单目标锁定，不加载 PyTorch。该路径使用独立 latest-frame worker；历史同步 MSP 实测在
  15 Hz 隔离感知下曾出现 65.964/68.568 ms 单次发送间隙。异步 MSP、RKNN 进程隔离和 CPU
  分区已部署；180 s DISARM 满载基线最大间隔为 32.389 ms，但最近一次 ARM/RC7 主动接管仍出现
  81.534 ms 单次间隔，因此实时性缺口尚未关闭。
- 当前固件支持 `MSP_MOTOR (104)` 四电机输出读取。无桨 profile 新增锁存式电机联锁：ARM 后
  任一输出高于 1200 us、四电机极差高于 150 us、数据缺失或超过 0.75 s 时禁止算法输出并退回
  锁存人工 RC；任一联锁故障保持到 DISARM。该保护只限制拆桨台架，不能替代飞控 failsafe。

### 从 `src` 复用的修改模型识别链路

板端已有模型
`drone_v8n_v21_kd_relu_lambda008_640_640-rk3588.rknn` 已复制到 `models/rknn/`，其
SHA256 为 `ad905c19e3e2b5386fa1a5d562285a02d5e5a75ad02d89ce2f1d344810c60f59`。
`native/rknn_detector/vendor/circle_pilot/` 保留 `src/circle_pilot` 的 RKNN engine、
letterbox、DFL/NMS 和 detection filter 源码，集成代码只放在 bridge 层，避免悄然改写模型
契约。该模型不能按标准 Ultralytics 单输出解码。

2026-07-11 板端查询得到实际张量：输入为 640x640 RGB，输出为 4 个 NCHW 量化张量：

```text
box_8  [1, 64, 80, 80]   cls_8  [1, 1, 80, 80]
box_16 [1, 64, 40, 40]   cls_16 [1, 1, 40, 40]
```

每个 box head 的 64 通道给出四条边各 16 个 DFL bin，即 `reg_max=64/4=16`。对每条边
执行 softmax 后计算 `sum(bin * probability)`，乘网格 stride 形成 left/top/right/bottom；
类别 logit 经 sigmoid 后做 confidence gate。随后按 pad=114 的 letterbox scale/padding
反变换到 640x512 去畸变图像，裁剪到图像边界并执行 IoU NMS。最后按 score、面积、长宽比
过滤。代码支持使用 `gate_radius_px` 与 `reacquire_area_ratio` 防止远处小框夺取当前目标，
但当前 PNG 配置与 `src` 一致，默认关闭该门控。

`rknn_native` 保留上述 `src` 可选时序门控，但它不是完整 ByteTrack：当前 `track_id` 表示
单目标代次，在连续漏检超过 `track_hint_max_misses` 后重建。需要 ByteTrack 的跨遮挡 ID 和
多目标关联时，仍须把 NMS 后的全部框暴露给独立 tracker 并做录制视频回归，不能把当前代次
ID 宣称为 ByteTrack 结果。

正式跟踪路径使用 `rknn_bytetrack`。它保留低分候选做第二阶段关联，轨迹 lost 时只保持
真实 ByteTrack ID，不向 LOS/TTC 输出预测框；当前轨迹 removed 前禁止其他目标接管。

## Betaflight 专用算法优化

当前仓库没有把 PX4 的 `mavlink_body_rate`、姿态四元数或 thrust 控制链路直接移植到
Betaflight。已实现的 Betaflight 优化集中在 `vision_guidance/flight_control.py`、
`vision_guidance/betaflight_msp.py` 和 `examples/run_betaflight_log_only.py`：

```text
PureVisionGuidancePipeline
  -> GuidanceEval(g_eval_I, inertial NED)
  -> g_eval_B = R_IB^T * g_eval_I (body FRD at exposure time)
  -> guidance_command.rate_gain_matrix
  -> GuidanceSetpoint(roll_rate, pitch_rate, yaw_rate, thrust)
  -> GuidanceSetpointHold
  -> entry_handoff
  -> tilt_envelope
  -> RcCommandMapper
  -> PWM slew limit
  -> MSP_SET_RAW_RC
```

### 1. 导引量到 Betaflight rate setpoint

`guidance_eval_to_setpoint()` 先用检测曝光时刻的姿态把惯性系导引量旋转到FRD机体系，再用
3x3矩阵映射为Betaflight可接收的roll/pitch/yaw rate候选量：

```text
g_eval_B = R_IB^T * g_eval_I
[roll_rate_deg_s, pitch_rate_deg_s, yaw_rate_deg_s]^T
    = rate_gain_matrix * g_eval_B + [0, 0, yaw_rate_bias_deg_s]^T
thrust = hover_thrust
```

其中`R_IB`是曝光时刻FRD机体到NED惯性系的旋转。直接对`g_eval_I`使用固定矩阵会随yaw变化
产生串轴，schema v11会把这种缺失机体系转换的日志判为违规。这里的`g_eval`来自LOS/TTC
纯视觉链路，不是PX4的`p/q/r`。示例配置
`config/betaflight.example.json` 中 `rate_gain_matrix` 默认全零，目的是让未标定系统
只产生日志和中性 RC，不产生实际姿态命令。上机前必须通过无桨台架确认矩阵符号、轴向
和增益。

### 2. RC 通道映射、限幅和斜率限制

`RcCommandMapper` 将 `GuidanceSetpoint` 转为 8 通道 RC microseconds：

```text
Betaflight rate inverse:
  find stick x in [-1,1] such that applyBetaflightRates(x, rc_rate, super, expo)=rate_deg_s
  rc_rate_us = mid_us + x * one_side_pwm_span

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
- `MSP_RC` 回读是逻辑 R/P/Y/T；发送前按 src 相同规则重排为当前飞控
  `MSP_SET_RAW_RC` 的 A/E/T/R wire order。接收机 `map AETR1234` 与 MSP 回读逻辑顺序不是
  同一个概念。
- `rate_mapping_type=betaflight` 使用 Betaflight `applyBetaflightRates` 的数值反函数，分别
  绑定三轴 RC Rate、Super Rate 和 Expo；无桨 profile 再施加独立 3/3/0 deg/s 硬限幅。
- 对 rate 和 throttle 先产生 `rc_raw_ch*`，再裁剪到 `min_us/max_us`，记录
  `rc_clipped_ch*`。
- `max_delta_us_per_s` 对最终通道做斜率限制，记录 `rc_slew_limited_ch*`，避免视觉
  抖动或矩阵调参错误直接形成阶跃 RC。
- mapper 在 inactive 时生成中性候选；实际 MSP worker 在 OVERRIDE 前发送实时物理杆透传，
  OVERRIDE 后门禁不满足或命令过期时发送接管前锁存的人工 R/P/Y/T，避免突然切到中性值。
- `aux_values_us` 可固定输出额外 AUX 通道值，但仍经过 RC us 裁剪。

### 3. 安全状态机和 watchdog

`BetaflightSafetyStateMachine` 只在所有 gate 满足时允许 `rc_active=1`：

```text
control_requested = (--control-mode msp_raw_rc)
allow_control     = (--allow-control)
snapshot_approved = approval scope、FC identity、快照 SHA 和当前 JSON SHA 全部匹配
override_available/active = BOXIDS 含 permanent ID 50，且 RC7/AUX3 已切入
prefill_ready      = RC7 人工侧已成功发送至少 10 帧有效物理 RC
physical_rc_fresh = MSP_RC 未超过 msp_runtime.physical_rc_timeout_s
telemetry_fresh   = telemetry_age_s <= safety.telemetry_timeout_s
attitude_synced   = attitude_age_s <= safety.attitude_timeout_s
motor_output_ok   = 四路 MSP_MOTOR 新鲜、有效且不超过无桨最大值/极差
voltage_ok        = vbat_v >= safety.min_vbat_v，或阈值为 0
watchdog_ok       = 最近有效 GuidanceEval 未超过 safety.watchdog_timeout_s
aux_enabled       = rc_in_ch[aux_enable.channel_index] 在配置区间内
target_valid      = hold/entry/tilt 后最终 setpoint.valid
```

只有 `ACTIVE` 允许 PNG setpoint 覆盖 R/P/Y/T。获准的无桨进程即使尚未 ACTIVE，也会以
`prefill` 或 `passthrough` 模式发送完整人工 RC 帧，这是避免 RC7 首次切入时四通道落到
`rx_min_usec=885` 的必要条件。程序在 RC7 已打开且没有有效人工锁存值时输出
`manual_rc_unavailable`/`physical_rc_invalid` 并拒绝发送，不能用软件修复错误的操作顺序。

### 3.1 roll/pitch 接管平滑与倾角包络

当前 Python 路径已迁移 `circle_ai_strike` 的两项高优先级保护，执行顺序固定为：

```text
guidance -> GuidanceSetpointHold -> entry_handoff -> tilt_envelope
         -> Betaflight Rate inverse -> PWM slew -> MSP_SET_RAW_RC
```

`entry_handoff` 仅作用于 roll/pitch。当前 Betaflight 实机路径固定
`rate_source=zero`：门禁由关闭变为打开时从零角速度起步，这与已飞行的 `circle` C++ 路径
“不提供 measured body rates”的行为一致。`gyro_max_age_s` 只保留给未来经 Blackbox 交叉验证的
rate source；当前 runner 会拒绝 `rate_source=gyro`。令
`u=clamp((t-t0)/duration_s,0,1)`、`h=u^2(3-2u)`，输出为
`rate=(1-h)*rate_start+h*rate_target`。门禁关闭、setpoint 无效或保护拒绝时立即复位；yaw 不参与
该混合，thrust 继续由 worker 既有的 0.4 s 油门交接处理，避免形成两套油门状态机。

`tilt_envelope` 对每个轴独立处理。只在 `attitude*command>0`，即命令继续增大当前倾角时，才在
`max_angle-softcap_band` 到 `max_angle` 之间线性把命令压到零；向回平方向的命令不受软限幅。
超过 `max_angle` 后，用
`w=smoothstep((abs(attitude)-max_angle)/hardcap_margin)` 混合到
`level_rate=clamp(-kp*attitude,+/-max_level_rate)`。达到 `max_angle+margin` 时输出必须与姿态反向。
启用包络但姿态缺失或非有限数时 setpoint 以 `tilt_attitude_unavailable` 失效。这里不再增加
LPF/jerk；最终抖动约束仍由现有 PWM slew 完成。

无桨 profile 当前固定启用 `duration=0.8 s`、`rate_source=zero`、roll/pitch 上限 `35 deg`、
soft band `10 deg`、hard margin `5 deg`、`kp=3`、最大回平 rate `3 deg/s`。通用及未来有桨示例
仅保存候选参数且默认关闭，不能把无桨参数直接视为飞行标定值。

### 4. MSP 遥测和姿态同步

`BetaflightMSPAdapter` 使用 MSP v1：

- `MSP_ATTITUDE` 的原始 `pitch_deg` 遵循 Betaflight 显示符号，抬机头为负；FRD/NED 的
  body-to-inertial 旋转和倾角包络统一使用 `pitch_nose_up_deg=-pitch_deg`。roll 与 yaw 保持
  MSP 原符号。CSV 的 `pitch_deg` 仍保留原始 MSP 值，`tilt_pitch_attitude_deg` 记录转换后的
  FRD 值，禁止在 PNG 输出增益处再次补偿该符号。

- 读取 `MSP_API_VERSION`、`MSP_FC_VARIANT`、`MSP_FC_VERSION` 写入 meta。
- 每轮读取 `MSP_STATUS`、`MSP_ATTITUDE`、`MSP_ANALOG`、`MSP_RC`。
- `MSP_ATTITUDE` 的 roll/pitch/yaw 转为 `R_IB`，写入 `AttitudeHistoryBuffer`，供
  `PureVisionGuidancePipeline` 按图像曝光时间查姿态。
- `MSP_SET_RAW_RC` 只在 `--control-mode msp_raw_rc --allow-control` 下使用。
- 单串口 worker 以 50 Hz 发布最新命令；接管时对物理油门到算法油门做 0.4 s 线性交接，
  staged command 超过 0.15 s 时退回锁存人工 RC，正常退出前发送 3 帧人工透传。

因此 Betaflight 侧优化的核心不是重新设计 PNG，而是把纯视觉导引约束成“可审计、
可限幅、可在门禁失败时退回锁存人工 RC”的飞控输入。

## 日志记录代码实现

日志实现位于 `examples/run_betaflight_log_only.py`：

- `_log_fields(channel_count)` 定义 CSV 字段。
- `_log_row(...)` 将 MSP、检测、LOS/TTC、setpoint、RC 和安全 gate 展开到同一行。
- `_write_run_meta(...)` 保存运行参数、配置、字段列表和 FC identity。
- `EdgeEventLogger` 生成同名前缀的 `events.jsonl`，只在 ARM、OVERRIDE、prefill、安全状态、
  目标、watchdog、RC freshness 或 MSP 错误发生边沿时写入并立即 flush。
- `PlatformHealthSampler` 在独立 1 Hz 线程读取 RK3588 温度、频率、内存、磁盘和进程 RSS；
  MSP 串口线程不读取 sysfs。
- `PythonGcPauseMonitor` 使用 `gc.callbacks` 记录回收次数、代次、最近/最大/累计暂停，用于判断
  Python GC 是否与 SET_RAW_RC 长间隔同一时刻发生。
- `BetaflightMspIoWorker` 记录预填、透传、算法、命令过期和最后实际发送通道；正常退出发送
  锁存人工 RC，而不是主动发送 ARM/AUX 或中性油门。

主循环每帧执行：

```text
50 Hz MSP worker
  -> SET_RAW_RC first
  -> at most one scheduled STATUS/RAW_IMU/ATTITUDE/RC/ANALOG request
  -> merge independent sample timestamps into telemetry snapshot
main loop
  -> push only new MSP attitude samples into AttitudeHistoryBuffer
read_detection()
  -> PureVisionGuidancePipeline.process()
guidance_eval_to_setpoint()
  -> GuidanceSetpointHold
  -> entry_handoff
  -> tilt_envelope
  -> SafetyStateMachine.update()
  -> RcCommandMapper.map_setpoint()
  -> maybe MSP_SET_RAW_RC
  -> writer.writerow(_log_row(...))
```

CSV 字段按用途分组如下。

|类别|字段|
|---|---|
|运行与安全|`timestamp`, `elapsed_s`, `safety_state`, `safety_reason`, `control_requested`, `allow_control`|
|gate 诊断|`telemetry_fresh`, `attitude_synced`, `motor_interlock_ok/reason/latched`, `watchdog_ok`, `voltage_ok`, `aux_enabled`, `msp_prefill_ready`|
|age 与错误|`telemetry_age_s`, `attitude_age_s`, `watchdog_age_s`, `telemetry_error`, `send_error`|
|MSP status|`cycle_time_us`, `i2c_error_count`, `sensor_flags`, `mode_flags`, `profile`|
|电源与姿态|`vbat_v`, `mah_drawn`, `rssi`, `amperage_a`, `roll_deg`, `pitch_deg`, `yaw_deg`|
|电机|`motor_output_ch1..ch8`, `motor_output_age_s`, `motor_interlock_output_max_us`, `motor_interlock_output_spread_us`|
|RAW IMU|`acc_raw_*`, `gyro_msp_raw_*`, `mag_raw_*`, `msp_raw_imu_age_s`；未验证单位时 `gyro_*_deg_s` 留空|
|输入 RC|`rc_in_count`, `rc_in_all`（完整MSP通道）, `rc_in_ch1..rc_in_ch8`|
|检测|`detector_source`, `detector_reject_reason`, `detector_*_count`, `frame_id`, `detection_exposure_ts`, `detection_score`, `track_id`|
|RKNN 性能|`rknn_selected_index`, `rknn_preprocess_ms`, `rknn_inference_ms`, `rknn_postprocess_ms`, `rknn_total_ms`|
|ByteTrack|`tracker_state/id/age/hits/lost_frames`, `tracker_high/low_count`, `tracker_match_iou`, `tracker_switch/fragment_count`, `tracker_update_ms`|
|感知 worker|`perception_seq`, `perception_worker_rate_hz`, `perception_result_age_ms`, `perception_queue_dropped`, `perception_worker_error`|
|bbox|`bbox_x1..bbox_y2`, `bbox_width`, `bbox_height`, `bbox_area`, `bbox_area_ratio`, `bbox_clip_*`, `bbox_clipped`|
|LOS|`los_valid`, `los_reject_reason`, `los_quality`, `los_innovation_norm`, `lambda_I_*`, `lambda_dot_I_*`, `omega_los_*`|
|TTC|`ttc_valid`, `ttc_reject_reason`, `ttc_quality`, `ttc_s`, `ttc_area_filtered`, `ttc_area_dot_filtered`|
|导引|`guidance_valid`, `guidance_reject_reason`, `guidance_quality`, `g_eval_x/y/z`|
|接管与倾角整形|`pre_shape_sp_*`, `shaping_valid/reason`, `entry_handoff_*`, `tilt_*attitude_deg`, `tilt_*softcap_factor`, `tilt_*level_weight`, `tilt_hardcap_active`|
|setpoint|`sp_valid`, `sp_source`, `sp_reject_reason`, `sp_roll_rate_deg_s`, `sp_pitch_rate_deg_s`, `sp_yaw_rate_deg_s`, `sp_thrust`|
|映射链|`map_requested_*`, `map_limited_*`, `map_*_stick`, `rc_target_ch*`, `throttle_handover_*`|
|输出 RC|`rc_active`, `rc_reason`, `rc_raw_ch*`, `rc_ch*`, `rc_sent_all`, `rc_sent_ch*`, `rc_clipped_ch*`, `rc_slew_limited_ch*`|
|MSP 命令|每个 command 的 `attempt/success/error_count`, `last/max_rtt_ms`, `last_success_age_s`, `last_error`|
|MSP 发布|`msp_publish_mode`, `msp_prefill_success_count`, `msp_passthrough_send_count`, `msp_algorithm_send_count`, `msp_stale_command_count`, `msp_send_success_*`, `msp_publish_deadline_miss_count`|
|RK3588/运行时|`host_*temp_c`, `host_*freq_mhz`, `host_mem_available_mb`, `host_disk_free_gb`, `host_process_rss_mb`, `python_gc_*pause_ms`, `loop_period_s`|

无桨运行后执行以下命令。审计从 meta 中读取实际 3/3/0 deg/s、1000--1100 us 和 50 Hz
参数，发现 885 us、RC/电机越界、电机极差超限、联锁失败时算法发送、连续发送错误或成功帧间隔
超过三个周期时返回非零退出码：

```bash
python3 tools/analyze_betaflight_noprop_log.py --csv logs/<betaflight_log.csv>
```

## RK3588 上机前仍需完成的工作

Betaflight 非视觉能力的 `src` 来源、候选参数、已知冲突和迁移状态见
`doc/BETAFLIGHT_SRC_MIGRATION_RECORD.md`。机器可读来源清单
`config/betaflight.src-reference.json` 仅用于审计，禁止作为运行配置加载。

当前仓库可以作为 RK3588 无桨 `log_only` 联调起点，但还不是可直接上桨的机载程序。
下表区分已有代码和仍需实现/验证的边界。

| 工作项 | 当前状态 | 完成标准 |
|---|---|---|
| MSP 遥测与 RC 帧 | 已完成 60 s 无桨只读实测 | 目标飞控连续运行 60 min，无不可解释超时、错帧或串口重连 |
| 视觉与导引 | 算法已实现 | 真实镜头完成内外参、畸变、延迟标定，日志可复现 LOS/TTC |
| RK3588 推理 | `src` 修改模型的 RKNN NPU backend 已接入并短测 | 完成真实目标精度回归、30 min 满载及 P95/P99/温度验收 |
| 相机采集 | USB MJPG 通路已实测 | 固定曝光/增益，获得可信曝光时间戳并拒绝陈旧帧 |
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
飞控符号链接，避免重启后 `/dev/ttyUSB0` 编号变化。`--camera-device` 支持整数索引和
`/dev/v4l/by-id/...` 路径；CSI GStreamer pipeline 或 MIPI 专用 API 仍需要新增采集后端。

### 2. RK3588 推理路径与剩余验收

当前保留两个互不混用的后端：

1. **PyTorch 基线**：`yolo_bytetrack` 用 `.pt` + Ultralytics ByteTrack，只做离线或已证明
   稳定的平台基线。当前板端 CPU 持续推理会触发整机重启，因此配置默认阻断。
2. **正式机载路径**：`rknn_bytetrack` 通过 `ctypes` 加载
   `native/rknn_detector/build/librknn_detector_bridge.so`，使用板端已有 Rockchip runtime
   和 `src` 修改模型。该原生库不链接 MSP/RC 代码，不能输出飞控命令。

RKNN 的 RGB、letterbox、修改模型输出解释、坐标反缩放、confidence、IoU/NMS 和时序筛选
已按 `src` 代码复用。剩余工作是用包含真实无人机目标的同一录制视频，对 `.pt` 与 RKNN
逐帧比较框中心、面积、漏检和误检；若要求 ByteTrack，还要比较 ID switch。单张无目标
相机帧和 10 s 联合短测只证明通路正确，不证明检测精度。

当前 RK3588 JSON 的识别阈值直接取自 `src` 的 PNG 有效配置：`conf_threshold=0.20`、
`iou_threshold=0.45`、`max_det=300`、`min_score=0.25`、`min_bbox_area=0`、
`max_bbox_aspect_ratio=3.0`、`temporal_gating_enabled=false`、
`track_hint_max_misses=30`。这些值是移植基线，不是实机精度验收结论；修改时必须保留日志和
同一视频回归证据。

性能验收至少记录：采集耗时、预处理、NPU/CPU 推理、NMS、ByteTrack、导引、MSP 请求、
RC 发送、CSV 写入、循环周期、丢帧数、CPU/NPU 温度和是否降频。根据实测 P99 延迟再确定
`--rate-hz` 和 watchdog，不能直接沿用仿真帧率或默认 20 Hz。

### 3. 完成真实相机采集和标定

- 为 USB/CSI 相机实现可配置采集后端，启动时设置并回读 width、height、FPS、FOURCC、
  exposure、gain 和 buffer size；设置失败应退出，不能只使用 JSON 中的期望值继续运行。
- 当前 JSON 使用 `camera.capture_width/height` 配置并校验采集尺寸，使用
  `camera.width/height` 作为缩放、去畸变、内参和 bbox 边界的输出尺寸。启动后仍必须检查
  日志中的输入/输出尺寸，否则 LOS 和 TTC 均不可信。
- 标定 `fx/fy/cx/cy`、畸变系数和完整 `R_BC`。OpenCV 相机路径会在检测前使用配置的
  畸变系数去畸变；固定上视安装仍应通过多姿态静态目标验证 body/camera 轴方向，而不是
  只填写 `pitch_up_deg=90`。
- 实机链路采用 OpenCV 相机坐标 `C=(x右,y下,z光轴向前)` 和 Betaflight FRD 机体坐标
  `B=(x前,y右,z下)`；上视中心光轴必须满足 `R_BC*[0,0,1]=[0,0,-1]`。`R_BC` 的三列分别
  是相机 x/y/z 轴在机体系中的方向。程序会检查矩阵有限、正交且行列式为 `+1`，反射/镜像
  图像不能用旋转矩阵补偿，必须先关闭相机镜像。
- `LOG_ONLY` 允许保留 `pitch_up_deg` 以暴露旧数据，但启动日志和 `_meta.json` 会记录
  `legacy_pitch_up_deg`、光轴误差和 `control_ready=false`。任何带 `--allow-control` 的运行以及
  无桨批准工具都要求显式 `camera.R_BC`、`extrinsic_validation.verified=true`、FRD/OpenCV
  坐标声明，并要求上视光轴误差不超过配置阈值。当前 `pitch_up_deg=90` 会把中心光轴映射到
  机体 `+X`，相对机体上方误差为 `90 deg`，不能用于 RC 输出。
- 锁定短曝光和增益上限，量化运动模糊、滚动快门和振动影响；记录相机掉帧、重复帧和
  自动曝光造成的检测置信度变化。

### 4. 修正时间戳和实时流水线

`rknn_native` 将 `capture.read()` 返回后的单调时间写入 `detection_exposure_ts`，比
`loop_start` 更接近帧到达时间，但仍不是硬件曝光时刻。`yolo_bytetrack` 仍使用循环时间。
两条路径都没有 V4L2 硬件曝光时间戳，姿态插值、LOS rate 和 TTC 仍可能带入系统性时差。

需要实施以下改造：

- 从 V4L2/相机驱动取得单调时钟域的帧时间戳；无法取得曝光时刻时，至少记录 dequeue
  时刻，并单独标明 `timestamp_source`，不能命名为真实曝光时间。
- 当前 `detection_attitude_offset_ms` 定义为“检测帧时间减去最新姿态样本时间”。负值表示
  历史姿态缓冲已经覆盖该帧，不是同步误差；是否完成正确融合应同时检查 `fusion_status`、
  `fusion_wait_ms` 和 dropped count。`capture_return_monotonic` 与真实曝光时刻之间仍有未知偏差。
- 记录 `capture_ts`、`inference_start/end_ts`、`command_ts` 和 `send_done_ts`，计算帧龄和
  端到端命令延迟；姿态样本和图像必须落在同一个 `CLOCK_MONOTONIC` 时钟域。
- 将相机采集、推理、MSP telemetry、RC 发送和日志拆为有界队列/独立任务。图像队列采用
  latest-frame 策略，明确丢弃旧帧，禁止积压后仍输出过期导引。
- MSP v1 当前每轮串行请求 status、attitude、analog、RC，单次超时可阻塞主循环。需要给
  各消息设置独立频率，测量请求往返时间，并由一个串口所有者串行化读写，避免多线程抢占
  同一 UART。
- 进程级 watchdog 必须独立于 YOLO 主循环；推理阻塞、Python异常或RK3588断电时，应由
  Betaflight的RX/failsafe机制收敛到已验证状态，不能依赖正常退出时的3帧人工透传。

### 5. 补齐 Betaflight 控制和人工接管闭环

- `RcCommandMapper` 已实现 Betaflight Rates 的 RC Rate/Super Rate/Expo 数值反算，批准
  工具会把 profile 0 的三轴 `100/0/70` 与 JSON 绑定。仍需用 Blackbox 实测“RC us ->
  期望角速度/实测角速度”，因为 deadband、rate limit、飞行模式和机体动态不在反函数内。
- `guidance_command.guidance_eval_frame`必须为`inertial_ned`，
  `rate_gain_input_frame`必须为`body_frd`。运行时缺项或写成惯性系会拒绝启动，批准工具也会拒绝。
- `guidance_command.rate_gain_matrix` 当前通用示例为全零。上视相机无桨候选使用body-Y到roll、
  body-X到pitch，但正负号仍须单轴台架确认；不能把此前的`pitch <- g_eval_z`恢复为控制配置。
  先用日志回放确定轴映射和符号，再做
  小幅阶跃/扫频，标定增益、斜率限制、姿态响应延迟和饱和边界。
- 标定整机质量、电池、桨和 hover throttle。当前 setpoint 始终使用固定
  `hover_thrust`，没有高度/垂向速度闭环，不能直接承担自动起飞、降落或高度保持。
- Python worker 已按 src 实现完整物理 RC 预填、mask内算法合并和mask外AUX保留；门禁失败
  时发送接管前锁存人工RC，不使用 `send_neutral_when_inactive` 冒充人工透传。仍须实测
  MSP断流、`SIGKILL`、UART拔出和Orange Pi掉电；这些故障无法由Python `finally` 保证。
- 人工发射机的 DISARM/接管开关必须在 RK3588 卡死、串口拔出和电源掉电时仍有效。依次做
  AUX 关闭、目标丢失、相机断开、串口断开、进程 `SIGKILL`、RK3588 断电和低电压测试。
- 当前已有基于BOX ID 0的armed解析，但没有地理围栏、高度限制、电机状态或GPIO急停输入；
  若测试方案要求这些gate，需要增加代码和日志后才能扩大飞行包线。

### 6. RK3588 运行诊断日志和剩余缺口

schema v11 已补充以下诊断：

- MSP：每命令请求/成功/错误、RTT、最后成功 age、发布 tick、发送间隔和连续失败。
- 飞控反馈：RAW_IMU 的原始三轴整数与 ACC/MAG raw，以及各消息独立时间戳。当前定制固件的
  gyro整数不能直接命名为deg/s，需与ATTITUDE差分和Blackbox交叉标定。
- 控制链：请求/限幅 rate、反算 stick、斜率限制前 PWM、油门交接 source/target/alpha。
- 平台：独立 1 Hz 缓存的 CPU 频率、SOC/NPU/最高温度、内存、RSS、负载和磁盘。
- 对时：ARM、RC7/OVERRIDE、prefill、ACTIVE、目标、watchdog 和错误边沿 JSONL。
- 坐标系：`guidance_eval_frame=inertial_ned`、`rate_gain_input_frame=body_frd`，并列记录
  `g_eval_x/y/z`和`g_eval_body_frd_x/y/z`，用于核对姿态旋转和rate矩阵输入。

仍未记录硬件曝光时间、NPU真实利用率、板级欠压/降频标志、CSV写入耗时和Blackbox自动
解析。sysfs不存在或权限不足的字段留空，不伪造数值。高带宽原始视频暂不写入；后续如需
增加，应使用独立环形文件、磁盘配额和`frame_id/capture_ts`关联，不能阻塞RC发送。

### 7. 产品化启动和故障恢复

- 增加 `systemd` 服务，固定工作目录、Python 环境、配置路径和日志目录；默认启动必须是
  `log_only`，控制许可不能仅靠服务自动重启后永久携带 `--allow-control`。
- 增加 udev 规则固定飞控串口名称和权限；相机路径也要稳定。启动前检查模型哈希、配置
  schema、可写磁盘、相机实际格式、飞控 identity 和 Betaflight profile，不匹配则拒绝控制。
- 实现日志轮转、断电后 CSV 恢复或分段写入，并记录软件 git commit、模型 SHA256、配置
  SHA256、RKNN runtime/driver、启动原因和上次退出原因。
- 稳定性测试应覆盖重复开关机、服务崩溃重启、日志盘满、相机热插拔和串口重连。飞行时
  是否允许自动重连必须作为显式策略；默认应退出控制许可并要求人工重新使能。

当前已提供 `deploy/systemd/png-betaflight-log-only.service.in` 和
`tools/install_betaflight_log_only_service.sh`。模板固定 `log_only` 且不包含
`--allow-control`；安装器只安装并强制保持 disabled/inactive，不会自动开机运行。

### 建议实施顺序

1. **P0，只读联调**：锁定 ARM64 环境，打通稳定串口和真实相机，保持 `rate_gain_matrix`
   全零；先增加实际采集格式、真实取帧时间和分段耗时日志。
2. **P0，感知基准**：已接入修改模型 RKNN detector 和完整 ByteTrack；下一步用带标注真实
   目标视频逐帧回归，搜索 high/low/new/match 阈值并冻结模型、配置哈希。
3. **P0，运行架构**：采集、RKNN 和 ByteTrack 已移入 latest-frame worker；继续分离
   MSP/RC 与写盘，增加陈旧帧硬拒绝、串口统计、平台健康日志与进程 watchdog。
4. **P1，无桨控制**：受限profile、Rate反算、通道重排和预填代码已完成；下一步在Orange Pi
   验证八通道映射、AUX、接管、failsafe 和所有断链/断电故障注入。
5. **P1，机载固化**：增加配置校验、依赖锁定、udev、systemd、日志轮转和模型/配置哈希；
   完成 30 min 推理满载和 60 min 联合老化。
6. **P2，受限飞行**：按“无桨 -> 系留 -> 人工主控悬停 -> 非碰撞移动目标”逐级放开，
   每一级均用 CSV 与 Blackbox 对齐结果决定是否进入下一级。

### RK3588 放桨前阻断条件

以下任一项未完成时，只允许无桨 `log_only` 或本文限定的 `noprop_bench`，不得装桨：

- 相机实际分辨率、内外参、畸变和时间戳来源未确认。
- `.pt`/RKNN 后端的真实视频精度和持续 P99 延迟未验收。
- MSP 超时、进程崩溃、RK3588 断电后的 Betaflight failsafe 未实测。
- Python人工遥控优先级、DISARM、AUX gate 和锁存人工RC回退行为未实测。
- rate/expo/PID profile、RC 到角速度曲线、hover throttle 和命令限幅未标定。
- CSV 与 Blackbox 不能通过公共事件对齐，或缺少帧龄、RC 发送周期和热状态证据。
- `rate_gain_matrix` 仍为全零、轴向不确定，或测试中出现持续 clipping/slew limiting。
- 15 Hz/无 MJPEG/隔离感知配置下，带真实目标的算法发送最大间隔仍出现单次 65--69 ms，尚未
  连续满足 60 ms 审计门限。
- 图像仍使用接近 `capture.read()` 返回时刻的软件时间戳；200 ms 有界延迟融合已经消除本轮
  `timestamp_after_buffer` 主导拒绝，但硬件曝光时间与 Blackbox 对时仍未完成。

## 部署前准备

1. 在 Betaflight Configurator 中配置飞控：
   - UART 开启 MSP，并确认 baud rate。
   - Receiver、通道顺序、ARM/PREARM、MSP Override 或等效 AUX 使能逻辑。
   - Failsafe、低电压告警、Blackbox 记录。
   - Receiver tab 中确认 `roll/pitch/yaw/throttle/AUX` 通道方向正确。

2. 安装 Python 基础依赖。RKNN 路径还要求板卡镜像提供匹配的 `rknn_api.h` 和
   `librknnrt.so`，不能通过 `--yolo-device` 替代：

   ```bash
   python3 -m pip install pyserial
   ```

   在 RK3588 上构建原生桥：

   ```bash
   cmake -S native/rknn_detector -B native/rknn_detector/build \
     -DCMAKE_BUILD_TYPE=Release
   cmake --build native/rknn_detector/build -j2
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

### Orange Pi 5 Max Python 台架配置

仓库提供 `config/betaflight.rk3588.example.json`，其参数来自板端现有
`src/circle_pilot` 配置：Betaflight 使用 `/dev/ttyS1@115200`，`mono_a1_011` 相机采集
1280x1024 MJPG 并缩放/去畸变到 640x512。实际部署时复制为板端 local 配置；相机重新
接入后优先使用 `/dev/v4l/by-id/...` 稳定路径，不能假定设备始终为 `/dev/video1`。

Python runner 支持三种机载相机模式：

- `--detector-source camera_only`：只读取、缩放和去畸变图像，不加载 YOLO，不产生检测或
  guidance，适合第一阶段 MSP+相机联合测试。
- `--detector-source yolo_bytetrack`：在相同采集链路上运行 `.pt` 模型和 ByteTrack。
- `--detector-source rknn_native`：运行从 `src/circle_pilot` 复用的修改模型 RKNN NPU
  后端和时序门控；不加载 PyTorch。
- `--detector-source rknn_bytetrack`：在 RKNN 全候选上运行完整 ByteTrack 和单目标锁定，
  使用独立 latest-frame worker；这是正式感知验证路径。

相机日志增加 `camera_device`、`camera_frame_ok`、`camera_capture_ts`、`camera_read_ms`、
输入/输出尺寸、请求/回读 FPS、FOURCC、失败帧累计和 `loop_period_s`。当前
`camera_capture_ts` 是 `capture.read()` 返回时的单调时钟，不等于硬件曝光时间。

RK3588 Python CPU 推理配置包含 `torch_runtime.num_threads=1` 和
`torch_runtime.disable_mkldnn=true`。板端 PyTorch 2.1.0 的默认多线程/MKLDNN 卷积路径
在实测中发生段错误；禁用后同一 `.pt` 模型可完成推理。该设置只用于当前 CPU 台架验证，
不是 RKNN/NPU 性能方案。

当前 RK3588 示例同时设置 `torch_runtime.allow_cpu_inference=false`。2026-07-11 台架中，
禁用 MKLDNN 后单张黑图推理成功，但相机联合持续推理期间 Orange Pi 发生整机重启；因此
CPU YOLO 被 fail-fast 阻断。只有在供电稳定性问题关闭并重新做满载验收后才能显式解除，
正式机载路径使用已实现的 `rknn_bytetrack`，`rknn_native` 用于单框诊断，CPU YOLO 只
保留为受控基线。

### Python 浏览器遥测

Python runner 已内置只读 HTTP 遥测，不复用 `src` 的 C++ 共享内存或 `bf_debugd`。配置
`telemetry_web.enabled=true` 后，Web 服务先于串口和相机绑定端口；绑定失败会终止启动，
不会静默改端口。运行期间 Web 线程只消费主循环发布的内存快照，不调用 MSP adapter、
安全状态机或 RC mapper。

Orange Pi 当前局域网配置为：

```json
"telemetry_web": {
  "enabled": true,
  "bind": "0.0.0.0",
  "port": 8080,
  "allowed_subnets": ["127.0.0.0/8", "192.168.124.0/24"],
  "sample_hz": 5.0,
  "history_s": 60.0,
  "stale_after_s": 1.0,
  "max_sse_clients": 4,
  "preview": {"enabled": true, "max_fps": 10.0, "jpeg_quality": 70, "max_clients": 2}
}
```

PC 访问 `http://192.168.124.42:8080/`。接口固定为 `GET /api/v1/telemetry`、
`GET /api/v1/history`、`GET /api/v1/stream`、`GET /api/v1/video/mjpeg` 和
`GET /healthz`；所有写方法返回 405，非允许网段返回 403。SSE 以 5 Hz 发布结构化遥测，
页面显示安全状态、MSP/RC、姿态、MSP gyro raw、检测/ByteTrack、LOS/TTC、PNG命令和平台状态。
MSP_RC 原始输入是 `A/E/R/T`，当前 SET_RAW_RC wire map 是 `A/E/T/R`；API 保留原始
`input_us`，并另发按 wire map 重排的 `physical_us`。页面使用后者，避免把 885 us 油门误
标成 yaw。

Web schema v3 区分 `vision.new_result` 和 `vision.display_held`。20 Hz 主循环未取到新的
感知 worker 结果时，页面保留上一份真实 track/bbox 并累计 `result_age_ms`，避免把
`perception_no_new_result` 显示成 ByteTrack 换 ID；真实 `no_detection_candidates`、
`active_track_lost` 等结果仍立即清空。该保持只属于显示层，CSV 原始 detection 和 PNG 输入
不会重复使用旧框。页面同时显示 `vision.attitude_offset_ms`，正值表示图像时间晚于姿态缓存
最新样本，持续为正且超过 ATTITUDE 周期时会触发 `timestamp_after_buffer`。页面另显示最佳
候选分数、raw/class/high/low/output 数量、hits、关联阶段、match IoU 和 selector reason。

预览使用检测线程产出的同一张 640x512 图像和 bbox。独立编码线程采用单槽覆盖队列，只有
存在 MJPEG 客户端时才编码；但 2026-08-27 实测表明共享 Python 进程在目标跟踪、热负载和
MJPEG 并发下仍会饿死 MSP 线程，不能把“编码线程独立”等同于控制时序隔离。控制测试必须使用
`--disable-web-preview`，长期方案应将 MSP/RC 移到独立进程或等效实时边界。Python runner 与
`bf_debugd` 不能同时使用 8080，也不能与 C++ flight 进程并发占用相机和串口。当前监督测试
不配置 systemd 自动启动，浏览器页面不属于飞行安全闭环。

2026-08-27 已在 `192.168.124.42` 完成 44.96 s 的无动力电池、无桨、`log_only +
rknn_bytetrack` 联调：PC 可读取健康检查、JSON 和真实 MJPEG，895 行日志中 Web/MSP 错误及
`MSP_SET_RAW_RC` 计数均为 0，审计通过。`publish_deadline_miss_count=228` 仍需在真实控制前
处理；详细指标和归档文件名见 `doc/BETAFLIGHT_SRC_MIGRATION_RECORD.md`。

### 2026-07-11 Orange Pi 5 Max 实测结果

- 部署目录：`/home/orangepi/png_betaflight_python`；初始验证阶段原有
  `/home/orangepi/src/circle_pilot` 未修改、未运行。后续仅新增独立bench配置，原源码和
  production配置保持不变。
- 2026-07-11 后续无桨动力供电bench确认相机实际视频节点为 `/dev/video0`，`/dev/video1`
  仅为UVC metadata节点。src使用独立bench配置成功完成MSP静默读取、相机和RKNN联合运行；
  全程 `send_hz=0` 且无 `MSP_SET_RAW_RC`。8080页面必须同时启动 `bf_flight_png` 与
  `bf_debugd`；bench debug改用MJPEG后，根页面、实时曲线API和预览流均从工作站侧验证
  可达。完整配置、计数和SHA256见 `doc/BETAFLIGHT_SRC_MIGRATION_RECORD.md`。
- ARM64 依赖锁定在 `requirements-rk3588-stage1.txt`；CPU YOLO 实验版本记录在
  `requirements-rk3588-yolo-bench.txt`，但受 `allow_cpu_inference=false` 阻断。
- Betaflight：`/dev/ttyS1@115200`，识别 `BTFL 25.12.2`、MSP API 1.47。
- 60 s MSP-only：300 行，遥测错误 0、发送错误 0，16 路 RC 和姿态持续可读；全程
  `LOG_ONLY`、`control_requested=0`、`allow_control=0`、`rc_active=0`。
- 相机：`DHZJ Camera: ZSKJ`，视频节点必须使用 by-id `video-index0`；`video-index1` 是
  metadata capture。实测支持并回读 1280x1024 MJPG@180 FPS。
- 60 s MSP+camera-only：300/300 帧成功、相机失败 0、MSP 错误 0；取帧耗时均值/P95/P99/
  最大为 28.9/78.1/93.0/175.9 ms，循环 P95 为 200.3 ms。
- CPU YOLO：PyTorch 默认卷积段错误；单线程并禁用 MKLDNN 后单图成功，但持续联合运行
  触发整机重启，测试停止。重启后 5 s camera-only 再验证 25/25 帧成功、MSP 错误 0。
- 修改模型 RKNN：原生桥编译和 NPU 初始化成功，真实输出为 4 个 box/class head；单帧
  通路总耗时 13.5 ms。10 s 相机+MSP+RKNN 联合 `log_only` 完成 50/50 帧，MSP/相机
  错误均为 0，全程 `LOG_ONLY`、`rc_active=0`；RKNN preprocess/inference/postprocess/
  total 均值为 0.208/6.029/0.053/6.290 ms，总耗时最大 6.692 ms。画面中无目标，故 50 帧
  均为可解释的 `rknn_no_candidates`。按 `src` PNG 有效阈值修正后又完成 5 s、25/25 帧
  复测，NPU inference 均值/最大 6.046/6.905 ms、总耗时均值/最大 6.307/7.283 ms；仍无
  真实目标，尚未构成检测精度验收。
- 使用 `src/circle_pilot/logs_ws/debug_frames/raw` 中前 20 张历史 `*_det-int.jpg` 做静态
  解码回归，9 张输出有效 bbox，其余为 5 次 `rknn_candidates_filtered` 和 6 次
  `rknn_no_candidates`。这证明修改模型的多头解码、阈值筛选和坐标反变换可运行，但样本无
  独立 ground truth，不能把 9/20 当作召回率。
- 完整 ByteTrack 的 40 帧历史序列测试接收 82 个 NMS 候选，形成两个三帧确认轨迹区间；
  lost 帧不输出 bbox。30 Hz latest-frame worker 实际约 27.45 Hz，结果帧龄均值/最大
  1.75/15.87 ms，tracker 均值/最大 0.254/0.305 ms，无 worker error 或候选截断。
- 10 s 相机+MSP+RKNN+ByteTrack worker 联合测试完成 50 行，MSP 错误 0，主循环稳定
  5 Hz，全程 `LOG_ONLY`、`rc_active=0`。尚无真实目标 ground truth 和长时间热稳定性结论。
- 最终同步后再次完成 5 s、25 行联合短测：控制请求/许可/RC 输出均为 0，MSP、发送、worker
  错误和候选截断均为 0；感知约 27.61 Hz，结果帧龄均值/最大 1.24/11.02 ms，RKNN 总耗时
  均值/最大 5.743/7.531 ms，ByteTrack 均值/最大 0.251/0.343 ms。
- 板端系统时间曾不正确且重启后回跳。2026-07-11 从同步工作站校准后，Orange Pi 又于
  22:16:54 重启，`rtc-hym8563` 回到 2021-01-01；当前系统时间不能证明 RTC 可保持。局域网
  无外部 NTP 源，`chrony` 未同步，且没有前一 boot journal 可定位重启原因。
- 非视觉能力补充后完成 60 s、300 行 `LOG_ONLY` 联合验证：RAW RC 尝试/成功计数均为 0，
  MSP、发送、相机、感知 worker 错误均为 0。只读快照确认 `BTFL 25.12.2`/API 1.47，但
  实际 BOXIDS 不含 `src` 假设的 OVERRIDE permanent ID 50，因此控制保持硬阻断。完整脱敏
  结果及 artifact SHA256 见 `config/betaflight.rk3588.validation.json`。

### 1. 无桨只读 MSP 验证

```bash
python3 examples/run_betaflight_log_only.py \
  --config config/betaflight.example.json \
  --serial-port /dev/ttyUSB0 \
  --duration-s 60 \
  --detector-source none
```

在常规日志前建议先生成独立的飞控配置快照：

```bash
python3 tools/capture_betaflight_snapshot.py \
  --config config/betaflight.rk3588.example.json \
  --duration-s 5 --rate-hz 5 \
  --cli-diff-all /path/to/betaflight_diff_all.txt \
  --cli-dump-all /path/to/betaflight_dump_all.txt
```

CLI 文件必须从 Configurator 人工导出；程序不得自动进入 CLI。schema v2 读取 MSP API、
FC identity、BOXIDS/BOXNAMES 和时钟状态，解析 Ports、Receiver、Modes、Failsafe、PID、
Rate、Blackbox、Battery，并将原始文件、`configuration_review.json`、遥测和 SHA256 保存到
`logs/betaflight_snapshots/`。`--cli-export` 仅用于兼容旧单文件命令。

2026-07-11 实机确认 BOXNAMES 使用 350 字节 MSP v1 jumbo frame。31 个名称与 ID 完整对齐；
ID 51/52/53 是 `STICK COMMANDS DISABLE`、`BEEPER MUTE`、`READY`，不是缺失的 ID 50。
后续核对 Betaflight 源码确认：只有 `msp_override_channels_mask != 0` 时才将 ID 50 加入 BOX
列表。2026-07-11 快照中的 mask=0 会隐藏该 mode，因此不能把当时缺失 ID 50 解释为固件
不支持。2026-08-27已在当前固件重新导出并采集新快照：mask=15、
`aux 2 50 2 1700 2100 0 0`；MSP同时报告 `MSP OVERRIDE` permanent ID 50，位于BOX index
28。新快照为 Orange Pi 上的
`logs/betaflight_snapshots/betaflight_snapshot_20260827_153739/manifest.json`，25次采样无MSP
错误，CLI结构、八类配置和跨导出一致性检查均通过。旧 manifest 和旧 `aux ... 42 ...`
记录不得用于当前批准。

同日完成真实 `diff all`/`dump all` 审计：八类配置与结构均完整，解析错误、重复赋值和
跨导出冲突均为 0。原始文件和最终 manifest 的 SHA256 见
`config/betaflight.rk3588.validation.json`。确认的主要参数如下：

- UART1 为 mask 1、115200；UART2 为 mask 131073、230400。RK3588 当前 115200 通路与
  UART1 配置相容，但仍须通过线束/Ports 页面确认物理 UART，不能仅由 baud 反推。
- Receiver 为 CRSF、`AETR1234`。ARM=RC5/900--1300 us，ANGLE=RC8/1700--2100 us，
  BEEPER=RC6/1300--1700 us。
- Failsafe delay=15（0.1 s 单位）、procedure=`DROP`、throttle=1000 us；主通道为 AUTO，
  AUX 为 HOLD。必须做无桨接收机断链实测，不能只依据 CLI 宣称 failsafe 已验收。
- PID profile 0 的 R/P/Y PIDF 分别为 `51/64/22/84`、`54/67/25/87`、`51/64/0/84`。
  Rate profile 0 为 Betaflight `100/0/70`，RPY rate limit 为 1998 deg/s。
- Blackbox 为 SDCARD、NORMAL、1/4；电压/电流计均为 ADC，缩放值尚未用功率电池校验。

该历史审计关闭了“缺少 CLI 配置”的 blocker，但当时没有开放控制。此后飞控已人工改为
override mask 15，MSP OVERRIDE 分配到 RC7/AUX3，RC5 仅保留低位 ARM。Python 已增加
`100/0/70` 精确曲线反算、RPYT→AETR 重排、预填门禁和无桨硬限幅；仍需用新快照生成
`noprop_bench` 批准并完成 Python 无桨实测，才可关闭本阶段 blocker。

无桨批准配置必须显式声明 `msp_runtime.override_mode_cli_id=50`。批准工具将该值与快照CLI的
AUX3/1700--2100 us范围比对，同时单独校验BOX permanent ID 50；两者任一不一致即拒绝生成
批准。当前导出文件SHA256为：

- `betaflight_diff_all_20260827.txt`：
  `f2e60f6bb7f7b2d4cc644612f9f90bdb36027909fb84f33851ea7f22ffe066cc`；
- `betaflight_dump_all_20260827.txt`：
  `30d7b1cb71f4bcf52cf4541980acad9a5e5ff7086e93a2a6938c310659c7c144`。

### 与 src 实现的差异

Orange Pi 原 `/home/orangepi/src/circle_pilot` 会锁存接管前物理 R/P/Y/T、保留未覆盖 AUX，
并用 armed、OVERRIDE、状态 watchdog 和新鲜检测联合门控；这些设计已在 Python worker 中
采用。它还显式把 MSP_RC 的 R/P/Y/T 逻辑顺序转换为 SET_RAW_RC 的 A/E/T/R，和当前
`AETR1234` 一致。

但 `src` 仍不能替代当前 Python 的授权检查：在旧 mask=0 快照找不到 ID 50 时，它会回退到
bit 27，而该 bit 是 `LAUNCH CONTROL`；它也不读取 CLI 核验 mask。src 的 Rate 只按
100/0/70 中心斜率线性近似，hover 参数存在 0.078 与 0.283 冲突。其 dry-run 默认仍回写 RC，也没有电池、
低电压、NTP/RTC 或 Blackbox 门禁。因此不得用启动 `src` 服务绕过当前 blocker。

验收标准：
- CSV 中 `telemetry_error` 为空或偶发可解释。
- `roll_deg/pitch_deg/yaw_deg` 随机体姿态变化。
- `vbat_v`、`mode_flags`、`sensor_flags`、`rc_in_ch*` 有合理值。
- `*_meta.json` 中记录到 FC variant/version/API 信息，或明确记录读取错误。
- manifest 中 `capture.error_count=0`、BOX 名称/ID 数量一致，且所有 artifact 哈希可复算。
- `clock.ntp_synchronized=true`；仅人工校准 RTC 不等于跨设备对时完成。
- `clock.rtc_matches_system_date=true`，且至少经过一次断电/重启保持验证。

RK3588 首轮建议把配置另存为未跟踪的 `config/betaflight.rk3588.local.json`，串口使用
udev 稳定名称。此阶段保持 `rate_gain_matrix` 全零，不传 `--allow-control`。

### 2. 无桨视觉链路验证

先在不安装 YOLO 的情况下验证相机和 MSP 联合通路：

```bash
python3 examples/run_betaflight_log_only.py \
  --config config/betaflight.rk3588.local.json \
  --duration-s 60 \
  --rate-hz 5 \
  --detector-source camera_only
```

此命令不得附加 `--control-mode msp_raw_rc` 或 `--allow-control`。验收
`camera_frame_ok=1`、输入 1280x1024、输出 640x512、`safety_state=LOG_ONLY` 和
`rc_active=0` 后，再进入 YOLO 测试。

使用摄像头和修改模型 RKNN（当前 RK3588 推荐路径）：

```bash
python3 examples/run_betaflight_log_only.py \
  --config config/betaflight.rk3588.example.json \
  --duration-s 60 \
  --rate-hz 5 \
  --control-mode log_only \
  --detector-source rknn_bytetrack
```

该命令不得附加 `--allow-control`。meta JSON 必须包含模型 SHA256 和 4 个真实输出张量；
CSV 必须包含 `rknn_preprocess_ms`、`rknn_inference_ms`、`rknn_postprocess_ms`、
`rknn_total_ms`、tracker 状态/耗时、perception FPS/帧龄和候选计数。

使用摄像头和 PyTorch YOLO/ByteTrack 基线：

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

该阶段允许真实 RKNN+ByteTrack+PNG 进入极小输出，但四个桨叶必须物理拆除。先复制配置，
只修改稳定串口、相机和模型路径，不提高任何限值：

```bash
cp config/betaflight.rk3588.noprop.example.json \
  config/betaflight.rk3588.noprop.local.json
```

在 Configurator 重新导出设置 mask=15 和 RC7 mode 后的 `diff all`、`dump all`。关闭
Configurator释放串口，再采集新快照；命令会打印新 `manifest.json` 路径：

```bash
python3 tools/capture_betaflight_snapshot.py \
  --config config/betaflight.rk3588.noprop.local.json \
  --duration-s 5 --rate-hz 5 \
  --cli-diff-all /path/to/betaflight_diff_all.txt \
  --cli-dump-all /path/to/betaflight_dump_all.txt
```

批准工具会核验 ID 50、mask 15、RC7/AUX3、AETR、Rate profile 0=`100/0/70`、配置哈希、
无桨限值和相机外参。示例配置故意保留 `extrinsic_validation.verified=false`，必须先根据实物
安装写入显式 `R_BC` 并完成以下方向检查，不能直接把标志改成 true：

1. 关闭任何相机水平/垂直镜像，给机头、机右和画面上方做可见标记。
2. 水平放置机体，确认画面中心光线指向机体上方；目标沿机头和机右方向移动时记录 bbox
   中心的 u/v 增减方向，据此确定 `R_BC` 第一、二列。
3. 保持目标静止，分别小幅抬机头和右滚机体；检查日志中的姿态符号、`lambda_I` 与目标运动
   方向一致。反向时修改矩阵，不能修改 PNG 增益符号掩盖外参错误。
4. 将矩阵写入配置后先运行 `LOG_ONLY`，确认 `_meta.json` 中 determinant 约为 1、
   orthonormal error 接近 0、optical-axis error 小于阈值，再标记 verified。

2026-08-28 实机轴向测试确认本机OpenCV图像`+v`对应机体`+X`、图像`+u`对应机体`+Y`，因此
板端矩阵为`[[0,1,0],[1,0,0],[0,0,-1]]`。同时确认MSP抬机头为负pitch，代码转换为FRD
`(roll,-pitch,yaw)`。修正后相同yaw的10.4 deg右滚使惯性LOS仅变化0.79 deg；抬头对照在固定
yaw重算时残差1.74 deg。板端已完成verified LOG_ONLY和新无桨批准，但RAW_IMU动态pitch rate
符号、yaw倾斜补偿精度和硬件曝光时间戳仍未关闭，因此该结果不构成有桨许可。

完成后，`<manifest>` 必须替换为快照命令实际打印的文件：

```bash
python3 tools/create_betaflight_noprop_approval.py \
  --snapshot <manifest.json> \
  --config config/betaflight.rk3588.noprop.local.json \
  --output logs/betaflight_noprop_approval.json \
  --operator orangepi \
  --acknowledge-props-removed
```

启动前保持 RC7 在人工侧、RC5 为 DISARM，并确认 Configurator、src、debugd 和其他 MSP
进程均已退出。运行真实视觉 PNG：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
python3 examples/run_betaflight_log_only.py \
  --config config/betaflight.rk3588.noprop.local.json \
  --duration-s 0 --rate-hz 20 \
  --control-mode msp_raw_rc \
  --allow-control \
  --detector-source rknn_bytetrack \
  --disable-web-preview \
  --isolate-rknn-process \
  --main-cpu-affinity 6,7 \
  --rknn-cpu-affinity 4,5
```

线程限制、进程隔离和 CPU 分区不改变批准 JSON 或控制包线，并会写入 meta。当前 RK3588 已验证
主进程/MSP 使用 CPU 6--7、相机/RKNN/ByteTrack 子进程使用 CPU 4--5；集合重叠或系统拒绝
affinity 时程序会在启动阶段失败。隔离模式要求关闭 MJPEG；根页面的 JSON/SSE 状态仍可读，但
视频流不可用，目标位置需在接管前通过 log-only 预览确认。配置要求
`msp_runtime.transport_mode=async_pipeline`，单 UART 使用非阻塞请求/响应流水线。

严格按以下顺序操作：

1. 等终端出现 `prefill=1 publish=passthrough`，且 `sent_aetr` 前四通道没有 885；未出现时
   不得 ARM 或打开 RC7。
2. RC5 低位 ARM，确认无桨电机只正常怠速；将目标放入视场并移动/靠近，使
   `target=1`。静止且面积不增长的目标可能因 TTC 无效而保持 `target=0`。
3. 最后将 RC7 拨到接管侧。`state=ACTIVE publish=algorithm` 时才是 PNG 覆盖；没有有效
   目标时应为 `DEGRADED/target_invalid` 和 `passthrough`。
4. 横向及纵向缓慢移动目标，核对实际 `sent_aetr`：A/E 只能约在 1493--1507 us，T 必须
   在 1000--1100 us，R 固定约 1500 us；同时记录机体姿态和电机响应方向。
5. 结束时先关闭 RC7 恢复人工，再 RC5 DISARM，最后按 `Ctrl-C`。任何 885、串轴、突跳、
   `manual_rc_unavailable`、持续 stale/error 都立即按该顺序退出。

CSV 的 `rc_ch*` 是 mapper 候选，`rc_sent_ch*` 才是 worker 实际写入飞控的 AETR 帧。只有
实际发送值、计数、状态变化和 Blackbox 均可解释，才算通过 Python PNG 无桨测试。

### 3.1 2026-08-27 当前无桨验收状态

最新完整测试日志为 `logs/betaflight_log_20260827_202613.csv`：动力电池、桨叶拆除、RC5 ARM、
RC7 MSP OVERRIDE 和移动目标流程均已执行。60284 次 SET_RAW_RC 全部成功，522 个主循环记录
`publish=algorithm`，实际 A/E/T/R 为 1497--1503/1498--1502/990--1078/1500 us；目标丢失、
静止无闭合率或 bbox 裁切时退回 `FAILSAFE/passthrough`。退出顺序已确认
`RC7=1000 -> RC5=2000 -> Ctrl-C`。

该轮不构成放桨许可。审计唯一违规是一次 68.568 ms 成功发送间隔，门限为 60 ms；前一轮带
目标测试也出现一次 65.964 ms。问题发生于单 UART 的同步 MSP 轮询/发送调度，不是 RC 错误或
控制包线越界。该结论是异步修正前的历史基线，不得用来评价新 pipeline。

### 3.2 单 UART 异步修正与板端基线

2026-08-27 离线完成、2026-08-28 板端部署异步 MSP 实现。每个
20 ms worker 周期严格执行：写最新 SET_RAW_RC、排入最多一个到期遥测请求、在最多 3 ms 内
排空响应；错过的发布周期直接跳过，不补发积压命令。解析器保存跨周期字节缓冲，可处理分片、
粘包和噪声重同步。每种遥测命令最多保留一个未决请求，SET ACK 则按 FIFO 关联到实际写入帧。

RC7 接管前必须收到至少 10 个已确认的人工透传 ACK。RC7 生效后使用切换前已校验的人工 RC
锁存值并暂停 RC 查询，避免 Betaflight OVERRIDE 下的 885 us 回读污染人工输入；STATUS 报告
OVERRIDE 关闭后恢复 RC 查询。最后一个 SET ACK 超过 250 ms 时，安全状态为
`msp_set_raw_rc_ack_stale`，worker 只继续人工透传，不允许 `publish=algorithm`。
当前无桨配置中 RC7 同时承担接管与 AUX 许可，因此
`safety.aux_enable.satisfied_by_override_mode=true`；若以后改用独立 AUX，必须设为 false 并恢复
该通道的持续物理 RC 证据。

板端定位发现完整统计快照位于 50 Hz 热路径会随样本数增长拖慢 worker；修复后改为轻量 ACK/
write 时间读取、增量有序分位数和固定时基调度，并保留 16 ms 最小写间隔防止追赶式突发。
纯 MSP 日志 `transport_incremental_fix_20260828_190022_20260828_190023.csv` 审计通过，平均
49.799 Hz、最大间隔28.700 ms且deadline miss为0。未做CPU分区的完整RKNN日志只有38.148 Hz、
最大159.706 ms并出现3行`physical_rc_stale`；程序均按设计退回预填充，未发送算法命令。

使用程序参数显式分区后的
`isolated_explicit_affinity_20260828_192131_20260828_192132.csv`在完整RKNN+ByteTrack负载下
审计0违规：平均49.955 Hz、最大29.578 ms、P99.9 27.035 ms、deadline miss为0，RC最大年龄
145.610 ms，发送/请求/checksum/parser错误均为0。meta记录main `[6,7]`、detector `[4,5]`。
这只关闭RC5 DISARM、RC7人工侧的满载传输基线；尚未关闭ACTIVE算法输出或故障注入验收。

继续按以下顺序验证，全部保持拆桨：

1. 先运行 `log_only`，不带 `--allow-control`，确认 Web 中 transport 为 `async_pipeline`，
   SET write/ACK 都为 0，所有遥测命令持续更新且 parser/checksum error 为 0。
2. 使用本节无桨命令在 `/dev/ttyS1@115200` 运行，保持 RC5 DISARM、RC7 人工，确认
   `prefill=1` 后再执行既有 ARM、动态目标、RC7 接管和人工退出流程。
3. 对当前 schema v9 CSV 执行审计。除写错误、885 us、门禁违规、解析/校验错误均为 0 外，
   平均 SET 写入率不低于 49 Hz，P99.9 间隔不超过 40 ms，最大间隔不超过 60 ms，ACK stall
   不超过 250 ms；整形因子必须在 `[0,1]`，硬倾角区输出必须反向回平。至少连续三轮相同流程
   通过后才进入下一比较。
4. 仅在 115200 结果可复现后，由人工同时把 Betaflight MSP UART 和 JSON `serial.baud` 改为
   230400，重复完全相同的流程；任一 identity/BOXID、解析或 ACK 指标退化即恢复 115200。
5. 若 115200/230400 均不满足门限，再评估 FC 空闲 UART4 到 Orange Pi `/dev/ttyS7` 的双串口
   方案。当前代码尚未实现双适配器，不能只接线或开启端口后直接运行。

### 3.3 接管平滑离线实现后的必做复验

本节功能最初在 Orange Pi 断电期间完成，随后已完成 schema v8--v12 板端复验；最新结果见3.4。
每次配置内容改变后，旧 `noprop_bench` approval 都会因 JSON SHA256 不匹配自动失效；不得编辑
approval JSON 绕过。重新上电后必须先采集包含 `diff all`/`dump all` 的新快照，再运行
`tools/create_betaflight_noprop_approval.py` 生成新批准文件。

无桨复验除 3.2 的串口门限外，还必须覆盖：RC7 每次切入时
`entry_handoff_source=zero`、`entry_handoff_progress` 从0连续到1、整形后R/P无阶跃；手动倾斜机体进入软区时
只衰减继续外倾命令；进入硬区时 `tilt_hardcap_active=1` 且输出反向，仍不超过 3 deg/s。页面
只用于观察，最终以 CSV、meta、events、审计 JSON 和 Blackbox 一致性为准。完成这组复验前
继续保持拆桨，不得据此进入系留或有桨测试。

### 3.4 2026-08-29 主动接管结果与当前阻断项

在拆桨、固定机体和功率电供电下，稳定单目标 LOG_ONLY 运行40 s，目标有效率98%，195个新跟踪
结果保持同一 track，未发送 `MSP_SET_RAW_RC`。随后主动接管日志
`fixed_vm_isolated_fixed_target_takeover_20260829_20260829_175710.csv`在约
109.68--175.58 s进入算法发布；roll/pitch候选仅约
`-0.018..+0.016/-0.010..+0.009 deg/s`，说明视觉命令接近零。

但该阶段电机输出出现明显分化。最初抽取窗口为M1约1281--1363、M2约1346--1456、M3约1056、
M4约1325--1431；新版全日志审计进一步得到最大输出1625 us、最大极差569 us，分别从elapsed
124.781/125.285 s首次越过1200/150 us门限。这与接近零的上位机rate命令不相称。固定无桨机体
上的Betaflight PID/I-term或mixer积累是候选原因，但没有同时间轴Blackbox的setpoint/P/I/D、
gyro和motor数据，当前不能下结论。该日志另有一次81.534 ms SET_RAW_RC写间隔，发生在
122.519 s附近；当时MJPEG和预览编码均为0、RKNN约6.27 ms，因此仅关闭网页预览不足以解释或
消除该间隔。

当前代码已把日志schema升级到13，增加Python GC暂停时间和锁存式无桨电机联锁。`noprop_bench`
控制启动必须启用四电机轮询和联锁；ARM后输出超过1200 us或极差超过150 us即停止算法授权，
数据缺失/过期也按相同方式阻断；worker继续发送接管前锁存的人工RC，故障保持到DISARM。批准
工具绑定这些参数，修改配置后必须重新生成批准文件。该改动已同步到`.42/orangepi5max`，板端
Betaflight测试105/105、控制测试24/24通过；控制配置SHA256为
`a9178d2924501fa4edf56c250d09d0e06bf0c7517b81317c857784b699b8393c`，新批准文件SHA256为
`6f209269a45593c7ed2b5073ec034a868e3e54830a406abb4c77c67d641f054b`。

schema 13实机LOG_ONLY运行59.965 s，1197行、四电机始终1000、SET_RAW_RC为0，1153个确认的新
跟踪结果均为同一track；审计0违规。随后DISARM预填运行89.979 s，4491次写入、平均49.925 Hz、
最大间隔34.820 ms、P99.9为33.849 ms，四电机仍为1000且全部MSP错误为0，审计0违规。两轮GC
最大暂停分别为2.141/2.323 ms。证据归档SHA256为
`4042da5e45e1c09613967b23fd6a66a1576526247e9ad19bca09da11bff1f669`。取得Blackbox并对齐
RC7/ARM边沿前，不重复长时固定机体主动接管，也不进入有桨、系留或自由飞行。

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
- `logs/betaflight_log_<stamp>_events.jsonl`

执行无桨审计后另生成 `logs/betaflight_log_<stamp>_audit.json`。

CSV 已包含：
- Betaflight：姿态、电压、电流、RSSI、mode flags、sensor flags、profile、完整输入 RC。
- 视觉：bbox、面积、面积比例、裁切标志、track id、检测拒绝原因。
- RKNN：模型哈希/输出 schema（meta），raw/accepted/selected 候选数、最佳候选置信度和预处理、
  NPU 推理、后处理、总耗时（CSV）。
- ByteTrack：真实轨迹 ID、状态、age/hits/lost、关联阶段/IoU、切换/碎片计数和耗时；感知
  worker 的实际 FPS、新结果标志、结果帧龄、覆盖旧结果数和异常。
- 导引：LOS、LOS rate、LOS omega、TTC、`g_eval`。
- 控制：整形前/后 setpoint、接管起点与进度、倾角 soft/hard 权重、raw RC、最终 RC、限幅标志、
  斜率限制标志。
- 安全：state/reason、telemetry/attitude/watchdog age、AUX、电压和控制许可 gate。
- MSP 运行：armed、OVERRIDE available/active、物理 RC 帧龄、快照授权、串口请求/错误/
  字节数、RAW RC 尝试/成功计数和 worker poll/stage/skip/error。
- 电机联锁：`MSP_MOTOR`输出/年龄/命令统计、四电机最大值与极差、联锁原因和锁存状态；schema
  v13审计禁止在联锁未通过时出现`publish=algorithm`。
- MSP 接管证据：output/algorithm authorization、worker观察到的override、预填是否完成及成功
  帧数、透传/算法/过期计数、staged command age、publish mode 和 `rc_sent_ch*` 实际发送值。
- 发送当刻门禁：schema v7+ 的 `msp_last_publish_output_enabled`、
  `msp_last_publish_algorithm_authorized`、`msp_last_publish_override_active`、
  `msp_last_publish_prefill_ready`、`msp_last_publish_physical_rc_fresh` 和
  `msp_last_publish_command_fresh`、`msp_last_publish_set_raw_rc_ack_fresh` 与同一实际写入帧绑定，
  不能用当前 staged gate 或同步 ACK 完成时间代替。
- MSP 时序：schema v7+ 分开记录 SET 写尝试/成功/错误和 ACK 数量/age/FIFO 深度，记录实际写
  interval、最大间隔、窗口平均频率及 P50/P95/P99/P99.9；另记录遥测未决数、RX 丢弃字节、
  checksum/parser error、RC 轮询暂停状态和各命令 RTT。
- 映射与反馈：请求/限幅后rate、Betaflight反算stick、斜率限制前PWM、油门交接参数、
  RAW_IMU原始整数、图像相对最新姿态样本的时间偏差，以及各类遥测的独立sample age。没有可信
  换算时`gyro_*_deg_s`必须为空，不能用字段名替代单位证据。
- 平台与公共事件：RK3588 温度/频率/内存/磁盘/RSS、Python GC最近/最大/累计暂停，以及可与
  Blackbox ARM/RC7 边沿对齐的JSONL。RAW IMU本阶段只记录，不反馈到PNG；相机时间戳仍不是
  硬件曝光时间。
- Web 遥测：服务状态、SSE/MJPEG 客户端数、快照发布数、预览投递/编码/丢弃数、HTTP请求/
  子网拒绝数、错误数和最后错误。Web schema v10直接显示SET ACK gate、transport、写入频率、
  最大/P99.9 gap、ACK age/pending、parser/CRC、电机联锁和GC暂停；旧日志仍可读取，但只有日志
  schema v13能证明本轮电机联锁门禁。

`src` 参考的单串口 MSP worker 已在 Python 中实现，但 RK3588 示例默认关闭。任何未来 RC
输出都必须使用 worker；同步日志路径不再允许直接发送，也不会在退出时发送中性 RC。控制
授权还要求独立 approval manifest 对快照、FC identity、参数哈希和冲突关闭状态进行绑定。

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
- `camera.device`
- `camera.capture_width` / `camera.capture_height` / `camera.fps` / `camera.fourcc`
- `camera.width` / `camera.height`
- `camera.fx` / `camera.fy` / `camera.cx` / `camera.cy`
- `camera.distortion_coefficients`
- `camera.pitch_up_deg` 或 `camera.R_BC`
- `rknn_detector.library` / `rknn_detector.model`
- `rknn_detector.conf_threshold` / `rknn_detector.iou_threshold`
- `rknn_detector.min_score` / `min_bbox_area` / `max_bbox_aspect_ratio`
- `rknn_detector.core_mask` / `max_det`
- `rknn_detector.temporal_gating_enabled` / `gate_radius_px` /
  `reacquire_area_ratio` / `track_hint_max_misses`
- `rknn_bytetrack.detector_conf_threshold` / `detector_iou_threshold`
- `rknn_bytetrack.track_high_thresh` / `track_low_thresh` / `new_track_thresh` /
  `match_thresh` / `low_match_thresh` / `fuse_score`
- `rknn_bytetrack.track_buffer_s` / `frame_rate` / `minimum_confirmed_frames` /
  `perception_rate_hz` / `final_min_score`

`match_thresh`控制高分检测的第一阶段关联；`low_match_thresh`控制低分检测的第二阶段IoU距离
上限，默认`0.5`。阈值越大，关联越宽松，必须通过相同视频A/B证明ID switch、fragment和误关联
均未恶化后才能修改。`final_min_score`是跟踪成功后的最终输出门槛，不能与建轨门槛混为一谈。
当前实物LOG_ONLY候选仅验证`track_low_thresh=0.05`和`final_min_score=0.05`；
`low_match_thresh=0.8`在低对比合成A/B中无改善，已拒绝用于实机候选配置。
- `rc_mapping.channel_map`
- `msp_runtime.set_raw_rc_channel_map`（当前必须与上项同为 `AETR1234`）
- `msp_runtime.prefill_enabled` / `prefill_min_frames` / `prefill_valid_min_us` /
  `prefill_valid_max_us`
- `msp_runtime.staged_command_timeout_s` / `shutdown_passthrough_frames`
- `msp_runtime.transport_mode`（无桨批准配置必须为 `async_pipeline`）
- `msp_runtime.response_drain_budget_ms`（当前 3 ms）/
  `msp_runtime.response_stale_s`（不得高于 0.25 s）
- `msp_runtime.status_poll_hz` / `attitude_poll_hz` / `raw_imu_poll_hz` /
  `motor_poll_hz` / `rc_poll_hz` / `analog_poll_hz`（当前无桨为 5/10/5/2/10/2 Hz）
- `msp_runtime.control_publish_hz`（当前无桨默认50 Hz）
- `safety.aux_enable.channel_index` / `min_us` / `max_us` /
  `satisfied_by_override_mode`（当前 RC7 共用模式必须为 true）
- `logging.platform_health_hz`（默认1 Hz；设为0仅用于显式关闭平台采样）
- `rc_mapping.rate_mapping_type`、`betaflight_rc_rate`、`betaflight_super_rate`、
  `betaflight_expo`、`betaflight_rate_profile_index`
- `rc_mapping.roll_rate_limit_deg_s`
- `rc_mapping.pitch_rate_limit_deg_s`
- `rc_mapping.yaw_rate_limit_deg_s`
- `rc_mapping.roll_command_limit_deg_s` / `pitch_command_limit_deg_s` /
  `yaw_command_limit_deg_s`
- `rc_mapping.thrust_min`
- `rc_mapping.thrust_hover`
- `rc_mapping.thrust_max`
- `rc_mapping.throttle_min_us`
- `rc_mapping.throttle_hover_us`
- `rc_mapping.throttle_max_us`
- `rc_mapping.neutral_throttle_us`
- `rc_mapping.max_delta_us_per_s`
- `guidance.law`（`ttc_png`或`fixed_vm_png`，禁止隐式混用）
- `guidance.max_guidance_accel_mps2`
- 固定Vm模式的`guidance.navigation_constant`和`guidance.fixed_vm_m_s`
- `guidance_command.guidance_eval_frame`（必须为`inertial_ned`）
- `guidance_command.rate_gain_input_frame`（必须为`body_frd`）
- `guidance_command.entry_handoff.enabled` / `duration_s` / `rate_source`（当前必须为`zero`）
- `guidance_command.tilt_envelope.enabled` / `max_roll_angle_deg` /
  `max_pitch_angle_deg` / `softcap_band_deg` / `hardcap_margin_deg` /
  `hardcap_level_kp` / `hardcap_max_level_rate_deg_s`
- `safety.aux_enable.channel_index`
- `safety.aux_enable.min_us` / `safety.aux_enable.max_us`
- `safety.telemetry_timeout_s`
- `safety.attitude_timeout_s`
- `safety.watchdog_timeout_s`
- `safety.min_vbat_v`
- `safety.motor_output_interlock.enabled` / `channel_count` / `max_output_us` /
  `max_spread_us` / `telemetry_timeout_s` / `latch_until_disarm`（无桨批准固定为启用、4路、
  1200 us、150 us、最多0.75 s、锁存到DISARM）

### RC 输出前额外必填

只读日志阶段可以让 `guidance_command.rate_gain_matrix` 保持全零。准备执行
`--control-mode msp_raw_rc --allow-control` 前必须写入并台架确认：

- `guidance_command.rate_gain_matrix`
- `guidance_command.hover_thrust`
- 接管起点采用的 gyro 轴向、时间戳年龄和 smoothstep 时长
- roll/pitch 软倾角、硬倾角和最大回平 rate；无桨批准上限不能直接作为有桨值
- roll/pitch/yaw 正负方向确认结果
- 最大允许命令幅度
- 丢目标时 throttle 策略

### 识别模型和相机运行参数

使用 RK3588 正式路径时，必须确认 `--detector-source rknn_bytetrack`、稳定相机路径、原生库
路径、`.rknn` 模型路径及 SHA256，并在 meta 中核对输入和 4 个输出张量。以下参数只适用于
PyTorch/ByteTrack 基线：

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
- 相机内参、畸变和显式固定上视外参 `R_BC`；`pitch_up_deg` 只允许只读诊断，不能批准控制。
- 端到端延迟、时间戳来源、安装刚性和振动隔离。
- 修改模型 RKNN artifact/hash、RGB/letterbox 约定、四输出 head schema、confidence/NMS、
  时序门控参数、有效检测距离和漏检率。
- ByteTrack 的 high/low/new/match 阈值、按实际 FPS 换算的 track buffer、三帧确认、ID switch
  和 lost 不输出策略；`rknn_native` 仅作为不带完整跟踪的诊断后端。

### 导引与控制映射

- 导引律选择。`ttc_png`要求尺度膨胀TTC有效；`fixed_vm_png`使用
  `N * Vm * (omega_LOS x lambda_I)`并仅旁路TTC门，不能旁路LOS和安全状态机。
- 固定Vm不是测速结果。`fixed_vm_m_s`必须写明试验假设，首轮拆桨候选值为1.0 m/s；正式飞行前
  必须根据实测拦截机速度重新确定，禁止照搬AirSim的`speed_ratio * intruder_speed`。
- `guidance_command.rate_gain_matrix` 的符号、轴向和增益。
- `rc_mapping.channel_map` 是否与 Betaflight Receiver tab 一致。
- roll/pitch/yaw 命令方向和最大幅度。
- throttle 标定：`thrust_min/thrust_hover/thrust_max` 到 RC us 的映射。
- RC 斜率限制、watchdog 超时、inactive 状态的锁存人工RC回退策略。
- 丢目标时的降级策略和恢复条件。

### 测试包线与安全

- 初始距离、高度差、侧向偏置、目标速度。
- 目标 S 机动幅度/周期、风、场地边界。
- 非碰撞近距通过标准和终止条件。
- 急停、人工接管、围挡/软目标、无桨和系留验收流程。
- 每轮测试后必须检查 CSV/meta 与 Blackbox 是否一致。

### 固定 Vm 无桨测试顺序

1. 从已验证硬件local配置复制独立VM配置和独立批准文件，不覆盖TTC配置。
   批准工具会显式校验并记录`law/N/Vm/N*Vm`，且无桨配置的
   `max_guidance_accel_mps2`不得超过1.0；修改任一值后必须重新生成批准文件。
2. DISARM、RC7人工侧，以`log_only`运行；网页应显示`fixed_vm_png`、`N*Vm=3.0`和TTC
   `BYPASS`，`SET_RAW_RC`写入必须为0。
3. 放置静止单目标，确认tracker/LOS/guidance有效且候选rate接近0；若静止目标产生持续大命令，
   先查相机抖动、姿态同步和LOS滤波，不进入接管。
4. 手持目标分别缓慢向机头、机尾、左、右移动，记录`omega_los`、`g_eval`、shaper输入和目标RC，
   确认符号与既有外参验证一致。
5. 仅在以上数据通过后，保持拆桨并使用独立VM批准启动`msp_raw_rc --allow-control`；先ARM怠速，
   再由操作者切RC7。确认ACTIVE/algorithm、命令不跳变、退出RC7立即回人工透传，最后DISARM。
   `motor_interlock_ok`必须持续为1；若变为0或`latched=1`，立即退出RC7并DISARM，不得在同一次
   ARM内清故障后重试。
6. 固定机体接管只用于短脉冲和方向确认。出现电机最大值高于1200 us、极差高于150 us或MSP写
   间隔超过60 ms时，本轮判失败；先取得并对齐Blackbox，再决定PID/I-term、mixer或调度修正。

## 不应直接沿用的参数

- AirSim/PX4 的 `mavlink_body_rate`、thrust、速度上限和命中判据。
- AirSim 中的视觉帧率、延迟、推力模型、碰撞判据。
- 未经台架确认的 `rate_gain_matrix`、hover throttle、最大角速度和 RC 斜率限制。
