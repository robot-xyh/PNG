# 当前项目实施方案

## 1. 目标

本实施方案对应当前仓库代码状态，目标是建立一条可测试、可回放、可审计的纯视觉导引验证链路，并为 Betaflight 实机部署提供安全的日志和受限 RC 输出适配。

实施优先级：

1. 保持算法模块可单元测试。
2. 保持 AirSim/PX4 验证路径稳定。
3. Betaflight 先 log-only，再无桨 RC 注入，最后系留/低速测试。
4. 所有实机参数必须经台架和日志确认。

## 2. 已实现模块

核心模块：

- `vision_guidance/geometry.py`：相机射线、旋转矩阵、相机外参。
- `vision_guidance/attitude_buffer.py`：姿态历史缓存和时间戳查表。
- `vision_guidance/los_filter.py`：6D LOS Kalman filter。
- `vision_guidance/ttc.py`：bbox 面积膨胀 TTC。
- `vision_guidance/png_eval.py`：TTC gain schedule 和 `GuidanceEval`。
- `vision_guidance/yolo_bytetrack_detector.py`：YOLOv8 + ByteTrack / KCF 检测适配。
- `vision_guidance/flight_control.py`：通用 setpoint、RC 映射、安全状态机、watchdog。
- `vision_guidance/betaflight_msp.py`：MSP v1 编解码、遥测读取、`MSP_SET_RAW_RC`。

运行入口：

- `examples/run_synthetic.py`
- `examples/run_airsim_blocks.py`
- `examples/run_airsim_gimbal_vision_png.py`
- `examples/run_airsim_strapdown_vision_png.py`
- `examples/run_airsim_truth_png.py`
- `examples/run_betaflight_log_only.py`

## 3. 阶段化实施流程

### 阶段 1：基础算法验证

命令：

```bash
python3 -m unittest discover -s tests -v
python3 examples/run_synthetic.py
```

退出条件：

- 几何、姿态缓存、LOS、TTC、pipeline 测试通过。
- 合成示例能输出有效 `GuidanceEval`。

### 阶段 2：AirSim 感知链路验证

使用 AirSim 内置 detection 或 YOLO/ByteTrack 验证 bbox 到 LOS/TTC 的链路：

```bash
python3 examples/run_airsim_blocks.py --duration-s 30
```

退出条件：

- 检测框只来自视觉接口，不使用目标真值进入导引。
- track id、bbox、LOS、TTC 和质量门控字段可解释。

### 阶段 3：AirSim/PX4 控制映射验证

固定上视相机 strapdown 路径用于验证视觉低帧率、PX4 setpoint 响应、frame guard 和 terminal extrapolation：

```bash
python3 examples/run_airsim_strapdown_vision_png.py \
  --enable-motion \
  --detector-source yolo_bytetrack \
  --yolo-model /path/to/model.pt \
  --yolo-class-id 0
```

退出条件：

- CSV/meta/plot 能说明命中、近失、丢检、裁切、PX4 响应和实际过载。
- `accel_body_rate` 和 `accel_attitude` 参数只作为 PX4 仿真结果记录，不迁移到 Betaflight。

### 阶段 4：Betaflight log-only 台架

命令：

```bash
python3 examples/run_betaflight_log_only.py \
  --config config/betaflight.example.json \
  --serial-port /dev/ttyUSB0 \
  --duration-s 60 \
  --detector-source none
```

退出条件：

- 能读取 MSP attitude/status/analog/RC。
- CSV 中 `telemetry_error` 可解释。
- meta JSON 保存 args、config、fields 和 FC identity。
- 不发送 `MSP_SET_RAW_RC`。

### 阶段 5：Betaflight 视觉 log-only

命令：

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

退出条件：

- bbox、LOS、TTC、`g_eval` 连续合理。
- `safety_state=LOG_ONLY`，`rc_active=0`。
- 延迟、漏检、裁切和 track 切换能从日志中解释。

### 阶段 6：无桨 RC 注入

命令：

```bash
python3 examples/run_betaflight_log_only.py \
  --config config/betaflight.example.json \
  --serial-port /dev/ttyUSB0 \
  --control-mode msp_raw_rc \
  --allow-control \
  --detector-source csv \
  --detections-csv /path/to/detections.csv
```

退出条件：

- AUX 未使能时不进入 active。
- AUX 使能且所有 gate 正常时才 `rc_active=1`。
- `rc_raw_ch*`、`rc_ch*`、`rc_clipped_ch*`、`rc_slew_limited_ch*` 与 Betaflight Receiver tab 一致。
- 丢目标、遥测超时、watchdog 超时、低电压会退出 active。

### 阶段 7：系留/低速测试

退出条件：

- 人工遥控优先级明确。
- 仓库 CSV/meta 与 Betaflight Blackbox 能对齐。
- `rate_gain_matrix`、hover throttle、最大角速度和斜率限制经过逐步扩大验证。

## 4. 配置管理

不要直接修改示例配置用于实机。建议复制：

```bash
cp config/betaflight.example.json config/betaflight.local.json
```

必须确认：

- `serial.port`, `serial.baud`
- `camera.*`
- `rc_mapping.*`
- `safety.*`
- `guidance_command.rate_gain_matrix`
- `guidance_command.guidance_eval_frame=inertial_ned`
- `guidance_command.rate_gain_input_frame=body_frd`

本机路径、串口号和模型路径不应提交到公共仓库。

## 5. Betaflight 优化与日志验收

当前仓库已存在 Betaflight 专用适配优化，验收时必须按代码实际字段检查。

### 5.1 已实现优化

- `guidance_command.rate_gain_matrix`：把曝光时刻经`R_IB^T`旋转得到的
  `g_eval_body_frd_x/y/z`映射到roll/pitch/yaw rate；不能直接消费惯性系`g_eval_x/y/z`。
- `rc_mapping.*`：配置 `AETR1234` 通道顺序、最大角速度、throttle 标定和 RC us 边界。
- `max_delta_us_per_s`：限制 RC 通道变化率，降低视觉抖动和矩阵误标定带来的阶跃。
- `safety.*`：遥测超时、姿态超时、watchdog、低电压和 AUX enable gate。
- `send_neutral_when_inactive`：非 active 时发送中性 RC，便于 Receiver tab 和日志确认降级。

上机前的最小验收表：

|检查|必须看到|
|---|---|
|MSP 读数|`telemetry_error` 为空，`roll_deg/pitch_deg/yaw_deg` 跟随机体变化|
|AUX gate|AUX 低时 `safety_state=READY` 且 `rc_active=0`，AUX 高且其他 gate 正常时才可 `ACTIVE`|
|导引有效|目标出现时 `los_valid=1`、`ttc_valid=1` 或有明确 `*_reject_reason`|
|RC 映射|`sp_roll_rate_deg_s` 符号正确，`rc_raw_ch*` 与 Receiver tab 方向一致|
|限幅|大命令会产生 `rc_clipped_ch*=1`，斜率限制会产生 `rc_slew_limited_ch*=1`|
|失效回中|丢目标或 watchdog 超时时 `safety_state=DEGRADED/FAILSAFE`，最终 `rc_active=0`|

### 5.2 日志代码位置

- `examples/run_betaflight_log_only.py::_log_fields`：定义 CSV 字段。
- `examples/run_betaflight_log_only.py::_log_row`：展开遥测、视觉、导引、setpoint、RC 和安全状态。
- `examples/run_betaflight_log_only.py::_write_run_meta`：保存参数、配置、字段列表和 FC identity。
- `tests/test_betaflight_logging.py`：验证日志字段完整性、缺失数据为空字符串、meta JSON 内容。
- `tests/test_flight_control.py`：验证 RC 映射、限幅、斜率限制、watchdog 和安全状态机。

日志验收不依赖真实串口；修改字段或映射后必须先跑单元测试，再做无桨台架。

## 6. 测试要求

每次修改后运行：

```bash
python3 -m unittest discover -s tests -v
python3 -m py_compile vision_guidance/flight_control.py vision_guidance/betaflight_msp.py examples/run_betaflight_log_only.py
```

Betaflight 实机相关修改必须补充独立单元测试，不能依赖真实串口才能通过。

## 7. 当前风险

- 真实相机曝光时间戳和 MSP 姿态时间不一定同源，需要实测延迟。
- MSP RC 注入通道只有 8 通道，AUX 分配必须受限。
- Betaflight mode flags 目前按整数记录，不解析成固件版本相关的 mode 名称。
- 未做硬件验证前，`rate_gain_matrix` 必须保持零或极小值。
