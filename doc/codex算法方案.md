# 纯视觉 PNG 算法方案

## 1. 文档定位

本文描述当前仓库实际实现的算法流程。项目核心是纯视觉 LOS/TTC/PNG 评估链路，并提供 AirSim/PX4 仿真闭环和 Betaflight MSP/RC 实机适配。本文不描述旧版 Python 惯导对准方案，也不假设 Pixhawk 是唯一飞控。

## 2. 输入数据契约

所有检测源最终都转换为统一结构：

```text
FrameDetection:
  frame_id
  exposure_ts
  bbox_xyxy
  track_id
  score
```

检测源包括：

- AirSim `simGetDetections` 的 `box2D`。
- YOLOv8 + ByteTrack。
- YOLOv8 + KCF。
- Betaflight runner 中的 CSV 回放或 OpenCV 相机 + YOLO。

YOLO 模式 fail fast：缺模型、依赖或 `class_id` 时直接退出，不静默回退。

## 3. 坐标与时间对齐

相机模型使用 pinhole 内参：

```text
fx, fy, cx, cy, width, height
```

bbox 中心转相机系单位视线：

```text
x = (u - cx) / fx
y = (v - cy) / fy
los_C = normalize([x, y, 1])
```

固定相机外参 `R_BC` 将相机系转机体系，姿态缓存 `AttitudeHistoryBuffer` 按曝光时间查找或插值 `R_IB`：

```text
lambda_I = normalize(R_IB(t_exposure) * R_BC * los_C)
```

时间戳对齐是算法前提。姿态缓存缺失、时间戳超界或 track id 切换时，本帧不能作为正常更新。

## 4. 6D LOS 滤波

LOS 滤波器状态为：

```text
X = [lambda_x, lambda_y, lambda_z,
     lambda_dot_x, lambda_dot_y, lambda_dot_z]
```

它只使用 bbox 中心形成的角度观测，不使用 bbox 宽、高、面积或目标真实尺寸。更新后强制：

- `lambda_I` 归一化。
- `lambda_dot_I` 投影到垂直 `lambda_I` 的平面。
- `omega_los = lambda_I x lambda_dot_I`。

质量门控包括：

- 时间戳单调性。
- 姿态查表成功。
- track id 连续。
- innovation norm 未超过阈值。

## 5. Scale Expansion / TTC 通道

TTC 通道只使用 bbox 面积的相对膨胀趋势：

```text
A = bbox_width * bbox_height
TTC ~= A / A_dot
```

实现中会对面积和面积导数滤波，并对以下情况降权或拒绝：

- bbox 裁切。
- 面积过小。
- 面积突变。
- track id 切换。
- 面积导数无效或不表示接近。

TTC 不是距离，不能作为真实 range 使用。它只表示图像尺度膨胀的接近趋势。

## 6. 导引评估量

当前 reusable pipeline 输出：

```text
GuidanceEval:
  timestamp
  g_eval
  valid
  quality
  reject_reason
```

默认评估量：

```text
g_eval = K(TTC) * lambda_dot_I
```

`K(TTC)` 是平滑增益调度，不是实机控制增益。`g_eval` 的用途取决于运行路径：

- 合成和 AirSim Blocks：用于日志、评估或 bounded velocity command。
- AirSim/PX4 strapdown：可进一步生成 PNG 加速度，再映射为 velocity、body-rate/thrust 或 attitude/thrust。
- Betaflight：经 `rate_gain_matrix` 映射为 roll/pitch/yaw rate 候选量，再转 RC。

## 7. PX4/AirSim 控制映射

strapdown runner 支持三类输出：

1. 速度类：把导引候选速度发给 AirSim/PX4 velocity command。
2. `accel_body_rate`：PNG 需用加速度 + speed-hold，映射到 PX4 `SET_ATTITUDE_TARGET` body rates + thrust。
3. `accel_attitude`：PNG 需用加速度 + speed-hold，映射到 PX4 attitude quaternion + thrust。

PX4 结果用于仿真和 HIL 验证，不直接决定 Betaflight 参数。

## 8. Betaflight RC 映射

Betaflight 路径由 `vision_guidance.flight_control` 和 `vision_guidance.betaflight_msp` 实现：

```text
GuidanceEval
  -> rate_gain_matrix
  -> GuidanceSetpoint(roll_rate, pitch_rate, yaw_rate, thrust)
  -> RcCommandMapper
  -> RcCommand(ch1..ch8)
  -> MSP_SET_RAW_RC
```

RC 映射支持：

- `AETR1234` 等通道顺序。
- roll/pitch/yaw rate limit。
- throttle min/hover/max 标定。
- RC us 限幅。
- 斜率限制。
- raw/clipped/slew 诊断日志。

安全状态机要求 `allow_control`、AUX 使能、遥测新鲜、姿态同步、电压正常、watchdog 正常且目标有效，才允许 active RC 输出。

### 8.1 已实现的 Betaflight 优化

仓库中针对 Betaflight 的优化是“导引候选量到 RC 输入”的适配优化，不是 PX4 body-rate
控制器的移植。关键代码：

- `guidance_eval_to_setpoint()`：`rate_gain_matrix @ g_eval` 生成 roll/pitch/yaw rate。
- `RcCommandMapper.map_setpoint()`：rate/thrust 转 RC us，输出 raw、clipped、slew 三组诊断。
- `BetaflightSafetyStateMachine.update()`：按 telemetry、attitude、voltage、watchdog、AUX、
  target_valid 逐项 gate。
- `CommandWatchdog`：只有最近有效 `GuidanceEval` 新鲜时才允许 active。
- `BetaflightMSPAdapter.send_raw_rc()`：把最终 RC 通道打包为 `MSP_SET_RAW_RC`。

公式如下：

```text
rates_deg_s = rate_gain_matrix * g_eval + yaw_rate_bias
thrust      = hover_thrust
rc_rate_us  = mid_us + rates_deg_s / rate_limit_deg_s * (max_us - min_us) / 2
```

throttle 使用分段线性标定：

```text
thrust_min -> throttle_min_us
thrust_hover -> throttle_hover_us
thrust_max -> throttle_max_us
```

RC 输出先经过 `min_us/max_us` 限幅，再经过 `max_delta_us_per_s` 斜率限制。这样即使
`rate_gain_matrix` 过大，也能在日志中看到 `rc_clipped_ch*` 或 `rc_slew_limited_ch*`，
并在台架阶段停止放大参数。

### 8.2 Betaflight 与 AirSim/PX4 的差异

AirSim/PX4 strapdown 可接收 velocity、body-rate/thrust 或 attitude/thrust。Betaflight
runner 当前只输出 RC 语义：

```text
roll/pitch/yaw rate candidate + throttle candidate -> RC channels
```

因此以下参数不能直接复用：

- PX4 `body_rate_p/q/r`、`body_rate_thrust`、`attitude_quat`。
- AirSim thrust model、碰撞判据和速度上限。
- upward-camera 末端 `frame_guard`、`terminal_accel_hold` 的 PX4 参数。

Betaflight 需要重新确认 `rate_gain_matrix`、channel map、Betaflight rate/expo/PID、
hover throttle、RC 斜率和 AUX gate。

## 9. 日志与诊断

Betaflight CSV 已记录：

- MSP status、attitude、analog、完整输入 RC。
- bbox、面积、裁切、track id。
- LOS、LOS rate、LOS omega、TTC。
- `g_eval`、setpoint、raw RC、最终 RC、限幅和斜率限制。
- safety state、age、gate flags。

每次运行同时写 `*_meta.json`，保存 config、args、字段列表和 FC identity。

日志代码在 `examples/run_betaflight_log_only.py`：

- `_log_fields()` 固定字段顺序，并按 `channel_count` 展开 `rc_in_ch*`、`rc_raw_ch*`、
  `rc_ch*`、`rc_clipped_ch*`、`rc_slew_limited_ch*`。
- `_log_row()` 把 `BetaflightTelemetry`、`VisionGuidanceResult`、`GuidanceSetpoint`、
  `RcCommand` 和 safety decision 合成一行。
- `_write_run_meta()` 保存 `args`、`config`、`fields`、`fc_identity` 和 CSV 路径。

排故顺序：

1. 先看 `telemetry_error` 和 `send_error`，确认 MSP 链路。
2. 再看 `safety_state/safety_reason` 与 gate 字段，确认为什么没有 active。
3. 再看 `los_valid/ttc_valid/guidance_valid`，确认视觉导引是否有效。
4. 最后看 `sp_*`、`rc_raw_ch*`、`rc_ch*`、`rc_clipped_ch*`、`rc_slew_limited_ch*`，确认映射和限幅。

## 10. 关键限制

- 单目视觉不能可靠给出绝对距离和闭合速度。
- TTC 对 bbox jitter、裁切、目标姿态变化和曝光模糊敏感。
- 低视觉帧率会显著放大末端延迟风险。
- 相机内参、外参和安装刚性错误会直接产生系统性 LOS 偏差。
- Betaflight RC 映射必须重新标定，不能沿用 PX4 body-rate/thrust 参数。
