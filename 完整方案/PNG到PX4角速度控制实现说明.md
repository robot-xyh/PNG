# PNG 到 PX4 角速度/姿态控制实现说明

本文说明当前捷联视觉 PNG 程序中，从视觉 LOS/TTC 到 PX4 控制指令的实现流程。对应主程序为：

- `examples/run_airsim_strapdown_vision_png.py`
- 主要入口参数：`--guidance-law`、`--guidance-output-mode`、`--px4-command-mode`

需要先区分两条输出链路：

- `accel_body_rate + mavlink_body_rate`：PNG 加速度转换为 PX4 机体系角速度 `p/q/r` 和 thrust。这是真正的“PNG -> 角速度”链路。
- `accel_attitude + mavlink_attitude`：PNG 加速度转换为 PX4 姿态四元数和 thrust。最近的 SITL 批量实验默认使用这条链路，它不是直接发送角速度。

## 1. 输入量

视觉检测输出：

- `bbox center = (u, v)`
- `bbox area`
- `track_id`
- `score`
- `bbox clipped flags`

姿态缓存输出：

- `R_IB`：机体系到惯性系的旋转矩阵
- 曝光时间对齐后的机体姿态样本

视觉 LOS 估计输出：

- `lambda_I`：惯性系 LOS 单位向量
- `omega_los`：惯性系 LOS 角速度
- `los_quality`
- `reject_reason`

TTC 尺度通道输出：

- `ttc`
- `ttc_quality`
- `area_filtered`
- `area_dot_filtered`

## 2. 视觉 LOS 计算

bbox 中心点先转换为相机坐标系射线：

```text
x_n = (u - cx) / fx
y_n = (v - cy) / fy
los_C = normalize([x_n, y_n, 1])
```

再通过相机外参和机体姿态旋转到惯性系：

```text
lambda_I = normalize(R_IB * R_BC * los_C)
```

其中：

- `R_BC`：相机坐标系到机体系的旋转矩阵，包含相机固定安装角。
- `R_IB`：机体系到惯性系的旋转矩阵。
- `lambda_I`：惯性系 LOS 单位向量。

相邻 LOS 之间用差分得到 LOS 变化率，再投影到垂直于 LOS 的平面：

```text
lambda_dot_raw = project_perpendicular((lambda_k - lambda_{k-1}) / dt, lambda_k)
omega_los_raw = lambda_k x lambda_dot_raw
```

如果启用 6D LOS Kalman filter，则滤波器输出 `lambda_I` 和 `omega_los`。末端还会做两个处理：

- LOS 延迟补偿：

```text
lambda_dot = omega_los x lambda_I
lambda_pred = normalize(lambda_I + lambda_dot * delay_s)
omega_pred = project_perpendicular(omega_los, lambda_pred)
```

- 末端 KF 松弛门控：当目标面积较大、接近边缘、已经进入 terminal 状态或图像 KF 有效时，如果 KF 只是 `los_innovation_reject`，且 innovation 小于 `los_filter_terminal_innovation_reject`，允许使用原始 LOS/omega 继续导引。

## 3. 导引增益

当前有两种导引律入口。

### 3.1 固定 V_m 型

参数：

```text
--guidance-law fixed_vm_png
--navigation-constant N
```

增益：

```text
V_m = speed_ratio * intruder_speed
guidance_gain = N * V_m
```

这里 `V_m` 是拦截机期望速度量级，不需要真实目标位置和真实目标速度。

### 3.2 TTC 调度型

参数：

```text
--guidance-law ttc_png
--ttc-soft-guidance
```

TTC 来自 bbox 面积扩张：

```text
TTC ≈ A / A_dot
```

当 TTC 有效时，用 `TTCGainSchedule` 将 TTC 映射为增益。当前软导引模式下，TTC 主要做增益调度，主导引仍保留 LOS/V_m 结构：

```text
gain_scale = K(TTC) / K_max
guidance_gain = fixed_vm_gain * clamp(gain_scale, ttc_soft_min_gain_scale, 1)
```

当 TTC 无效时，只要 LOS 有效，仍可退化为固定 V_m 的 soft guidance。

## 4. PNG 加速度指令

当前加速度型 PNG 的核心实现为：

```text
a_cmd = guidance_gain * (omega_los x lambda_I)
a_cmd = clip_norm(a_cmd, max_guidance_accel_mps2)
```

含义：

- `lambda_I`：目标 LOS 单位向量。
- `omega_los`：LOS 角速度。
- `omega_los x lambda_I`：垂直于 LOS 的导引修正方向。
- `guidance_gain`：导引增益，来自固定 `N * V_m` 或 TTC 调度。
- `a_cmd`：导引层需用加速度，单位 `m/s^2`。

日志字段：

- `a_cmd_x/y/z`
- `a_cmd_norm_mps2`
- `n_cmd_g = |a_cmd| / g`
- `g_eval_x/y/z`

注意：`n_cmd_g` 是导引层需用过载，不等于无人机真实实现的过载。真实过载还受 PX4 姿态控制、推力限制、速度限制和视觉帧率影响。

## 5. 速度参考

在 `accel_body_rate` 和 `accel_attitude` 模式下，PNG 横向修正不再积分进速度指令。速度参考只用于沿 LOS 保速：

```text
v_ref = speed_cap * lambda_I
```

其中 `speed_cap` 会被 frame guard / terminal capture 调度：

- 远距：接近 `speed_ratio * intruder_speed`
- 目标靠近画面边缘：降低速度，优先保视场
- 末端 capture：进一步降低横向速度，避免目标被甩出画面

## 6. PNG -> 机体系角速度链路

该链路对应：

```text
--guidance-output-mode accel_body_rate
--px4-command-mode mavlink_body_rate
```

### 6.1 合成控制加速度

先在惯性系叠加沿 LOS 的速度保持项：

```text
a_speed_hold = K_v * (v_ref - v_current)
a_speed_hold = clip_norm(a_speed_hold, body_rate_speed_hold_max_accel_mps2)

a_control_I = a_cmd + a_speed_hold
a_control_I = clip_norm(a_control_I, body_rate_total_accel_limit_mps2)
```

其中：

- `a_cmd`：PNG 需用加速度。
- `a_speed_hold`：速度保持加速度。
- `a_control_I`：最终用于生成角速度的合成加速度。

日志字段：

- `body_rate_speed_hold_accel_x/y/z`
- `body_rate_control_accel_x/y/z`
- `body_rate_control_accel_norm_mps2`

### 6.2 惯性系加速度转机体系

```text
a_control_B = R_BI * a_control_I
R_BI = R_IB^T
```

其中：

- `a_control_B.x`：机体系前向加速度需求。
- `a_control_B.y`：机体系右向加速度需求。
- `a_control_B.z`：机体系下向加速度需求。

日志字段：

- `a_cmd_body_x/y/z`

### 6.3 加速度转期望姿态角

当前用小角度近似把横向加速度转成 roll/pitch setpoint：

```text
roll_sp  = body_rate_roll_gain  * a_control_B.y / g
pitch_sp = -body_rate_pitch_gain * a_control_B.x / g
```

并做倾角限制：

```text
roll_sp  = clamp(roll_sp,  -body_rate_max_tilt, body_rate_max_tilt)
pitch_sp = clamp(pitch_sp, -body_rate_max_tilt, body_rate_max_tilt)
```

符号解释：

- 正 `a_control_B.y` 表示希望向机体右侧加速，因此给正 roll。
- 正 `a_control_B.x` 表示希望向前加速，在 PX4/AirSim NED 约定下对应负 pitch。

### 6.4 姿态误差转角速度

用比例环把期望姿态角变成机体系角速度：

```text
p_cmd = K_att * (roll_sp  - roll)
q_cmd = K_att * (pitch_sp - pitch)
r_cmd = yaw_rate_cmd
```

并限幅：

```text
p_cmd = clamp(p_cmd, -body_rate_max_roll_rate,  body_rate_max_roll_rate)
q_cmd = clamp(q_cmd, -body_rate_max_pitch_rate, body_rate_max_pitch_rate)
r_cmd = clamp(r_cmd, -max_yaw_rate,             max_yaw_rate)
```

其中：

- `p_cmd`：机体系 roll rate，单位 `rad/s`。
- `q_cmd`：机体系 pitch rate，单位 `rad/s`。
- `r_cmd`：机体系 yaw rate，单位 `rad/s`。
- `yaw_rate_cmd`：由图像中心误差和 frame-centering / terminal extrapolation 产生。

日志字段：

- `roll_sp_deg`
- `pitch_sp_deg`
- `body_rate_p_rad_s`
- `body_rate_q_rad_s`
- `body_rate_r_rad_s`
- `body_rate_p_deg_s`
- `body_rate_q_deg_s`
- `body_rate_r_deg_s`

### 6.5 垂直加速度转 thrust

```text
thrust = body_rate_hover_thrust + body_rate_thrust_gain * (-a_control_I.z / g)
thrust = clamp(thrust, body_rate_min_thrust, body_rate_max_thrust)
```

AirSim/PX4 使用 NED 坐标，`z` 向下；因此希望向上加速时 `a_control_I.z < 0`，thrust 增大。

日志字段：

- `body_rate_thrust`

### 6.6 MAVLink 发送

发送函数为 `send_body_rate()`，使用 MAVLink：

```text
SET_ATTITUDE_TARGET
type_mask = ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
q = [1, 0, 0, 0]
body_roll_rate  = p_cmd
body_pitch_rate = q_cmd
body_yaw_rate   = r_cmd
thrust          = thrust
```

含义：

- 忽略姿态四元数。
- PX4 接收机体系角速度 `p/q/r` 和归一化 thrust。
- 这条链路要求 PX4 处于 Offboard，并持续接收 setpoint。

## 7. PNG -> 姿态四元数链路

该链路对应：

```text
--guidance-output-mode accel_attitude
--px4-command-mode mavlink_attitude
```

这是最近 50-100m SITL 批量测试使用的默认链路。

### 7.1 合成控制加速度

与角速度链路类似，但使用 attitude 参数：

```text
a_speed_hold = attitude_speed_hold_gain * (v_ref - v_current)
a_speed_hold = clip_norm(a_speed_hold, attitude_speed_hold_max_accel_mps2)

a_control_I = a_cmd + a_speed_hold
a_control_I = clip_norm(a_control_I, attitude_total_accel_limit_mps2)
```

日志字段：

- `attitude_speed_hold_accel_x/y/z`
- `attitude_control_accel_x/y/z`
- `attitude_control_accel_norm_mps2`

### 7.2 期望 yaw

先由图像误差得到 `yaw_rate_cmd_deg_s`。随后 attitude 模式用短时前视得到期望 yaw：

```text
yaw_sp = current_yaw + yaw_rate_cmd * attitude_yaw_lookahead_s
```

如果 `lambda_I` 有效，则优先让机头朝向 LOS 的水平投影：

```text
target_yaw = atan2(lambda_I.y, lambda_I.x)
yaw_error = wrap(target_yaw - current_yaw)
yaw_sp = current_yaw + clamp(
    yaw_error,
    -max_yaw_rate * attitude_yaw_lookahead_s,
     max_yaw_rate * attitude_yaw_lookahead_s
)
```

### 7.3 惯性系加速度转 yaw 坐标系

因为姿态 setpoint 同时包含 yaw，roll/pitch 必须在期望 yaw 对应的水平坐标系下生成：

```text
a_yaw_body = R_z(yaw_sp)^T * a_control_I
```

日志字段：

- `attitude_accel_yaw_body_x/y/z`

### 7.4 加速度转 roll/pitch setpoint

```text
pitch_sp = -a_yaw_body.x / g
roll_sp  =  a_yaw_body.y / g
```

并按 `attitude_max_tilt_deg` 限幅。

### 7.5 roll/pitch/yaw 转四元数

```text
R_sp = R_z(yaw_sp) * R_y(pitch_sp) * R_x(roll_sp)
q_sp = quat(R_sp)
```

日志字段：

- `attitude_roll_sp_deg`
- `attitude_pitch_sp_deg`
- `attitude_yaw_sp_deg`
- `attitude_quat_w/x/y/z`

### 7.6 thrust

```text
thrust = attitude_hover_thrust + attitude_thrust_gain * (-a_control_I.z / g)
thrust = clamp(thrust, attitude_min_thrust, attitude_max_thrust)
```

日志字段：

- `attitude_thrust`

### 7.7 MAVLink 发送

发送函数为 `send_attitude()`，使用 MAVLink：

```text
SET_ATTITUDE_TARGET
type_mask = BODY_ROLL_RATE_IGNORE
          | BODY_PITCH_RATE_IGNORE
          | BODY_YAW_RATE_IGNORE
q = [qw, qx, qy, qz]
body rates = 0
thrust = thrust
```

含义：

- PX4 忽略 body rate 字段。
- PX4 跟踪姿态四元数和 thrust。
- 该模式更接近 PX4 内部姿态控制器的常规使用方式，避免直接 body-rate 控制对调参和响应延迟过于敏感。

## 8. yaw-rate 的来源

捷联相机固定在机体上，因此视场保持必须控制航向。当前 yaw-rate 主要来自图像中心误差：

```text
yaw_error_rad = atan2(pixel_error_x, fx)
yaw_rate_cmd = yaw_error_gain * yaw_error_rad
yaw_rate_cmd = clamp(yaw_rate_cmd, -max_yaw_rate_deg, max_yaw_rate_deg)
```

之后会被 frame-centering / terminal-capture / image-KF 外推修正：

- 目标接近画面边缘时，提高“看住目标”的优先级。
- 丢检短时间内，使用图像 KF 预测的中心运动继续保持 yaw-rate。
- 末端 capture 时限制速度和横向修正，避免固定相机把目标甩出视场。

在 `accel_body_rate` 中，`yaw_rate_cmd` 直接成为 `r_cmd`。

在 `accel_attitude` 中，`yaw_rate_cmd` 只参与生成短时 `yaw_sp`，最终发送的是姿态四元数。

## 9. 当前批测默认链路

当前 `run_yolo_sitl_ttc_vm_batch.sh` 默认：

```text
GUIDANCE_OUTPUT_MODE=accel_attitude
PX4_COMMAND_MODE=mavlink_attitude
```

因此最近 TTC/V_m 50-100m 实验中，控制链路是：

```text
YOLO bbox
  -> LOS / LOS rate / TTC
  -> PNG acceleration a_cmd
  -> speed-hold acceleration
  -> attitude roll/pitch/yaw setpoint
  -> quaternion + thrust
  -> PX4 SET_ATTITUDE_TARGET
```

它不是：

```text
PNG acceleration -> p/q/r body-rate
```

如果要测试真正的角速度链路，应显式使用：

```bash
GUIDANCE_OUTPUT_MODE=accel_body_rate \
PX4_COMMAND_MODE=mavlink_body_rate \
./run_yolo_sitl_ttc_vm_batch.sh
```

## 10. 主要参数

PNG 和导引：

- `--guidance-law ttc_png | fixed_vm_png`
- `--navigation-constant`
- `--speed-ratio`
- `--max-guidance-accel-mps2`

body-rate 链路：

- `--body-rate-max-tilt-deg`
- `--body-rate-roll-gain`
- `--body-rate-pitch-gain`
- `--body-rate-attitude-p`
- `--body-rate-max-roll-rate-deg`
- `--body-rate-max-pitch-rate-deg`
- `--body-rate-hover-thrust`
- `--body-rate-thrust-gain`
- `--body-rate-min-thrust`
- `--body-rate-max-thrust`
- `--body-rate-speed-hold-gain`
- `--body-rate-speed-hold-max-accel-mps2`
- `--body-rate-total-accel-limit-mps2`

attitude 链路：

- `--attitude-max-tilt-deg`
- `--attitude-yaw-lookahead-s`
- `--attitude-hover-thrust`
- `--attitude-thrust-gain`
- `--attitude-min-thrust`
- `--attitude-max-thrust`
- `--attitude-speed-hold-gain`
- `--attitude-speed-hold-max-accel-mps2`
- `--attitude-total-accel-limit-mps2`

视场保持：

- `--yaw-error-gain`
- `--max-yaw-rate-deg`
- `--frame-centering`
- `--terminal-capture-speed-ratio`
- `--terminal-capture-max-lateral-speed`

## 11. 调试字段

CSV 中建议重点查看：

- LOS：`lambda_x/y/z`、`omega_x/y/z`、`omega_effective_norm_rad_s`
- PNG：`a_cmd_x/y/z`、`n_cmd_g`
- body-rate：`body_rate_p/q/r_rad_s`、`roll_sp_deg`、`pitch_sp_deg`、`body_rate_thrust`
- attitude：`attitude_roll_sp_deg`、`attitude_pitch_sp_deg`、`attitude_yaw_sp_deg`、`attitude_quat_w/x/y/z`、`attitude_thrust`
- 实际响应：`interceptor_vel_x/y/z`、`load_factor_fd_g`
- 视觉状态：`detected`、`valid`、`reject_reason`、`image_kf_mode`、`bbox_top_clipped`、`bbox_clipped`

## 12. 关键结论

1. 当前程序已经具备真正的 PNG 加速度输出，不再只能把 PNG 修正积分成速度。
2. 真正输出角速度的是 `accel_body_rate + mavlink_body_rate`。
3. 最近默认测试使用的是 `accel_attitude + mavlink_attitude`，它输出姿态四元数而不是角速度。
4. 两条链路都保留 `v_ref = speed_cap * lambda_I` 作为沿 LOS 保速参考，但 PNG 横向修正不再直接加到速度指令中。
5. `n_cmd_g` 是导引层需用过载；真实过载需要看 `load_factor_fd_g` 或速度差分估计。
