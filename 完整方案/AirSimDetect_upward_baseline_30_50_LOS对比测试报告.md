# AirSim Detect upward-camera baseline 30-50m LOS comparison report

## 1. 实验目的

在真正 PX4 SITL actor 场景中使用 AirSim detect 理想 bbox 作为闭环检测源，比较两种捷联视觉比例导引。本报告优先使用 `n_cmd_g` 作为需用过载；旧日志没有该字段时才回退到 `g_eval` 等效过载。

- `TTC` 组：`ttc_png`，TTC 只参与增益调度，并保留 LOS/Vm soft guidance。
- `VM` 组：`fixed_vm_png`，不使用 TTC，固定 `N * V_m` 导引增益。
- `accel_integral` 输出模式：导引律先计算 `a_cmd` / `n_cmd_g`，再按当前仿真步长积分为速度 setpoint；这不是直接向 PX4 发送加速度 setpoint。
- `accel_body_rate` 输出模式：导引律先计算 PNG 需用加速度，再转换为 PX4 `SET_ATTITUDE_TARGET` 机体系角速度 `p/q/r` 和 thrust；速度只作为沿 LOS 保速参考，不再把 PNG 横向修正直接加到速度指令上。
- `accel_attitude` 输出模式：导引律先计算 PNG 需用加速度，再转换为 PX4 `SET_ATTITUDE_TARGET` 姿态四元数和 thrust；速度只作为沿 LOS 保速参考。

AirSim detect closed-loop upward-camera baseline at 30/35/40/45/50m; collision is success, YOLO baseline LOS is compared offline.

## 2. 基准条件

|参数|值|
|---|---|
|stamp|`upward_detect_baseline_30_50_los_20260629_234254`|
|settings|`/home/linux/Documents/PNG-px4-upward-camera/config/airsim_blocks_px4_actor_upward_camera_settings.json`|
|拦截机|`PX4 SITL / mavlink_body_rate`|
|目标 actor|`IntruderActor`|
|actor asset|`Quadrotor1`|
|actor scale|`1.5`|
|检测源|`airsim`|
|YOLO链路|未使用；本轮由 AirSim detect 函数直接提供理想 bbox|
|相机外参|`x=0.0, y=0.0, z=0.0`|
|upward centering|`True`, gain `8.0`, max accel `4.0 m/s^2`|
|near-hit radius|`1.5 m`|
|FOV / resolution|`120.0 deg`, `640x480`|
|高度差|`30.0 m`|
|目标速度 / speed ratio|`5.0 m/s` / `2.0`|
|rate_hz|`8.0`|
|guidance output|`accel_body_rate`|
|max guidance accel|`20.0 m/s^2`|
|min speed ratio|`0.6`|
|thrust model|`empirical`, mass `1.0 kg`, max total thrust `16.717785072 N`|
|body-rate tilt / attitude P|`35.0 deg` / `4.0`|
|body-rate roll/pitch max rate|`120.0` / `120.0 deg/s`|
|body-rate profile|`legacy`|
|body-rate v2 Kp roll/pitch/yaw|`5.0` / `5.0` / `3.0`|
|body-rate v2 max pq / slew pq-r|`120.0 deg/s` / `720.0`-`540.0 deg/s^2`|
|body-rate v2 thrust reserve / guard|`0.15` / error `0.55`, PNG scale `0.6`, speed-hold scale `0.45`|
|body-rate thrust|min/hover/max `0.25` / `0.5` / `0.85`|
|body-rate speed hold|gain `1.2`, max accel `8.0 m/s^2`, total limit `28.0 m/s^2`|
|attitude tilt / yaw lookahead|`25.0 deg` / `0.25 s`|
|attitude thrust|min/hover/max `0.25` / `0.5865998371` / `0.95`|
|attitude speed hold|gain `1.2`, max accel `6.0 m/s^2`, total limit `18.0 m/s^2`|
|LOS filter|`1`|
|LOS KF q lambda / lambda_dot|`0.0005` / `0.02`|
|LOS KF r / innovation gate|`0.008` / `0.75`|
|LOS terminal gate / delay|`1.2` / `0.18 s`|
|terminal image KF|predict `1.0 s`, reject `0.35 rad`, soft reject `1`|
|terminal image KF dynamics|accel noise `8.0 rad/s^2`, max rate `12.0 rad/s`|
|terminal velocity blind-push|`False`|
|terminal blind requires visual loss|`1`|
|terminal accel hold|`True`, window `0.35 s`, decay `0.6 s`, max `20.0 m/s^2`|
|frame_guard|`True`|
|bbox noise|`0`|

## 3. 总览图

![summary](assets/AirSimDetect_upward_baseline_30_50_LOS对比测试报告/summary_upward_detect_baseline_30_50_los_20260629_234254.png)

## 4. 汇总表

|组别|碰撞命中|近距命中|碰撞距离m|近距距离m|未命中距离m|最小中心距离m|检测帧/总帧|有效帧/总帧|平均检测FPS|
|---|---:|---:|---|---|---|---:|---:|---:|---:|
|TTC|5/5|5/5|30, 35, 40, 45, 50|30, 35, 40, 45, 50|-|0.531|395/414|405/414|2099.97|
|VM|5/5|5/5|30, 35, 40, 45, 50|30, 35, 40, 45, 50|-|0.681|407/430|413/430|1957.79|

## 5. 明细表

|组别|距离m|碰撞|近距|碰撞时间s|近距时间s|近距距离m|最小距离m|终点距离m|检测帧率|有效帧率|检测FPS|sim FPS|实际过载max g|速度指令差分P95 g|需用过载P95 g|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|TTC|30|1|1|9.77|9.52|1.283|0.720|0.720|94.9%|100.0%|2363.98|7.98|0.55|0.91|0.80|
|VM|30|1|1|9.27|9.02|1.141|0.681|0.736|94.7%|100.0%|1829.43|7.98|0.60|0.78|0.65|
|TTC|35|1|1|9.65|9.52|1.071|0.531|0.531|93.6%|100.0%|2093.12|7.98|0.63|0.51|0.52|
|VM|35|1|1|9.27|9.14|1.092|0.763|0.763|96.0%|100.0%|2043.12|7.98|0.55|0.50|0.62|
|TTC|40|1|1|10.02|9.77|1.472|0.667|0.667|97.5%|93.8%|2022.80|7.98|0.60|0.74|0.47|
|VM|40|1|1|9.90|9.65|1.376|0.851|0.878|96.2%|97.5%|1856.55|7.98|0.59|1.11|0.94|
|TTC|45|1|1|10.52|10.40|1.143|0.924|0.924|95.3%|97.6%|1997.56|7.98|0.62|0.79|0.45|
|VM|45|1|1|10.28|10.15|1.167|0.851|0.851|97.6%|97.6%|2034.51|7.98|0.61|0.76|0.47|
|TTC|50|1|1|11.28|11.15|1.268|0.814|0.814|95.6%|97.8%|2022.40|7.98|0.58|0.78|0.51|
|VM|50|1|1|14.54|14.28|1.226|0.703|1.008|90.6%|88.9%|2025.33|7.98|0.86|1.28|2.04|

## 6. 分距离曲线

每个距离一张图，包含真实中心距离、bbox 面积、TTC 估计、实际过载/需用过载和检测 FPS。

![30m](assets/AirSimDetect_upward_baseline_30_50_LOS对比测试报告/yolo_sitl_ttc_vm_030m.png)
![35m](assets/AirSimDetect_upward_baseline_30_50_LOS对比测试报告/yolo_sitl_ttc_vm_035m.png)
![40m](assets/AirSimDetect_upward_baseline_30_50_LOS对比测试报告/yolo_sitl_ttc_vm_040m.png)
![45m](assets/AirSimDetect_upward_baseline_30_50_LOS对比测试报告/yolo_sitl_ttc_vm_045m.png)
![50m](assets/AirSimDetect_upward_baseline_30_50_LOS对比测试报告/yolo_sitl_ttc_vm_050m.png)

## 7. LOS KF 与失败原因诊断

|组别|距离m|最近距离m|最近点状态|主要失败/降级原因|检测率|有效率|
|---|---:|---:|---|---|---:|---:|
|TTC|30|0.720|`bbox_bottom_clipped`|valid:59, area_not_expanding:13, image_kf_predict:4, bbox_bottom_clipped:3|94.9%|100.0%|
|VM|30|0.681|`bbox_bottom_clipped`|valid:69, image_kf_predict:4, bbox_bottom_clipped:2|94.7%|100.0%|
|TTC|35|0.531|`bbox_bottom_clipped`|valid:58, area_not_expanding:12, image_kf_predict:5, bbox_bottom_clipped:3|93.6%|100.0%|
|VM|35|0.763|`bbox_bottom_clipped`|valid:69, image_kf_predict:3, bbox_bottom_clipped:3|96.0%|100.0%|
|TTC|40|0.667|`terminal_lost`|valid:64, area_not_expanding:10, terminal_lost:3, bbox_bottom_clipped:2|97.5%|93.8%|
|VM|40|0.851|`bbox_bottom_clipped`|valid:71, bbox_bottom_clipped:6, image_kf_predict:3|96.2%|97.5%|
|TTC|45|0.924|`bbox_bottom_clipped`|valid:67, area_not_expanding:10, bbox_bottom_clipped:4, no_detection:2|95.3%|97.6%|
|VM|45|0.851|`bbox_bottom_clipped`|valid:77, bbox_bottom_clipped:4, no_detection:2|97.6%|97.6%|
|TTC|50|0.814|`bbox_bottom_clipped`|valid:73, area_not_expanding:10, bbox_bottom_clipped:4, no_detection:2|95.6%|97.8%|
|VM|50|0.703|`terminal_lost`|valid:94, terminal_lost:8, image_kf_predict:7, no_detection:4|90.6%|88.9%|

- LOS KF 参数：`q_lambda=0.0005`、`q_lambda_dot=0.02`、`r=0.008`、`innovation_reject=0.75`、`terminal_reject=1.2`。
- 本轮平均实际过载峰值约 `0.62 g`，平均需用过载 P95 约 `0.75 g`。两者不是同一个量：`n_cmd_g` 是导引层需求，实际过载还受 PX4 姿态/推力限制、约 8Hz 控制循环和 frame centering 限速影响。

## 8. 相机光心真值影子测试诊断

![shadow_summary](assets/AirSimDetect_upward_baseline_30_50_LOS对比测试报告/shadow_summary_upward_detect_baseline_30_50_los_20260629_234254.png)

影子测试不参与导引，只用日志中的相机光心 `camera_world_*` 与目标真值位置离线计算经典 `N*Vc` 和固定 `N*Vm` PNG 理论需用过载，并和视觉 LOS、检测连续性对齐。

|组别|距离m|碰撞|最小距离m|最近点检测率|最近点无检测帧|视觉LOS误差P95|影子N*Vc P95 g|影子N*Vm P95 g|视觉需用P95 g|实际过载max g|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|TTC|30|1|0.720|100.0%|0/8|26.8|0.39|0.85|0.80|0.55|
|VM|30|1|0.681|100.0%|0/9|63.9|0.33|0.54|0.65|0.60|
|TTC|35|1|0.531|87.5%|1/8|10.3|0.38|0.62|0.52|0.63|
|VM|35|1|0.763|100.0%|0/8|20.8|0.34|0.52|0.62|0.55|
|TTC|40|1|0.667|100.0%|0/8|13.2|0.37|0.81|0.47|0.60|
|VM|40|1|0.851|100.0%|0/9|67.9|0.32|0.54|0.94|0.59|
|TTC|45|1|0.924|100.0%|0/8|33.2|0.32|0.49|0.45|0.62|
|VM|45|1|0.851|100.0%|0/8|24.4|0.28|0.40|0.47|0.61|
|TTC|50|1|0.814|100.0%|0/8|27.6|0.29|0.45|0.51|0.58|
|VM|50|1|0.703|100.0%|0/9|12.1|0.26|0.75|2.04|0.86|

- 如果影子 `N*Vc` P95 很低但视觉 LOS 误差和无检测帧较高，优先定位检测连续性、LOS KF/外推和 frame-centering。
- 如果视觉需用过载高而实际过载低，优先定位 PX4 姿态/推力响应、倾角限制和 speed-hold 混合项。

## 9. body-rate 控制诊断

|组别|距离m|最近距离m|frame-centering激活|推力饱和|p/q/r峰值deg/s|roll/pitch指令峰值deg|thrust min/max|
|---|---:|---:|---:|---:|---|---|---|
|TTC|30|0.720|63.3%|2.5%|109.6/120.0/33.2|34.0/35.0|0.25/0.85|
|VM|30|0.681|73.3%|2.7%|120.0/120.0/26.6|35.0/35.0|0.25/0.85|
|TTC|35|0.531|96.2%|0.0%|120.0/120.0/34.0|35.0/33.9|0.60/0.79|
|VM|35|0.763|97.3%|1.3%|110.0/120.0/42.6|35.0/33.9|0.49/0.85|
|TTC|40|0.667|100.0%|0.0%|120.0/120.0/57.2|35.0/35.0|0.49/0.75|
|VM|40|0.851|98.8%|2.5%|120.0/120.0/50.1|35.0/35.0|0.25/0.76|
|TTC|45|0.924|97.6%|0.0%|58.4/120.0/22.7|12.3/35.0|0.50/0.79|
|VM|45|0.851|97.6%|0.0%|64.4/120.0/51.9|15.5/35.0|0.50/0.75|
|TTC|50|0.814|97.8%|0.0%|55.6/120.0/48.1|15.8/35.0|0.29/0.78|
|VM|50|0.703|94.9%|6.0%|120.0/120.0/60.0|35.0/35.0|0.25/0.85|

- 本轮 `body_rate_control_profile=legacy`。legacy body-rate 使用欧拉角误差比例环，没有 v2 的 frame guard 降权和 slew rate limit。
- `frame-centering激活` 表示固定上视相机进入视场保持/末端捕获/丢失保持状态的比例；比例高时，命中结果更受视场保持策略和 yaw 丢失外推影响。
- `推力饱和` 与 `thrust min/max` 用于判断末端是否被垂向机动和总推力限制约束；若饱和高，应优先放宽 thrust、tilt 或垂向速度上限。

## 10. PNG 到过载、姿态和角速度的控制流程

本轮实际使用 `guidance_output=accel_body_rate`、`px4_command_mode=mavlink_body_rate`、`body_rate_control_profile=legacy`。因此控制链路是“PNG 需用加速度 -> 合成控制加速度 -> PX4 `SET_ATTITUDE_TARGET` 机体系 `p/q/r` 角速度 + thrust”。

### 10.1 视觉量到 6D LOS

AirSim detect 函数输出目标 `box2D`，bbox 中心先通过相机内参转换成相机坐标系单位射线：

```text
x_n = (u - cx) / fx
y_n = (v - cy) / fy
lambda_C = normalize([x_n, y_n, 1])
```

再使用相机外参和机体姿态转到惯性系：

```text
lambda_I = normalize(R_IB * R_BC * lambda_C)
```

其中 `R_BC` 是相机到机体的固定安装旋转，`R_IB` 是机体到惯性系的姿态。LOS 角速度由相邻 LOS 差分并投影到垂直 LOS 的平面得到：

```text
lambda_dot = project_perpendicular((lambda_I[k] - lambda_I[k-1]) / dt, lambda_I[k])
omega_LOS = lambda_I x lambda_dot
```

启用 LOS KF 时，滤波器输出平滑后的 `lambda_I` 和 `omega_LOS`；末端允许更松的 innovation gate，避免目标仍在检测框内时 PNG 加速度被过早清零。

### 10.2 PNG 生成需用加速度和需用过载

两种导引的共同输出都是导引层需用加速度 `a_cmd`：

```text
a_cmd = guidance_gain * (omega_LOS x lambda_I)
a_cmd = clip_norm(a_cmd, max_guidance_accel_mps2)
n_cmd_g = ||a_cmd|| / g
```

`omega_LOS x lambda_I` 给出垂直于视线的修正方向；`n_cmd_g` 是导引层需用过载，只表示 PNG 希望产生的机动强度。它不等于无人机真实过载，真实过载还受 PX4 姿态控制、推力限制、速度保持项、视觉帧率和 frame centering 限速影响。

TTC 组使用 bbox 面积扩张估计 `TTC ~= A / A_dot`，当前只把 TTC 用作增益调度和末端触发；当 TTC 无效但 LOS 有效时，仍保留 LOS/V_m soft guidance。V_m 组不使用 TTC，直接采用固定：

```text
guidance_gain = N * V_m
V_m = speed_ratio * intruder_speed
```

### 10.3 与速度保持项合成

在 `accel_attitude` 和 `accel_body_rate` 中，PNG 横向修正不再积分成速度指令。速度只作为沿 LOS 的保速参考：

```text
v_ref = speed_cap * lambda_I
a_speed_hold = K_v * (v_ref - v_current)
a_control_I = clip_norm(a_cmd + a_speed_hold, total_accel_limit)
```

`a_cmd` 是纯 PNG 需用加速度；`a_speed_hold` 是工程闭环项，用于避免飞机速度掉到无法追击或过度横向漂移。报告中的 `n_cmd_g` 仍来自 `a_cmd`，而 `attitude_control_accel_*` / `body_rate_control_accel_*` 记录合成后的控制加速度。

### 10.4 加速度到姿态四元数链路

在 `accel_attitude + mavlink_attitude` 下，程序先由图像中心误差和 LOS 水平投影得到期望航向：

```text
yaw_sp = current_yaw + yaw_rate_cmd * attitude_yaw_lookahead_s
```

随后把惯性系合成加速度旋转到期望 yaw 对应的水平坐标系，得到 roll/pitch setpoint，并发送姿态四元数和 thrust。

### 10.5 加速度到机体系角速度链路

在 `accel_body_rate + mavlink_body_rate` 下，程序先把 `a_control_I` 转到机体系：

```text
a_control_B = R_BI * a_control_I
```

再得到期望 roll/pitch。legacy 用欧拉角误差比例环：

```text
p_cmd = K_att * (roll_sp  - roll)
q_cmd = K_att * (pitch_sp - pitch)
r_cmd = yaw_rate_cmd
```

body-rate v2 使用四元数误差：

```text
q_err = inverse(q_current) * q_desired
e_att = 2 * q_err.xyz
p_raw = Kp_roll  * e_att.x
q_raw = Kp_pitch * e_att.y
r_raw = Kp_yaw   * e_att.z + yaw_rate_cmd
```

随后做角速度限幅和斜率限制。v2 不增加一阶 LPF，只保留 slew rate limit，避免末端视觉闭环额外相位滞后。MAVLink 仍使用 `SET_ATTITUDE_TARGET`，但 `type_mask` 忽略姿态四元数，只让 PX4 接收 `body_roll_rate/body_pitch_rate/body_yaw_rate` 和 thrust。

v2 的 thrust 使用向量投影法，并通过 `body_rate_v2_thrust_reserve` 预留电机差速余量：

```text
z_B_I = R_IB * [0, 0, 1]^T
required_specific_force_I = [-a_control_I.x, -a_control_I.y, g - a_control_I.z]
thrust_raw = mass * dot(required_specific_force_I, z_B_I) / max_total_thrust
```

### 10.6 本报告中过载曲线的含义

- `需用过载 n_cmd_g`：由 PNG 的 `a_cmd` 直接换算，是导引层希望产生的过载。
- `实际过载 max g`：由拦截机真实速度差分估计，体现 PX4 和 AirSim 动力学真正实现出的机动。
- `速度指令差分 P95 g`：兼容旧速度输出模式的指标；在 `accel_body_rate` 和 `accel_attitude` 下主要作为参考，不代表直接发送给 PX4 的控制量。

因此，若 `n_cmd_g` 很平滑但实际过载不足，问题通常在姿态/推力响应、速度保持、限幅或视觉低帧率；若 `n_cmd_g` 本身突变，则应优先检查 LOS/KF、bbox 裁切、丢检外推和 frame guard 状态切换。

## 11. 结论

- TTC: 命中 `5/5`，命中距离 `30m, 35m, 40m, 45m, 50m`，未命中 `-`，检测帧比例 `95.4%`，有效导引帧比例 `97.8%`，平均检测 FPS `2099.97`。
- VM: 命中 `5/5`，命中距离 `30m, 35m, 40m, 45m, 50m`，未命中 `-`，检测帧比例 `94.7%`，有效导引帧比例 `96.0%`，平均检测 FPS `1957.79`。
- 本轮闭环检测源为 AirSim detect 函数，检测框为仿真理想 bbox；YOLO+ByteTrack 只在第 12 节作为离线 baseline 对照，不能把 detect 结果等同于真实视觉链路。
- `accel_integral` 模式的 `n_cmd_g` 来自导引层 `a_cmd`，底层仍通过 PX4/AirSim 速度 setpoint 闭环；实际过载由真实速度差分估计，因此会同时受 PX4 响应、速度限幅和视觉帧率影响。
- `accel_body_rate` 模式下 `n_cmd_g` 仍表示纯 PNG 需用过载；实际发送给 PX4 的是 `SET_ATTITUDE_TARGET` 机体系 `p/q/r` 角速度和归一化 thrust，日志中的 `body_rate_control_accel_*` 额外包含沿 LOS 的速度保持加速度。
- `accel_attitude` 模式下 `n_cmd_g` 同样表示纯 PNG 需用过载；实际发送给 PX4 的是 `SET_ATTITUDE_TARGET` 姿态四元数和归一化 thrust，日志中的 `attitude_control_accel_*` 记录姿态指令生成前的合成加速度。
## 12. 实际 LOS 与 YOLO baseline LOS 对比

本节为离线对比：`actual LOS = normalize(intruder - camera_world)`；视觉 LOS 使用各自 CSV 中的 `lambda_x/y/z`。
本轮 AirSim detect 是闭环检测源；YOLO LOS 来自已固化的 upward-camera YOLO+ByteTrack baseline 日志。两者轨迹不是逐帧同一条轨迹，因此表格比较的是各自相对 actual LOS 的误差分布。

![actual_vs_yolo_los_error_p95](assets/AirSimDetect_upward_baseline_30_50_LOS对比测试报告/actual_vs_yolo_los_error_p95.png)

下图给出逐帧 LOS 误差曲线，横轴为各自日志的归一化飞行进度；实线为 AirSim detect，虚线为 YOLO+ByteTrack。

![los_error_curves_TTC_detect_vs_yolo](assets/AirSimDetect_upward_baseline_30_50_LOS对比测试报告/los_error_curves_TTC_detect_vs_yolo.png)

![los_error_curves_VM_detect_vs_yolo](assets/AirSimDetect_upward_baseline_30_50_LOS对比测试报告/los_error_curves_VM_detect_vs_yolo.png)

|算法|距离m|detect命中|detect最小m|detect LOS P50/P95 deg|detect终端P95 deg|YOLO命中|YOLO最小m|YOLO LOS P50/P95 deg|YOLO终端P95 deg|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|TTC|30|1|0.720|1.93/6.48|16.56|1|1.881|3.49/17.89|24.50|
|TTC|35|1|0.531|1.58/6.52|3.26|1|1.324|4.17/7.91|19.42|
|TTC|40|1|0.667|1.49/6.67|2.02|1|1.701|3.85/13.38|20.94|
|TTC|45|1|0.924|1.33/6.99|16.93|1|1.610|1.64/13.37|21.12|
|TTC|50|1|0.814|1.34/7.09|15.04|1|1.760|1.61/10.64|10.68|
|VM|30|1|0.681|1.48/6.39|30.78|0|2.668|4.64/24.10|21.96|
|VM|35|1|0.763|1.51/6.76|9.24|1|1.603|3.74/19.34|20.73|
|VM|40|1|0.851|1.38/6.69|47.11|1|1.625|3.48/11.18|23.12|
|VM|45|1|0.851|1.04/6.91|12.46|1|1.512|1.81/42.89|12.43|
|VM|50|1|0.703|1.21/26.57|1.16|0|1.993|4.39/16.67|17.89|

要点：
- detect 闭环在 30/35/40/45/50m 的 TTC 和 VM 均 collision 命中，合计 `10/10`。
- YOLO baseline 在相同距离集内 TTC 为 `5/5` 命中；VM 在 `30m` 和 `50m` 未命中。
- detect 的全程 LOS P95 除 VM 50m 外通常约 `6-7 deg`；VM 50m detect 曲线中段有一次明显重捕获/外推尖峰，但终端 P95 已回到 `1.16 deg`。
- 曲线显示 YOLO baseline 在 VM 30m/45m/50m 等工况存在更长时间的 LOS 发散或尖峰，说明主要差异来自视觉检测连续性和终端重捕获，而不是 PNG 主公式。
- 终端 P95 会受大框、边缘框和终端捕获状态影响；它用于定位终端段抖动，不等同于全程平均 LOS 质量。
