# YOLO+KCF terminal_v2 PX4 SITL TTC / V_m 50-100m 拦截测试报告

## 1. 实验目的

按照此前已命中的 YOLO 案例配置，改用真正 PX4 SITL actor 场景，比较两种捷联视觉比例导引。本报告优先使用 `n_cmd_g` 作为需用过载；旧日志没有该字段时才回退到 `g_eval` 等效过载。

- `TTC` 组：`ttc_png`，TTC 只参与增益调度，并保留 LOS/Vm soft guidance。
- `VM` 组：`fixed_vm_png`，不使用 TTC，固定 `N * V_m` 导引增益。
- `accel_integral` 输出模式：导引律先计算 `a_cmd` / `n_cmd_g`，再按当前仿真步长积分为速度 setpoint；这不是直接向 PX4 发送加速度 setpoint。
- `accel_body_rate` 输出模式：导引律先计算 PNG 需用加速度，再转换为 PX4 `SET_ATTITUDE_TARGET` 机体系角速度 `p/q/r` 和 thrust；速度只作为沿 LOS 保速参考，不再把 PNG 横向修正直接加到速度指令上。
- `accel_attitude` 输出模式：导引律先计算 PNG 需用加速度，再转换为 PX4 `SET_ATTITUDE_TARGET` 姿态四元数和 thrust；速度只作为沿 LOS 保速参考。

检测源 yolo_kcf，终端策略 terminal_v2，TTC 与 V_m 均测试 50-100m。

## 2. 基准条件

|参数|值|
|---|---|
|stamp|`yolo_kcf_terminal_v2_20260624_085728`|
|settings|`/home/linux/Documents/PNG/config/airsim_blocks_px4_actor_settings.json`|
|拦截机|`PX4 SITL / mavlink_attitude`|
|目标 actor|`IntruderActor`|
|actor asset|`Quadrotor1`|
|actor scale|`1.0`|
|检测源|`yolo_kcf`|
|YOLO model|`vision_guidance/best.pt`|
|YOLO device|`0` runtime `cpu`|
|YOLO conf / iou / imgsz|`0.1` / `0.7` / `640`|
|tracker|`bytetrack.yaml`，single target `1`|
|相机外参|`x=0.5, y=0.0, z=0.0`|
|FOV / resolution|`120.0 deg`, `640x480`|
|高度差|`20.0 m`|
|目标速度 / speed ratio|`5.0 m/s` / `2.0`|
|rate_hz|`8.0`|
|guidance output|`accel_attitude`|
|max guidance accel|`15.0 m/s^2`|
|min speed ratio|`0.6`|
|thrust model|`airsim_generic_quad`, mass `1.0 kg`, max total thrust `16.717785072 N`|
|body-rate tilt / attitude P|`20.0 deg` / `4.0`|
|body-rate roll/pitch max rate|`60.0` / `60.0 deg/s`|
|body-rate thrust|min/hover/max `0.25` / `0.5865998371` / `0.95`|
|body-rate speed hold|gain `1.2`, max accel `6.0 m/s^2`, total limit `18.0 m/s^2`|
|attitude tilt / yaw lookahead|`35.0 deg` / `0.25 s`|
|attitude thrust|min/hover/max `0.25` / `0.5865998371` / `0.95`|
|attitude speed hold|gain `1.2`, max accel `4.0 m/s^2`, total limit `24.0 m/s^2`|
|LOS filter|`1`|
|LOS KF q lambda / lambda_dot|`0.0005` / `0.02`|
|LOS KF r / innovation gate|`0.008` / `0.75`|
|LOS terminal gate / delay|`1.2` / `0.18 s`|
|terminal image KF|predict `0.55 s`, reject `0.35 rad`, soft reject `1`|
|terminal image KF dynamics|accel noise `12.0 rad/s^2`, max rate `12.0 rad/s`|
|frame_guard|`True`|
|bbox noise|`0`|

## 3. 总览图

![summary](assets/YOLO_KCF_terminal_v2_50_100测试报告/summary_yolo_kcf_terminal_v2_20260624_085728.png)

## 4. 汇总表

|组别|命中数|命中距离m|未命中距离m|最小中心距离m|检测帧/总帧|有效帧/总帧|平均检测FPS|
|---|---:|---|---|---:|---:|---:|---:|
|TTC|0/6|-|50, 60, 70, 80, 90, 100|3.246|1238/1840|1148/1840|9.06|
|VM|0/6|-|50, 60, 70, 80, 90, 100|3.560|1073/1816|987/1816|8.70|

## 5. 明细表

|组别|距离m|碰撞|碰撞时间s|最小距离m|终点距离m|检测帧率|有效帧率|YOLO FPS|sim FPS|实际过载max g|速度指令差分P95 g|需用过载P95 g|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|TTC|50|0|-|4.150|4.827|90.5%|89.2%|9.26|7.86|0.94|3.69|1.49|
|VM|50|0|-|4.400|20.439|83.8%|78.7%|9.22|7.85|0.95|3.69|1.45|
|TTC|60|0|-|3.246|8.929|85.1%|85.1%|9.08|7.81|1.08|3.72|1.41|
|VM|60|0|-|3.560|9.607|84.7%|84.7%|8.66|7.67|1.12|3.73|1.42|
|TTC|70|0|-|4.835|10.848|75.1%|75.1%|8.98|7.84|1.00|3.69|1.52|
|VM|70|0|-|3.922|24.345|61.3%|60.9%|8.54|7.69|0.99|3.68|1.33|
|TTC|80|0|-|4.598|14.498|81.8%|66.9%|8.92|7.76|0.96|3.68|1.33|
|VM|80|0|-|38.387|104.319|43.0%|32.5%|8.61|7.75|0.81|3.19|1.51|
|TTC|90|0|-|62.137|347.033|19.6%|8.0%|9.09|7.90|1.12|0.61|0.13|
|VM|90|0|-|5.061|8.753|55.0%|55.8%|8.49|7.72|1.02|3.73|1.07|
|TTC|100|0|-|5.051|12.686|65.4%|66.6%|9.01|7.82|1.09|3.71|1.11|
|VM|100|0|-|25.674|119.189|40.8%|28.9%|8.65|7.81|0.85|3.66|0.80|

## 6. 分距离曲线

每个距离一张图，包含真实中心距离、bbox 面积、TTC 估计、实际过载/需用过载和 YOLO 检测 FPS。

![50m](assets/YOLO_KCF_terminal_v2_50_100测试报告/yolo_sitl_ttc_vm_050m.png)
![60m](assets/YOLO_KCF_terminal_v2_50_100测试报告/yolo_sitl_ttc_vm_060m.png)
![70m](assets/YOLO_KCF_terminal_v2_50_100测试报告/yolo_sitl_ttc_vm_070m.png)
![80m](assets/YOLO_KCF_terminal_v2_50_100测试报告/yolo_sitl_ttc_vm_080m.png)
![90m](assets/YOLO_KCF_terminal_v2_50_100测试报告/yolo_sitl_ttc_vm_090m.png)
![100m](assets/YOLO_KCF_terminal_v2_50_100测试报告/yolo_sitl_ttc_vm_100m.png)

## 7. LOS KF 与失败原因诊断

|组别|距离m|最近距离m|最近点状态|主要失败/降级原因|检测率|有效率|
|---|---:|---:|---|---|---:|---:|
|TTC|50|4.150|`valid`|valid:117, area_not_expanding:82, no_detection:22, bbox_area_jump:4|90.5%|89.2%|
|VM|50|4.400|`valid`|valid:185, no_detection:38, los_innovation_reject:12|83.8%|78.7%|
|TTC|60|3.246|`valid`|valid:115, area_not_expanding:110, no_detection:40, ttc_out_of_range:2|85.1%|85.1%|
|VM|60|3.560|`valid`|valid:221, no_detection:40|84.7%|84.7%|
|TTC|70|4.835|`valid`|valid:127, area_not_expanding:87, no_detection:72, bbox_area_jump:7|75.1%|75.1%|
|VM|70|3.922|`no_detection`|valid:181, no_detection:115, los_innovation_reject:1|61.3%|60.9%|
|TTC|80|4.598|`no_detection`|valid:109, area_not_expanding:89, los_innovation_reject:56, no_detection:55|81.8%|66.9%|
|VM|80|38.387|`no_detection`|no_detection:187, valid:101, los_innovation_reject:39, image_kf_predict:8|43.0%|32.5%|
|TTC|90|62.137|`los_innovation_reject`|no_detection:277, los_innovation_reject:47, area_not_expanding:13, valid:7|19.6%|8.0%|
|VM|90|5.061|`valid`|valid:188, no_detection:151, image_kf_predict:3|55.0%|55.8%|
|TTC|100|5.051|`valid`|no_detection:116, area_not_expanding:113, valid:108, image_kf_predict:4|65.4%|66.6%|
|VM|100|25.674|`los_innovation_reject`|no_detection:201, valid:96, los_innovation_reject:45, image_kf_predict:4|40.8%|28.9%|

- LOS KF 参数：`q_lambda=0.0005`、`q_lambda_dot=0.02`、`r=0.008`、`innovation_reject=0.75`、`terminal_reject=1.2`。
- 检测率低于 60% 的工况：VM 80m(43.0%)，TTC 90m(19.6%)，VM 90m(55.0%)，VM 100m(40.8%)。这类失败优先归因于 YOLO/ByteTrack 连续性和固定相机视场保持，而不是导引律公式本身。
- 最近点处仍处于降级或无效状态的未命中工况：VM 70m:`no_detection`，TTC 80m:`no_detection`，VM 80m:`no_detection`，TTC 90m:`los_innovation_reject`，VM 100m:`los_innovation_reject`。这些样本说明末端质量门、视觉外推和 bbox 裁切处理仍会影响命中窗口。
- 本轮平均实际过载峰值约 `0.99 g`，平均需用过载 P95 约 `1.21 g`。两者不是同一个量：`n_cmd_g` 是导引层需求，实际过载还受 PX4 姿态/推力限制、YOLO 约 9 FPS 采样和 frame centering 限速影响。

## 8. 相机光心真值影子测试诊断

![shadow_summary](assets/YOLO_KCF_terminal_v2_50_100测试报告/shadow_summary_yolo_kcf_terminal_v2_20260624_085728.png)

影子测试不参与导引，只用日志中的相机光心 `camera_world_*` 与目标真值位置离线计算经典 `N*Vc` 和固定 `N*Vm` PNG 理论需用过载，并和视觉 LOS、检测连续性对齐。

|组别|距离m|碰撞|最小距离m|最近点检测率|最近点无检测帧|视觉LOS误差P95|影子N*Vc P95 g|影子N*Vm P95 g|视觉需用P95 g|实际过载max g|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|TTC|50|0|4.150|100.0%|0/15|46.9|0.23|1.16|1.49|0.94|
|VM|50|0|4.400|85.7%|2/14|30.8|0.17|1.50|1.45|0.95|
|TTC|60|0|3.246|73.3%|4/15|28.6|0.22|2.57|1.41|1.08|
|VM|60|0|3.560|64.3%|5/14|29.1|0.18|1.41|1.42|1.12|
|TTC|70|0|4.835|73.3%|4/15|54.9|0.22|1.59|1.52|1.00|
|VM|70|0|3.922|35.7%|9/14|65.2|0.25|1.66|1.33|0.99|
|TTC|80|0|4.598|78.6%|3/14|44.7|0.13|1.04|1.33|0.96|
|VM|80|0|38.387|73.3%|4/15|58.3|0.09|0.38|1.51|0.81|
|TTC|90|0|62.137|100.0%|0/15|54.9|0.09|0.39|0.13|1.12|
|VM|90|0|5.061|85.7%|2/14|10.8|0.11|1.11|1.07|1.02|
|TTC|100|0|5.051|100.0%|0/14|24.0|0.17|1.21|1.11|1.09|
|VM|100|0|25.674|100.0%|0/15|43.0|0.11|0.50|0.80|0.85|

- 如果影子 `N*Vc` P95 很低但视觉 LOS 误差和无检测帧较高，优先定位检测连续性、LOS KF/外推和 frame-centering。
- 如果视觉需用过载高而实际过载低，优先定位 PX4 姿态/推力响应、倾角限制和 speed-hold 混合项。

## 9. PNG 到过载、姿态和角速度的控制流程

当前批测默认 `guidance_output=accel_attitude`、`px4_command_mode=mavlink_attitude`，因此实际链路是“PNG 需用加速度 -> 姿态四元数 + thrust”，不是直接向 PX4 发送角速度。项目中仍保留 `accel_body_rate + mavlink_body_rate`，该模式才是“PNG 需用加速度 -> 机体系角速度 p/q/r + thrust”。

### 9.1 视觉量到 6D LOS

YOLOv8 + ByteTrack 输出 `bbox center=(u,v)`、`bbox area`、`track_id` 和置信度。bbox 中心先通过相机内参转换成相机坐标系单位射线：

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

### 9.2 PNG 生成需用加速度和需用过载

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

### 9.3 与速度保持项合成

在 `accel_attitude` 和 `accel_body_rate` 中，PNG 横向修正不再积分成速度指令。速度只作为沿 LOS 的保速参考：

```text
v_ref = speed_cap * lambda_I
a_speed_hold = K_v * (v_ref - v_current)
a_control_I = clip_norm(a_cmd + a_speed_hold, total_accel_limit)
```

`a_cmd` 是纯 PNG 需用加速度；`a_speed_hold` 是工程闭环项，用于避免飞机速度掉到无法追击或过度横向漂移。报告中的 `n_cmd_g` 仍来自 `a_cmd`，而 `attitude_control_accel_*` / `body_rate_control_accel_*` 记录合成后的控制加速度。

### 9.4 加速度到姿态四元数，本轮默认链路

当前批测使用 `accel_attitude + mavlink_attitude`。程序先由图像中心误差和 LOS 水平投影得到期望航向：

```text
yaw_sp = current_yaw + yaw_rate_cmd * attitude_yaw_lookahead_s
```

随后把惯性系合成加速度旋转到期望 yaw 对应的水平坐标系：

```text
a_yaw_body = R_z(yaw_sp)^T * a_control_I
```

再用小角度近似得到姿态 setpoint：

```text
pitch_sp = -a_yaw_body.x / g
roll_sp  =  a_yaw_body.y / g
```

roll/pitch 按 `attitude_max_tilt_deg` 限幅，然后和 `yaw_sp` 合成为姿态四元数：

```text
R_sp = R_z(yaw_sp) * R_y(pitch_sp) * R_x(roll_sp)
q_sp = quat(R_sp)
```

垂向加速度通过 AirSim GenericQuad 质量和最大推力参数换算成归一化 thrust 的线性近似：

```text
thrust = hover_thrust + thrust_gain * (-a_control_I.z / g)
thrust = clamp(thrust, min_thrust, max_thrust)
```

最后通过 MAVLink `SET_ATTITUDE_TARGET` 发送：

```text
type_mask = BODY_ROLL_RATE_IGNORE | BODY_PITCH_RATE_IGNORE | BODY_YAW_RATE_IGNORE
q = q_sp
body rates = 0
thrust = thrust
```

PX4 在 Offboard 下跟踪姿态四元数和 thrust，内部姿态/角速度控制器再驱动电机模型。

### 9.5 加速度到机体系角速度，备用链路

若显式设置 `GUIDANCE_OUTPUT_MODE=accel_body_rate`、`PX4_COMMAND_MODE=mavlink_body_rate`，程序会先把 `a_control_I` 转到机体系：

```text
a_control_B = R_BI * a_control_I
```

再用小角度近似得到期望 roll/pitch：

```text
roll_sp  =  body_rate_roll_gain  * a_control_B.y / g
pitch_sp = -body_rate_pitch_gain * a_control_B.x / g
```

姿态误差通过比例环变成机体系角速度：

```text
p_cmd = K_att * (roll_sp  - roll)
q_cmd = K_att * (pitch_sp - pitch)
r_cmd = yaw_rate_cmd
```

再按最大 roll/pitch/yaw rate 限幅。MAVLink 发送仍使用 `SET_ATTITUDE_TARGET`，但 `type_mask` 忽略姿态四元数，只让 PX4 接收 `body_roll_rate/body_pitch_rate/body_yaw_rate` 和 thrust。

### 9.6 本报告中过载曲线的含义

- `需用过载 n_cmd_g`：由 PNG 的 `a_cmd` 直接换算，是导引层希望产生的过载。
- `实际过载 max g`：由拦截机真实速度差分估计，体现 PX4 和 AirSim 动力学真正实现出的机动。
- `速度指令差分 P95 g`：兼容旧速度输出模式的指标，本轮 `accel_attitude` 下主要作为参考，不代表直接发送给 PX4 的控制量。

因此，若 `n_cmd_g` 很平滑但实际过载不足，问题通常在姿态/推力响应、速度保持、限幅或视觉低帧率；若 `n_cmd_g` 本身突变，则应优先检查 LOS/KF、bbox 裁切、丢检外推和 frame guard 状态切换。

## 10. 结论

- TTC: 命中 `0/6`，命中距离 `-`，未命中 `50m, 60m, 70m, 80m, 90m, 100m`，检测帧比例 `67.3%`，有效导引帧比例 `62.4%`，平均检测 FPS `9.06`。
- VM: 命中 `0/6`，命中距离 `-`，未命中 `50m, 60m, 70m, 80m, 90m, 100m`，检测帧比例 `59.1%`，有效导引帧比例 `54.4%`，平均检测 FPS `8.70`。
- 本轮使用真实 YOLOv8 + ByteTrack，因此检测连续性和 GPU 推理速度会直接进入闭环；结果不能和 AirSim detect 函数的理想 bbox 直接等价比较。
- `accel_integral` 模式的 `n_cmd_g` 来自导引层 `a_cmd`，底层仍通过 PX4/AirSim 速度 setpoint 闭环；实际过载由真实速度差分估计，因此会同时受 PX4 响应、速度限幅和视觉帧率影响。
- `accel_body_rate` 模式下 `n_cmd_g` 仍表示纯 PNG 需用过载；实际发送给 PX4 的是 `SET_ATTITUDE_TARGET` 机体系 `p/q/r` 角速度和归一化 thrust，日志中的 `body_rate_control_accel_*` 额外包含沿 LOS 的速度保持加速度。
- `accel_attitude` 模式下 `n_cmd_g` 同样表示纯 PNG 需用过载；实际发送给 PX4 的是 `SET_ATTITUDE_TARGET` 姿态四元数和归一化 thrust，日志中的 `attitude_control_accel_*` 记录姿态指令生成前的合成加速度。
