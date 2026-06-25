# YOLO+KCF frame_centering tuned PX4 SITL TTC / V_m 50-100m 测试报告

## 1. 实验目的

按照此前已命中的 YOLO 案例配置，改用真正 PX4 SITL actor 场景，比较两种捷联视觉比例导引。本报告优先使用 `n_cmd_g` 作为需用过载；旧日志没有该字段时才回退到 `g_eval` 等效过载。

- `TTC` 组：`ttc_png`，TTC 只参与增益调度，并保留 LOS/Vm soft guidance。
- `VM` 组：`fixed_vm_png`，不使用 TTC，固定 `N * V_m` 导引增益。
- `accel_integral` 输出模式：导引律先计算 `a_cmd` / `n_cmd_g`，再按当前仿真步长积分为速度 setpoint；这不是直接向 PX4 发送加速度 setpoint。
- `accel_body_rate` 输出模式：导引律先计算 PNG 需用加速度，再转换为 PX4 `SET_ATTITUDE_TARGET` 机体系角速度 `p/q/r` 和 thrust；速度只作为沿 LOS 保速参考，不再把 PNG 横向修正直接加到速度指令上。
- `accel_attitude` 输出模式：导引律先计算 PNG 需用加速度，再转换为 PX4 `SET_ATTITUDE_TARGET` 姿态四元数和 thrust；速度只作为沿 LOS 保速参考。

与 terminal_v2 之前的 YOLO+ByteTrack frame_centering_tuned 基线相同参数，仅将检测源替换为 yolo_kcf。

## 2. 基准条件

|参数|值|
|---|---|
|stamp|`yolo_kcf_frame_centering_tuned_20260624_092746`|
|settings|`/home/linux/Documents/PNG/config/airsim_blocks_px4_actor_settings.json`|
|拦截机|`PX4 SITL / velocity_yaw_rate`|
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
|guidance output|`accel_integral`|
|max guidance accel|`15.0 m/s^2`|
|min speed ratio|`0.6`|
|thrust model|`airsim_generic_quad`, mass `1.0 kg`, max total thrust `16.717785072 N`|
|body-rate tilt / attitude P|`20.0 deg` / `4.0`|
|body-rate roll/pitch max rate|`60.0` / `60.0 deg/s`|
|body-rate thrust|min/hover/max `0.25` / `0.5865998371` / `0.95`|
|body-rate speed hold|gain `1.2`, max accel `6.0 m/s^2`, total limit `18.0 m/s^2`|
|attitude tilt / yaw lookahead|`25.0 deg` / `0.25 s`|
|attitude thrust|min/hover/max `0.25` / `0.5865998371` / `0.95`|
|attitude speed hold|gain `1.2`, max accel `6.0 m/s^2`, total limit `18.0 m/s^2`|
|LOS filter|`1`|
|LOS KF q lambda / lambda_dot|`0.0005` / `0.02`|
|LOS KF r / innovation gate|`0.008` / `0.75`|
|LOS terminal gate / delay|`1.2` / `0.18 s`|
|terminal image KF|predict `0.35 s`, reject `0.2 rad`, soft reject `0`|
|terminal image KF dynamics|accel noise `8.0 rad/s^2`, max rate `8.0 rad/s`|
|frame_guard|`True`|
|bbox noise|`0`|

## 3. 总览图

![summary](assets/YOLO_KCF_frame_centering_tuned_50_100测试报告/summary_yolo_kcf_frame_centering_tuned_20260624_092746.png)

## 4. 汇总表

|组别|命中数|命中距离m|未命中距离m|最小中心距离m|检测帧/总帧|有效帧/总帧|平均检测FPS|
|---|---:|---|---|---:|---:|---:|---:|
|TTC|2/6|50, 100|60, 70, 80, 90|1.939|1323/1515|1430/1515|7.87|
|VM|0/6|-|50, 60, 70, 80, 90, 100|3.007|1497/1712|1545/1712|7.81|

## 5. 明细表

|组别|距离m|碰撞|碰撞时间s|最小距离m|终点距离m|检测帧率|有效帧率|YOLO FPS|sim FPS|实际过载max g|速度指令差分P95 g|需用过载P95 g|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|TTC|50|1|18.21|2.086|2.086|99.2%|97.7%|8.18|7.52|0.66|0.79|1.11|
|VM|50|0|-|3.205|5.520|93.5%|97.2%|7.73|7.28|0.77|0.84|1.49|
|TTC|60|0|-|3.237|3.237|94.4%|98.8%|7.83|7.39|0.81|0.91|0.76|
|VM|60|0|-|3.007|3.007|92.9%|98.4%|7.91|7.43|0.82|1.03|0.95|
|TTC|70|0|-|4.838|6.021|92.9%|98.6%|7.79|7.36|0.91|0.93|0.87|
|VM|70|0|-|4.486|4.911|88.3%|86.8%|7.86|7.34|0.86|1.33|1.06|
|TTC|80|0|-|4.492|6.894|87.5%|97.1%|7.75|7.29|0.81|1.01|1.16|
|VM|80|0|-|3.074|3.074|95.6%|99.1%|7.84|7.39|0.84|0.92|0.71|
|TTC|90|0|-|4.917|7.370|79.6%|89.0%|7.68|7.25|0.92|1.46|1.08|
|VM|90|0|-|14.554|14.554|76.1%|81.3%|7.83|7.40|1.01|1.58|0.66|
|TTC|100|1|30.07|1.939|1.939|75.8%|85.8%|7.99|7.48|0.86|1.01|0.51|
|VM|100|0|-|15.673|15.673|81.9%|82.6%|7.70|7.29|0.99|1.35|1.29|

## 6. 分距离曲线

每个距离一张图，包含真实中心距离、bbox 面积、TTC 估计、实际过载/需用过载和 YOLO 检测 FPS。

![50m](assets/YOLO_KCF_frame_centering_tuned_50_100测试报告/yolo_sitl_ttc_vm_050m.png)
![60m](assets/YOLO_KCF_frame_centering_tuned_50_100测试报告/yolo_sitl_ttc_vm_060m.png)
![70m](assets/YOLO_KCF_frame_centering_tuned_50_100测试报告/yolo_sitl_ttc_vm_070m.png)
![80m](assets/YOLO_KCF_frame_centering_tuned_50_100测试报告/yolo_sitl_ttc_vm_080m.png)
![90m](assets/YOLO_KCF_frame_centering_tuned_50_100测试报告/yolo_sitl_ttc_vm_090m.png)
![100m](assets/YOLO_KCF_frame_centering_tuned_50_100测试报告/yolo_sitl_ttc_vm_100m.png)

## 7. LOS KF 与失败原因诊断

|组别|距离m|最近距离m|最近点状态|主要失败/降级原因|检测率|有效率|
|---|---:|---:|---|---|---:|---:|
|TTC|50|2.086|`valid`|valid:100, area_not_expanding:25, los_innovation_reject:3, ttc_out_of_range:2|99.2%|97.7%|
|VM|50|3.205|`valid`|valid:200, image_kf_predict:8, no_detection:6|93.5%|97.2%|
|TTC|60|3.237|`valid`|valid:149, area_not_expanding:85, image_kf_predict:11, ttc_out_of_range:4|94.4%|98.8%|
|VM|60|3.007|`image_kf_predict`|valid:235, image_kf_predict:14, no_detection:4|92.9%|98.4%|
|TTC|70|4.838|`valid`|valid:158, area_not_expanding:101, image_kf_predict:16, ttc_out_of_range:4|92.9%|98.6%|
|VM|70|4.486|`valid`|valid:219, los_innovation_reject:29, image_kf_predict:25, no_detection:8|88.3%|86.8%|
|TTC|80|4.492|`valid`|valid:147, area_not_expanding:115, image_kf_predict:30, ttc_out_of_range:10|87.5%|97.1%|
|VM|80|3.074|`valid`|valid:303, image_kf_predict:11, no_detection:3|95.6%|99.1%|
|TTC|90|4.917|`valid`|valid:129, area_not_expanding:111, image_kf_predict:33, no_detection:32|79.6%|89.0%|
|VM|90|14.554|`valid`|valid:215, image_kf_predict:50, los_innovation_reject:33, no_detection:28|76.1%|81.3%|
|TTC|100|1.939|`image_kf_predict`|valid:92, area_not_expanding:66, no_detection:31, image_kf_predict:22|75.8%|85.8%|
|VM|100|15.673|`valid`|valid:232, image_kf_predict:33, los_innovation_reject:31, no_detection:25|81.9%|82.6%|

- LOS KF 参数：`q_lambda=0.0005`、`q_lambda_dot=0.02`、`r=0.008`、`innovation_reject=0.75`、`terminal_reject=1.2`。
- 最近点处仍处于降级或无效状态的未命中工况：VM 60m:`image_kf_predict`。这些样本说明末端质量门、视觉外推和 bbox 裁切处理仍会影响命中窗口。
- 本轮平均实际过载峰值约 `0.85 g`，平均需用过载 P95 约 `0.97 g`。两者不是同一个量：`n_cmd_g` 是导引层需求，实际过载还受 PX4 姿态/推力限制、YOLO 约 9 FPS 采样和 frame centering 限速影响。

## 8. 相机光心真值影子测试诊断

![shadow_summary](assets/YOLO_KCF_frame_centering_tuned_50_100测试报告/shadow_summary_yolo_kcf_frame_centering_tuned_20260624_092746.png)

影子测试不参与导引，只用日志中的相机光心 `camera_world_*` 与目标真值位置离线计算经典 `N*Vc` 和固定 `N*Vm` PNG 理论需用过载，并和视觉 LOS、检测连续性对齐。

|组别|距离m|碰撞|最小距离m|最近点检测率|最近点无检测帧|视觉LOS误差P95|影子N*Vc P95 g|影子N*Vm P95 g|视觉需用P95 g|实际过载max g|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|TTC|50|1|2.086|100.0%|0/6|10.2|0.18|0.68|1.11|0.66|
|VM|50|0|3.205|84.6%|2/13|11.2|0.15|1.39|1.49|0.77|
|TTC|60|0|3.237|100.0%|0/7|7.5|0.13|1.00|0.76|0.81|
|VM|60|0|3.007|83.3%|1/6|13.6|0.12|1.06|0.95|0.82|
|TTC|70|0|4.838|53.8%|6/13|11.7|0.11|1.00|0.87|0.91|
|VM|70|0|4.486|30.8%|9/13|31.8|0.08|1.16|1.06|0.86|
|TTC|80|0|4.492|76.9%|3/13|18.3|0.09|1.11|1.16|0.81|
|VM|80|0|3.074|100.0%|0/6|12.6|0.09|0.24|0.71|0.84|
|TTC|90|0|4.917|46.2%|7/13|13.0|0.08|0.87|1.08|0.92|
|VM|90|0|14.554|100.0%|0/8|1.7|0.06|0.23|0.66|1.01|
|TTC|100|1|1.939|71.4%|2/7|54.4|0.08|0.21|0.51|0.86|
|VM|100|0|15.673|100.0%|0/8|0.7|0.08|0.26|1.29|0.99|

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

- TTC: 命中 `2/6`，命中距离 `50m, 100m`，未命中 `60m, 70m, 80m, 90m`，检测帧比例 `87.3%`，有效导引帧比例 `94.4%`，平均检测 FPS `7.87`。
- VM: 命中 `0/6`，命中距离 `-`，未命中 `50m, 60m, 70m, 80m, 90m, 100m`，检测帧比例 `87.4%`，有效导引帧比例 `90.2%`，平均检测 FPS `7.81`。
- 本轮使用真实 YOLOv8 + ByteTrack，因此检测连续性和 GPU 推理速度会直接进入闭环；结果不能和 AirSim detect 函数的理想 bbox 直接等价比较。
- `accel_integral` 模式的 `n_cmd_g` 来自导引层 `a_cmd`，底层仍通过 PX4/AirSim 速度 setpoint 闭环；实际过载由真实速度差分估计，因此会同时受 PX4 响应、速度限幅和视觉帧率影响。
- `accel_body_rate` 模式下 `n_cmd_g` 仍表示纯 PNG 需用过载；实际发送给 PX4 的是 `SET_ATTITUDE_TARGET` 机体系 `p/q/r` 角速度和归一化 thrust，日志中的 `body_rate_control_accel_*` 额外包含沿 LOS 的速度保持加速度。
- `accel_attitude` 模式下 `n_cmd_g` 同样表示纯 PNG 需用过载；实际发送给 PX4 的是 `SET_ATTITUDE_TARGET` 姿态四元数和归一化 thrust，日志中的 `attitude_control_accel_*` 记录姿态指令生成前的合成加速度。
