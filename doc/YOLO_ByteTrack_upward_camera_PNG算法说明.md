# YOLO+ByteTrack Upward-Camera PNG 拦截算法说明

## 1. 场景与闭环链路

当前拦截链路面向固定竖直上视相机：拦截机由 PX4/AirSim 控制，目标机作为 AirSim actor 或脚本目标运动。相机固定在拦截机机体上，推荐参数为 `camera_pitch_deg=-90`、`camera_x=0`、`fov_deg=120`、图像 `640x480`。导引闭环不使用 AirSim 真值，真值只用于离线评价、碰撞判定和曲线绘制。

完整数据流为：

```text
AirSim 图像 -> YOLO+ByteTrack 检测/关联 -> bbox 中心转 LOS
          -> LOS 6D Kalman 滤波 + TTC 面积膨胀估计
          -> TTC-PNG 或 V_m-PNG 生成加速度指令
          -> 上视相机居中 / frame guard / 末端外推
          -> PX4 MAVLink body-rate + thrust
```

主实现位于 `examples/run_airsim_strapdown_vision_png.py`，视觉检测器位于 `vision_guidance/yolo_bytetrack_detector.py`。

## 2. YOLO+ByteTrack 检测与目标选择

闭环检测源使用 `--detector-source yolo_bytetrack`。代码通过 Ultralytics `model.track(image_bgr, persist=True, tracker="bytetrack.yaml", conf=..., iou=..., imgsz=...)` 对每帧 AirSim 相机图像做检测与 ByteTrack 关联。当前推荐模型为 `vision_guidance/best.pt`，目标类别为 `--yolo-class-id 0`，常用置信度为 `--yolo-conf 0.05`，`--yolo-iou 0.7`。

目标选择顺序如下：

1. 若已有 `active_track_id`，优先沿用相同 ByteTrack ID。
2. 若启用 `--yolo-single-target-mode`，使用上一帧中心和面积做连续性选择，抑制短时 ID 抖动。
3. 若没有连续性候选，选择最高置信度的有 track ID 目标。
4. 若启用 `--yolo-allow-untracked-fallback`，ByteTrack 尚未给 ID 时可临时使用最高置信度框，记为 `track_id=-1`。

`--shadow-airsim-detect` 只做影子日志对比，不进入导引闭环。只有 `--detector-source airsim` 时，AirSim 内置 detect 才会直接控制闭环。

## 3. LOS 测量与滤波

YOLO bbox 中心 `(u, v)` 先由相机内参转换为相机坐标系射线，再通过固定相机外参 `R_BC` 和机体姿态 `R_IB` 转到惯性系，得到测量视线 `lambda_I`。上视相机的关键区别是光轴竖直向上，因此图像边缘误差主要反映机体系横向偏差，不能直接复用前向相机的末端策略。

LOS 滤波器 `LOSKalmanFilter6D` 使用 6 维状态：

```text
x = [lambda_x, lambda_y, lambda_z, lambda_dot_x, lambda_dot_y, lambda_dot_z]
```

预测模型为常速度 LOS，更新量为归一化后的 `lambda_I`。每次预测/更新后，代码会重新归一化 `lambda`，并将 `lambda_dot` 投影到垂直于 `lambda` 的平面，保证几何约束成立。LOS 角速度按 `omega_los = lambda x lambda_dot` 计算。

当前 upward-camera 成功实验常用滤波参数：

| 参数 | 值 | 作用 |
| --- | ---: | --- |
| `los_filter_process_lambda` | `5e-4` | LOS 方向过程噪声 |
| `los_filter_process_lambda_dot` | `2e-2` | LOS 变化率过程噪声 |
| `los_filter_measurement_noise` | `8e-3` | bbox 测量噪声 |
| `los_filter_innovation_reject` | `0.75` | 常规创新门限 |
| `los_filter_terminal_innovation_reject` | `1.20` | 末端放宽门限 |
| `los_delay_compensation_s` | `0.18` | 检测/控制延迟补偿 |

延迟补偿使用 `lambda_dot = omega_los x lambda` 对 LOS 前推，减少视觉帧率、录屏和仿真 clock 降低带来的相位滞后。

## 4. TTC 估计

TTC 由 `ScaleExpansionTTC` 根据 bbox 面积膨胀估计。设滤波后的框面积为 `A`，滑动窗口线性拟合得到 `A_dot`，则：

```text
TTC = 2 * A / A_dot
```

只有目标框在图像内、面积足够大、面积变化没有跳变、且 `A_dot > 0` 时 TTC 有效。若 bbox 被裁剪，TTC 会返回 `bbox_top_clipped`、`bbox_bottom_clipped`、`bbox_left_clipped` 或 `bbox_right_clipped` 等拒绝原因；这些原因会触发末端预测或盲区逻辑，而不是继续相信失真的面积膨胀。

在 `ttc_png` 模式下，TTC 不直接替代 LOS。TTC 只用于调度导引增益：`TTCGainSchedule` 在 `ttc_fast_s=1.0s` 附近给最大增益，在 `ttc_slow_s=6.0s` 之后降到最小增益。当前实验通常开启 `--ttc-soft-guidance`：TTC 无效时仍使用 VM 风格的 LOS 导引；TTC 有效时将 `N*V_m` 按 TTC schedule 缩放，缩放下限由 `ttc_soft_min_gain_scale=0.55` 控制。

## 5. TTC-PNG 与 V_m-PNG

两种导引律共用同一个 LOS 和控制输出，只区别在导引增益来源。

### TTC-PNG

`--guidance-law ttc_png` 根据 TTC schedule 得到增益。若 `ttc_soft_guidance=false`，代码使用纯 TTC 增益，并在 TTC 无效时拒绝导引或走有限 fallback。当前 upward-camera 推荐配置开启 soft guidance，因此实际闭环为：

```text
gain = N * V_m                    # TTC 无效时
gain = N * V_m * schedule(TTC)    # TTC 有效时，带最小缩放
a_png = gain * (omega_los x lambda_I)
```

这样可以避免 YOLO 面积抖动或框裁剪造成的 TTC 瞬断，同时保留近距时的增益调度。

### V_m-PNG

`--guidance-law fixed_vm_png` 完全忽略 TTC，只使用固定闭合速度尺度：

```text
V_m = speed_ratio * intruder_speed
gain = N * V_m
a_png = gain * (omega_los x lambda_I)
```

当前常用 `navigation_constant=3`、`intruder_speed=5m/s`、`speed_ratio=2`，因此 `V_m=10m/s`，固定增益 `N*V_m=30`。VM 模式更稳定，适合检查 LOS/控制链路本身；TTC 模式在近距可更积极，但依赖 bbox 面积趋势质量。

两种模式的加速度都会被 `max_guidance_accel_mps2` 限幅，当前 upward-camera 成功配置常用 `20m/s^2`。

## 6. 上视相机保目标策略

上视相机不是“朝前看目标”，而是通过机体横向运动和姿态控制把目标保持在上方视场内。当前链路有三层保目标机制。

第一层是 `upward_centering`。它把 `lambda_I` 转到机体系，取 `lambda_B[:2]` 作为横向居中误差，并生成机体系 XY 加速度：

```text
a_center_B = upward_centering_gain * [lambda_B_x, lambda_B_y, 0]
```

当前常用 `upward_centering_gain=8.0`、`upward_centering_max_accel_mps2=4.0`。

第二层是 `frame_guard`。当图像误差、bbox 面积或 TTC 表明目标接近边缘/末端时，调低速度上限，缩放横向/垂向速度，并增大 yaw-rate 响应。当前成功配置常用 `enter_error_ratio=0.70`、`exit_error_ratio=0.45`、`area_mid_ratio=0.004`、`area_ratio=0.018`、`ttc_mid_s=1.6`、`ttc_terminal_s=0.7`。

第三层是 `frame_centering`。它是更硬的视场保持状态机，状态包括 `tracking`、`frame_centering`、`terminal_capture` 和 `loss_hold`。当前成功配置常用 `enter_error_ratio=0.62`、`terminal_error_ratio=0.85`、`area_ratio=0.02`、`loss_hold_s=1.1`。在 `terminal_capture` 或短时丢失时，代码会压低横向速度，让拦截机继续沿 LOS 方向推进，降低目标从上视视场滑出的概率。

## 7. 末端盲区外推

末端外推由两个互补模块组成，不能把前向相机策略直接套到上视相机。

### 图像平面 KF 短时预测

`TerminalImageKF` 跟踪图像角误差：

```text
x = [theta_x, theta_y, theta_dot_x, theta_dot_y]
```

当 bbox 被裁剪、短时无检测或图像创新过大时，KF 可以在 `max_predict_s` 内继续输出预测中心。主循环再用预测中心重建 `lambda_I` 和 `omega_los`，导引模式标记为 `*_kf_predict`。当前配置常用：

| 参数 | 值 |
| --- | ---: |
| `terminal_image_kf_max_predict_s` | `1.0` |
| `terminal_image_kf_meas_noise_rad` | `0.006` |
| `terminal_image_kf_innovation_reject_rad` | `0.35` |
| `terminal_image_kf_max_rate_rad_s` | `12.0` |
| `terminal_image_kf_soft_reject_predict` | `true` |

### TerminalExtrapolator 盲区状态机

`TerminalExtrapolator` 状态包括 `Waiting`、`Tracking`、`TerminalVisual`、`BlindPush`、`LossHold`、`Complete`、`AbortHold`。进入末端的依据包括 bbox 面积比例、soft image-KF 测量和有效导引历史。切入盲推的原因包括 bbox 裁剪、面积过大、目标丢失和安全门控。

上视加速度输出模式下，默认策略是：

| 开关 | 当前策略 | 原因 |
| --- | --- | --- |
| `terminal_velocity_blind_push` | `false` | 不直接用速度盲推覆盖闭环 |
| `terminal_accel_hold` | `true` | 保持近期 PNG 加速度更符合 body-rate 输出 |
| `terminal_blind_requires_visual_loss` | `true` | bbox 裁剪但 KF 仍可用时优先视觉预测 |
| `terminal_clipped_los_kf_predict` | `true` | 裁剪框先走 image-KF LOS，不立即盲推 |

当前成功参数为 `terminal_enter_area_ratio=0.008`、`terminal_soft_enter_area_ratio=0.004`、`terminal_cutoff_area_ratio=0.03`、`terminal_cutoff_miss_count=1`、`terminal_blind_duration_s=1.0`。盲区期间，近期加速度按 `terminal_accel_hold_window_s=0.35` 求均值，并按 `terminal_accel_decay_tau_s=0.6` 指数衰减，限幅 `terminal_accel_hold_max_mps2=20`。yaw-rate 也可使用短窗口均值衰减保持，减少末端图像丢失后的姿态突变。

## 8. PX4 body-rate 输出

当前 upward-camera 推荐闭环使用：

```text
guidance_output_mode = accel_body_rate
px4_command_mode = mavlink_body_rate
```

主循环先生成 `a_png`，再叠加速度保持加速度和上视居中加速度，得到总加速度命令。`_body_rate_command_from_accel` 将惯性系加速度转到机体系，计算期望 roll/pitch、body rates 和归一化 thrust，并通过 MAVLink offboard `send_body_rate(body_rates_rad_s, thrust)` 发送给 PX4。当前成功配置为 `body_rate_control_profile=legacy`，`body_rate_total_accel_limit_mps2=28`，最大倾角、推力上下限和速度保持参数由运行脚本/命令行传入。

这种输出方式比直接速度命令更接近真实无人机部署：导引输出是过载/加速度，飞控负责姿态角速度和推力执行。

## 9. 推荐运行参数快照

最近 upward-camera 成功链路的代表性参数来自：

```text
logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_visual_s_maneuver_35_clock0p5_20260715_063715_r35_h30_meta.json
```

关键参数如下：

```text
detector_source=yolo_bytetrack
yolo_model=vision_guidance/best.pt
yolo_class_id=0
yolo_conf=0.05
yolo_tracker=bytetrack.yaml
yolo_allow_untracked_fallback=true
yolo_single_target_mode=true

guidance_output_mode=accel_body_rate
px4_command_mode=mavlink_body_rate
camera_pitch_deg=-90
camera_x=0
fov_deg=120
speed_ratio=2
navigation_constant=3
intruder_speed=5

los_filter_process_lambda=5e-4
los_filter_process_lambda_dot=2e-2
los_filter_measurement_noise=8e-3
los_filter_innovation_reject=0.75
los_delay_compensation_s=0.18

terminal_image_kf_max_predict_s=1.0
terminal_accel_hold=true
terminal_velocity_blind_push=false
terminal_blind_requires_visual_loss=true
terminal_clipped_los_kf_predict=true
upward_centering=true
```

多 blocks 同时运行时，批处理脚本默认使用 `AIRSIM_RPC_HOST=127.0.0.2`，避免与其他 AirSim/PX4 实验争抢默认 RPC 地址。

## 10. 关键源码索引

| 功能 | 文件 |
| --- | --- |
| YOLO+ByteTrack 检测、单目标选择 | `vision_guidance/yolo_bytetrack_detector.py` |
| LOS 6D Kalman 滤波 | `vision_guidance/los_filter.py` |
| bbox 面积膨胀 TTC | `vision_guidance/ttc.py` |
| TTC 增益调度 | `vision_guidance/png_eval.py` |
| 图像平面末端 KF | `vision_guidance/terminal_image_kf.py` |
| 末端盲区状态机 | `vision_guidance/terminal_extrapolation.py` |
| 上视相机 PNG/PX4 主闭环 | `examples/run_airsim_strapdown_vision_png.py` |
| 批量 TTC/VM 运行入口 | `run_yolo_sitl_ttc_vm_batch.sh` |

