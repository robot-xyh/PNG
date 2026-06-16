# 捷联视觉 PNG ClockSpeed 0.2 无LOS滤波距离与过载测试报告

## 1. 测试目的

本报告整理 AirSim Blocks 在 `ClockSpeed=0.2` 下的捷联视觉 PNG 批量仿真结果。测试距离覆盖 `30-160m`，每次飞行记录真值距离、检测可用性、AirSim 碰撞结果、视觉速度指令等效过载、真值 PNG 理论需用过载、实际过载和仿真帧率。

成功判据仍采用 AirSim 双机碰撞检测；真值位置、最小距离和过载只用于离线评价，不参与捷联视觉 PNG 内部导引。真值 PNG 理论需用过载是假设“已知真实相对位置和速度”后事后计算得到，用于和视觉控制输出进行对照。

## 2. 测试条件

- 测试对象：`examples/run_airsim_strapdown_vision_png.py`
- 批量脚本：`examples/batch_strapdown_accuracy.py`
- AirSim 配置：`config/airsim_blocks_settings.json`
- 仿真时钟：`ClockSpeed=0.2`
- 显示模式：`ViewMode=NoDisplay`
- LOS Kalman 滤波：关闭；`lambda_x/lambda_y/lambda_z` 直接使用时间对齐后的原始测量 LOS
- 相机视场角：`120 deg`
- 拦截机起始高度：`50 m`
- 入侵机高度差：`20 m`
- 入侵机速度：`5.0 m/s`
- 速度比：`2.0`
- 侧向偏置：`-20 m`

运行命令：

```bash
python3 examples/batch_strapdown_accuracy.py --ranges 30 40 50 60 70 80 90 100 110 120 130 140 150 160 --altitude-offsets 20 --duration-s 28 --intruder-speed 5 --speed-ratio 2 --rate-hz 20 --print-every-n 0 --prefix strapdown_clock0p2_no_los_filter_$(date +%Y%m%d_%H%M%S) --trajectory-dir logs/strapdown_accuracy -- --no-los-filter
```

## 3. 结果总览

![总体结果](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_summary.png)

### 碰撞与最小距离简表

| 初始水平距离 | 是否碰撞 | 碰撞时间 | 最小距离 |
|---:|---:|---:|---:|
| 30m | 否 | - | 5.798m |
| 40m | 是 | 6.85s | 1.024m |
| 50m | 否 | - | 0.874m |
| 60m | 否 | - | 0.765m |
| 70m | 否 | - | 0.701m |
| 80m | 否 | - | 0.630m |
| 90m | 否 | - | 0.598m |
| 100m | 否 | - | 0.556m |
| 110m | 否 | - | 0.556m |
| 120m | 否 | - | 0.529m |
| 130m | 否 | - | 0.535m |
| 140m | 是 | 21.06s | 0.489m |
| 150m | 否 | - | 0.516m |
| 160m | 否 | - | 0.500m |

| 初始水平距离 | 是否碰撞 | 碰撞时间 | 最小距离 | 检测帧 | 有效帧 | 平均仿真FPS | 最大实际过载 | 视觉指令最大等效过载 | 视觉指令P95等效过载 | 理论最大需用过载 | 理论P95需用过载 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 30m | 否 | - | 5.798m | 240/2790 | 184/2790 | 99.69 | 0.99g | 66.30g | 0.42g | 1.89g | 1.05g |
| 40m | 是 | 6.85s | 1.024m | 629/684 | 599/684 | 99.72 | 18.95g | 89.51g | 7.59g | 3.23g | 1.78g |
| 50m | 否 | - | 0.874m | 736/2790 | 690/2790 | 99.70 | 33.60g | 43.83g | 1.35g | 2.13g | 1.05g |
| 60m | 否 | - | 0.765m | 861/2790 | 836/2790 | 99.70 | 0.92g | 59.51g | 0.90g | 3.01g | 0.80g |
| 70m | 否 | - | 0.701m | 982/2790 | 958/2790 | 99.70 | 0.92g | 63.77g | 1.00g | 3.48g | 0.64g |
| 80m | 否 | - | 0.630m | 1138/2790 | 1112/2790 | 99.71 | 0.92g | 67.38g | 0.98g | 3.95g | 0.54g |
| 90m | 否 | - | 0.598m | 1272/2790 | 1247/2790 | 99.70 | 0.92g | 75.01g | 1.10g | 4.18g | 0.46g |
| 100m | 否 | - | 0.556m | 1486/2790 | 1463/2790 | 99.71 | 0.92g | 67.52g | 0.95g | 4.30g | 0.37g |
| 110m | 否 | - | 0.556m | 1649/2790 | 1625/2790 | 99.71 | 0.93g | 64.23g | 0.87g | 4.49g | 0.32g |
| 120m | 否 | - | 0.529m | 1786/2790 | 1761/2790 | 99.69 | 0.92g | 71.25g | 0.85g | 4.76g | 0.29g |
| 130m | 否 | - | 0.535m | 1929/2790 | 1905/2790 | 99.71 | 0.93g | 68.86g | 0.78g | 4.73g | 0.27g |
| 140m | 是 | 21.06s | 0.489m | 2062/2099 | 2037/2099 | 99.70 | 50.48g | 73.56g | 0.78g | 5.08g | 0.25g |
| 150m | 否 | - | 0.516m | 2203/2790 | 2179/2790 | 99.71 | 0.93g | 78.50g | 0.72g | 4.87g | 0.23g |
| 160m | 否 | - | 0.500m | 2328/2790 | 2304/2790 | 99.70 | 0.92g | 78.06g | 0.59g | 4.97g | 0.21g |

## 4. 距离与轨迹

![距离曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_range_curves.png)

![俯视轨迹](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_topdown.png)

## 5. 需用过载、理论过载、实际过载与帧率

![过载和帧率](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_fps.png)

本报告区分三类过载：

- 视觉速度指令等效过载：由速度指令变化量离线计算，`n_cmd = ||Δv_cmd / Δt_sim|| / g`。因为当前捷联程序向 AirSim 发送速度设定值，所以该指标用于评价视觉 PNG、LossHold 和末端外推状态机对飞控提出的瞬时速度变化需求。
- 真值 PNG 理论需用过载：假设已知目标真实相对位置和速度，用经典三维 PNG 公式计算，`n_theory = ||N * Vc * lambda_dot|| / g`，本报告使用 `N=3.0`。该指标不使用视觉检测框，也不使用视觉状态机输出，用于表示同一几何轨迹下理想比例导引本身需要的机动强度。
- 实际过载：由拦截机真值速度有限差分计算，`n_act = ||Δv_truth / Δt_sim|| / g`。`load_factor_g` 也被记录，但 SimpleFlight 在部分版本中真值线加速度字段可能长期为零或更新不稳定，所以表格和曲线优先使用 `load_factor_fd_g`。

视觉速度指令最大等效过载用于暴露 BlindPush 进入/退出、LossHold、Complete 和速度指令重置带来的单帧尖峰；P95 更适合评价大部分飞行过程中的持续控制需求。真值 PNG 理论过载则用于回答“如果目标位置和速度完全已知，比例导引理论上需要多大过载”。

## 6. 各工况曲线汇总

本节按初始水平距离组织曲线。同一个工况下依次放置过载曲线和视线角曲线，便于直接对照该工况中的导引需求、无人机响应和 LOS 偏差。

过载曲线包含三条过载曲线；图下方的距离曲线只用于标注该工况的交会进程，不是轨迹图：

- `image-PNG theoretical/equivalent load`：捷联图像 PNG 输出速度指令的一阶差分等效过载，代表图像 PNG 对飞控提出的理论机动需求。
- `strapdown UAV actual load`：捷联视觉程序实际驱动无人机后，由无人机真值速度有限差分得到的实际过载。
- `shadow truth-position theoretical load`：影子测试曲线，在同一捷联实验轨迹上假设已知入侵机真实位置和速度，离线计算经典 PNG 理论需用过载。

视线角曲线包含两条 LOS 角度曲线和误差曲线：

- 捷联图像 LOS：来自捷联视觉程序日志中的 `lambda_x/lambda_y/lambda_z`，代表图像 PNG 实际用于导引的惯性系视线方向。
- 影子真值 LOS：使用同一时刻 `intruder_position - camera_world_position` 计算，即从拦截机相机光心指向入侵机原点，只用于离线评价，不参与捷联导引。新日志直接读取 `camera_world_x/y/z`；旧日志没有相机世界坐标时，用 `interceptor_position + yaw(camera_x,camera_y,camera_z)` 回算固定相机光心，因此仍能扣除主要的相机安装偏移。

本报告关闭 6D LOS Kalman 滤波。捷联图像 LOS 中的 `lambda_x/lambda_y/lambda_z` 等同于姿态去旋转后的原始测量 LOS；`omega_x/omega_y/omega_z` 由相邻原始 LOS 单位向量按仿真时间有限差分得到，并投影到垂直 LOS 的切平面。日志额外保留 `lambda_raw_*`、`omega_raw_*`、`los_dt_s` 和 `los_angle_step_deg`，用于检查无噪声仿真下原始视线角速度是否仍存在离散采样尖峰。

方位角按惯性系水平面 `atan2(y, x)` 计算，俯仰角按 NED 坐标中的 `atan2(-z, horizontal)` 计算。`LOS夹角误差` 是两条三维单位视线向量的夹角。表格中的平均值和 P95 只统计 `valid=1` 的有效导引帧；全程最大值保留失锁、裁切和穿越后的极端偏离，作为异常诊断参考。

| 初始水平距离 | 有效LOS帧 | 有效平均LOS夹角误差 | 有效P95 LOS夹角误差 | 有效平均方位误差 | 有效P95方位误差 | 有效平均俯仰误差 | 有效P95俯仰误差 | 全程最大LOS夹角误差 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 30m | 184/2790 | 0.38deg | 1.00deg | 0.49deg | 1.36deg | 0.16deg | 0.45deg | 150.76deg |
| 40m | 599/684 | 1.90deg | 6.68deg | 1.95deg | 7.87deg | 0.66deg | 1.61deg | 17.11deg |
| 50m | 690/2790 | 1.63deg | 6.21deg | 1.50deg | 5.63deg | 0.64deg | 3.30deg | 156.86deg |
| 60m | 836/2790 | 1.04deg | 4.25deg | 0.96deg | 4.02deg | 0.30deg | 1.56deg | 164.29deg |
| 70m | 958/2790 | 0.74deg | 3.04deg | 0.68deg | 2.84deg | 0.21deg | 1.13deg | 168.87deg |
| 80m | 1112/2790 | 0.53deg | 2.04deg | 0.45deg | 1.94deg | 0.18deg | 1.04deg | 175.69deg |
| 90m | 1247/2790 | 0.42deg | 1.67deg | 0.34deg | 1.54deg | 0.16deg | 0.86deg | 174.69deg |
| 100m | 1463/2790 | 0.30deg | 1.24deg | 0.24deg | 1.11deg | 0.12deg | 0.57deg | 174.69deg |
| 110m | 1625/2790 | 0.25deg | 1.06deg | 0.20deg | 0.88deg | 0.11deg | 0.49deg | 174.68deg |
| 120m | 1761/2790 | 0.21deg | 0.93deg | 0.16deg | 0.76deg | 0.10deg | 0.45deg | 171.23deg |
| 130m | 1905/2790 | 0.19deg | 0.83deg | 0.14deg | 0.64deg | 0.10deg | 0.40deg | 174.48deg |
| 140m | 2037/2099 | 0.17deg | 0.74deg | 0.12deg | 0.61deg | 0.09deg | 0.37deg | 167.14deg |
| 150m | 2179/2790 | 0.15deg | 0.67deg | 0.11deg | 0.55deg | 0.08deg | 0.35deg | 176.85deg |
| 160m | 2304/2790 | 0.14deg | 0.62deg | 0.10deg | 0.48deg | 0.08deg | 0.33deg | 168.87deg |

### 原始 LOS 差分诊断

本节直接读取日志中的 `omega_raw_norm_rad_s`、`los_angle_step_deg` 和 `los_dt_s`。在关闭 6D LOS Kalman 滤波后，`omega_raw_norm_rad_s` 就是图像 PNG 实际使用的 LOS 角速度模长，因此它能直接暴露有限差分在末端穿越、裁切、失锁和重捕获时产生的尖峰。

| 初始水平距离 | LOS来源帧数 | 最大原始LOS角速度 | P95原始LOS角速度 | 最大有效LOS角速度 | 最大单帧LOS角步长 | P95单帧LOS角步长 | 中位LOS dt |
|---:|---|---:|---:|---:|---:|---:|---:|
| 30m | none:2550, raw_fd:240 | 1.45rad/s | 0.27rad/s | 0.71rad/s | 4.77deg | 0.50deg | 10.2ms |
| 40m | none:55, raw_fd:629 | 4.19rad/s | 1.32rad/s | 4.19rad/s | 2.45deg | 0.76deg | 10.2ms |
| 50m | none:2054, raw_fd:736 | 50.73rad/s | 0.75rad/s | 4.57rad/s | 29.14deg | 0.79deg | 10.2ms |
| 60m | none:1929, raw_fd:861 | 62.02rad/s | 0.53rad/s | 4.29rad/s | 36.55deg | 0.34deg | 10.2ms |
| 70m | none:1808, raw_fd:982 | 64.51rad/s | 0.42rad/s | 4.52rad/s | 41.15deg | 0.26deg | 10.2ms |
| 80m | none:1652, raw_fd:1138 | 64.98rad/s | 0.35rad/s | 5.11rad/s | 41.52deg | 0.22deg | 10.2ms |
| 90m | none:1518, raw_fd:1272 | 64.17rad/s | 0.30rad/s | 5.79rad/s | 40.89deg | 0.18deg | 10.2ms |
| 100m | none:1304, raw_fd:1486 | 63.45rad/s | 0.23rad/s | 3.85rad/s | 40.34deg | 0.14deg | 10.2ms |
| 110m | none:1141, raw_fd:1649 | 66.97rad/s | 0.20rad/s | 3.98rad/s | 40.01deg | 0.12deg | 10.2ms |
| 120m | none:1004, raw_fd:1786 | 68.33rad/s | 0.18rad/s | 4.15rad/s | 41.00deg | 0.11deg | 10.2ms |
| 130m | none:861, raw_fd:1929 | 62.98rad/s | 0.16rad/s | 3.53rad/s | 39.98deg | 0.10deg | 10.2ms |
| 140m | none:37, raw_fd:2062 | 67.73rad/s | 0.15rad/s | 4.24rad/s | 40.56deg | 0.09deg | 10.2ms |
| 150m | none:587, raw_fd:2203 | 67.49rad/s | 0.14rad/s | 4.71rad/s | 40.39deg | 0.08deg | 10.2ms |
| 160m | none:462, raw_fd:2328 | 63.50rad/s | 0.13rad/s | 5.47rad/s | 40.37deg | 0.08deg | 10.2ms |

可以看到，即使 AirSim detection 给出的是理论检测框，原始 LOS 差分仍会在末端产生很高的角速度尖峰。根本原因不是检测框抖动，而是几何本身在近距穿越时 LOS 方向变化极快；再叠加固定相机出视场、bbox 裁切、`valid/invalid` 状态切换以及仿真离散采样，有限差分会把单帧角步长直接放大成 `rad/s` 量级的角速度脉冲。6D LOS Kalman 滤波的作用之一正是限制这种离散微分噪声，但代价是会引入相位滞后。


### 30m 工况

过载曲线：

![30m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_030m.png)

视线角曲线：

![30m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_030m.png)

### 40m 工况

过载曲线：

![40m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_040m.png)

视线角曲线：

![40m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_040m.png)

### 50m 工况

过载曲线：

![50m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_050m.png)

视线角曲线：

![50m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_050m.png)

### 60m 工况

过载曲线：

![60m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_060m.png)

视线角曲线：

![60m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_060m.png)

### 70m 工况

过载曲线：

![70m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_070m.png)

视线角曲线：

![70m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_070m.png)

### 80m 工况

过载曲线：

![80m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_080m.png)

视线角曲线：

![80m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_080m.png)

### 90m 工况

过载曲线：

![90m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_090m.png)

视线角曲线：

![90m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_090m.png)

### 100m 工况

过载曲线：

![100m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_100m.png)

视线角曲线：

![100m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_100m.png)

### 110m 工况

过载曲线：

![110m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_110m.png)

视线角曲线：

![110m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_110m.png)

### 120m 工况

过载曲线：

![120m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_120m.png)

视线角曲线：

![120m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_120m.png)

### 130m 工况

过载曲线：

![130m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_130m.png)

视线角曲线：

![130m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_130m.png)

### 140m 工况

过载曲线：

![140m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_140m.png)

视线角曲线：

![140m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_140m.png)

### 150m 工况

过载曲线：

![150m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_150m.png)

视线角曲线：

![150m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_150m.png)

### 160m 工况

过载曲线：

![160m 过载曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_load_compare_160m.png)

视线角曲线：

![160m 视线角曲线](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_los_compare_160m.png)


## 7. 视觉速度指令等效过载尖峰原因分析

本批日志中，视觉速度指令最大等效过载明显高于真值 PNG 理论需用过载。真值 PNG 理论最大值约为 `1.89g - 5.08g`，而视觉速度指令最大等效过载达到 `43.83g - 89.51g`。这说明高尖峰主要不是比例导引理论本身造成的，而是少数离散帧上的速度指令阶跃造成的。

由于视觉速度指令等效过载按 `n_cmd = ||Δv_cmd / Δt_sim|| / g` 计算，在 `ClockSpeed=0.2` 的日志中相邻仿真时间步约为 `0.01s`，即使 `v_cmd` 只发生 `0.5m/s` 量级变化，也会被换算为约 `5g` 的瞬时等效过载；如果状态切换导致 `v_cmd` 一帧内变化 `2-5m/s`，表格中的最大值就会达到几十 g。

本批测试使用 AirSim 内置 detection 函数输出理论检测框，因此检测框本身不代表真实 YOLO 噪声模型。报告中的 `detected=0` 主要表示固定相机几何下目标离开视场、目标框被严重裁切、穿越后相机几何失效，或 AirSim detection 在该帧没有返回匹配目标，不应简单理解为神经网络随机漏检。

对于关闭 LOS 滤波的无噪声仿真，速度指令尖峰还会来自原始 LOS 有限差分本身：近距穿越时 LOS 单位向量的单帧角步长会快速增大，`omega_LOS = lambda x lambda_dot` 会把这种角步长除以约 `10ms` 的仿真采样间隔，形成几十 `rad/s` 的瞬时角速度。也就是说，检测框不抖并不等于微分后的 LOS 角速度没有尖峰；滤波关闭后，尖峰会更直接地进入 `g_eval` 和速度指令。

从日志抽查看，尖峰主要集中在以下场景：

1. 初始接管阶跃。部分工况第 0 帧仍是默认前向速度，例如 `[10, 0, 0]`，第 1 帧进入视觉导引后变成带横向和垂向分量的速度，例如 `[8.29, -3.96, -3.95]`。两帧相隔约 `0.009s`，因此会出现 `60g` 量级的速度指令等效过载。
2. `Tracking / LossHold / BlindPush / Complete` 之间切换。典型情况是 `ttc_png -> invalid`、`invalid -> ttc_png`、`blind_push -> invalid` 或 `invalid -> blind_push`，上层速度指令从视觉 PNG 输出、末端盲推输出和保持输出之间切换，造成 `v_cmd` 不连续。
3. 固定相机末端出视场和严重裁切。重捕获或切换到 LossHold/BlindPush 时，速度指令容易发生单帧台阶。
4. `BlindPush` 结束后的回退。部分工况出现 `BlindPush -> Complete -> invalid` 后又因末端几何状态重新进入 `BlindPush` 的往复过程，横向速度和垂直速度分量会在短时间内反复切换。
5. 垂向速度限幅和末端偏置切换。捷联系统为了补偿高度差加入了垂直速度项，末端裁切后该项可能从正常限幅值跳到保持/盲推状态的历史值，`v_cmd_z` 的突变会显著抬高 `n_cmd`。

因此，视觉速度指令最大等效过载应被视为“指令连续性诊断指标”，不应直接等同于飞机真实承受的机体过载。后续优化应优先在上层指令输出端加入连续化处理：对 `v_cmd` 做 slew-rate limiter；初始接管阶段用 `0.2-0.5s` 淡入视觉修正；`LossHold` 保留上一帧完整速度并指数衰减；`BlindPush` 退出时平滑过渡到默认/重捕获控制；检测重获后用 `0.1-0.2s` 淡入 `ttc_png` 修正量。

## 8. 实际捷联过载偏大的原因

实际过载来自拦截机真值速度有限差分，定义为 `n_act = ||Δv_truth / Δt_sim|| / g`。多数距离工况的最大实际过载约为 `0.9g`，但 40m 和 50m 工况出现了 `20g+` 的离散尖峰。结合日志看，原因主要有三类：

1. 碰撞/近距接触导致速度状态不连续。40m 工况的最大实际过载出现在碰撞帧附近，最小距离约 `1.03m`，AirSim 碰撞体接触和物理求解会让真值速度在有限差分中出现尖峰。
2. AirSim SimpleFlight / RPC 状态离散更新造成有限差分放大。实际过载不是飞控内部连续加速度，而是用相邻日志帧的真值速度差计算。若 AirSim 在某几帧对位置或速度做了离散修正，相邻 `0.01s` 的差分会把这种修正放大成十几到几十 g。
3. 速度控制模式与物理状态不同步。脚本发送的是 `moveByVelocityAsync` 速度设定值，SimpleFlight 会用内部控制器追踪速度。若上一段命令、碰撞接触、姿态/高度控制或位置修正造成速度状态突然变化，有限差分实际过载会出现孤立尖峰，即使当前 `v_cmd` 已保持不变。

因此，实际过载中的 `20g+` 峰值更适合作为“仿真物理/离散采样异常点”排查指标，而不是直接等同于真实穿越机可承受或实际产生的持续机动过载。评价持续机动强度时，应优先同时查看 P95、峰值所在帧、是否碰撞、是否近距穿越以及该帧前后的速度状态。

## 9. 末端误差

![末端误差](assets/捷联ClockSpeed0p2无LOS滤波距离过载测试报告/strapdown_clock0p2_no_los_filter_terminal_errors.png)

| 初始水平距离 | 是否碰撞 | 最小距离时刻 | 最小距离 | 水平残差 | 垂直残差 | 像面误差 x/y | 末端状态 | 原因 |
|---:|---:|---:|---:|---:|---:|---:|---|---|
| 30m | 否 | 4.03s | 5.798m | 5.068m | -2.816m | 0.0/0.0px | LossHold | no_detection |
| 40m | 是 | 6.85s | 1.024m | 1.020m | 0.098m | 18.0/86.2px | TerminalVisual |  |
| 50m | 否 | 7.52s | 0.874m | 0.568m | -0.664m | 0.0/-117.0px | BlindPush | bbox_clipped |
| 60m | 否 | 8.54s | 0.765m | 0.314m | -0.697m | -99.6/-126.6px | BlindPush | bbox_area_large |
| 70m | 否 | 9.92s | 0.701m | 0.173m | -0.680m | -87.9/-118.3px | BlindPush | bbox_clipped |
| 80m | 否 | 11.39s | 0.630m | 0.063m | -0.627m | -90.3/-108.7px | BlindPush | bbox_clipped |
| 90m | 否 | 12.79s | 0.598m | 0.055m | -0.596m | -85.0/-100.0px | BlindPush | bbox_clipped |
| 100m | 否 | 15.00s | 0.556m | 0.044m | -0.554m | -71.5/-90.1px | BlindPush | bbox_clipped |
| 110m | 否 | 16.64s | 0.556m | 0.040m | -0.554m | -70.2/-88.5px | BlindPush | bbox_clipped |
| 120m | 否 | 18.08s | 0.529m | 0.066m | -0.525m | -79.5/-84.3px | BlindPush | bbox_clipped |
| 130m | 否 | 19.50s | 0.535m | 0.056m | -0.533m | -59.0/-80.6px | BlindPush | bbox_clipped |
| 140m | 是 | 20.93s | 0.489m | 0.106m | -0.477m | -69.0/-62.9px | BlindPush | bbox_clipped |
| 150m | 否 | 22.34s | 0.516m | 0.067m | -0.512m | -70.3/-80.7px | BlindPush | bbox_clipped |
| 160m | 否 | 23.77s | 0.500m | 0.083m | -0.493m | -64.2/-76.4px | BlindPush | bbox_clipped |

## 10. 结论

- 本批共 `14` 组，AirSim 碰撞命中 `2` 组。
- 最小距离范围：`0.489m - 5.798m`。
- 最大有限差分过载范围：`0.92g - 50.48g`。
- 视觉速度指令最大等效过载范围：`43.83g - 89.51g`。
- 视觉速度指令 P95 等效过载范围：`0.42g - 7.59g`。
- 真值 PNG 理论最大需用过载范围：`1.89g - 5.08g`。
- 真值 PNG 理论 P95 需用过载范围：`0.21g - 1.78g`。
- 平均仿真采样 FPS 范围：`99.69Hz - 99.72Hz`。

在 `ClockSpeed=0.2` 下，仿真以更慢的时钟推进，日志中的 `avg_wall_fps` 反映 Python 控制循环实际刷新率，`avg_sim_sample_fps` 反映 AirSim 状态时间戳推进后的采样频率。两者需要一起看：如果墙钟 FPS 正常但仿真 FPS 较低，说明仿真时钟确实被减速；如果墙钟 FPS 也明显下降，则要优先检查渲染、检测调用和 RPC 延迟。

## 11. 日志文件

- 总汇总：`logs/strapdown_accuracy/strapdown_clock0p2_no_los_filter_all_summary.csv`
自动纳入的批次汇总：
  - `logs/strapdown_accuracy/strapdown_clock0p2_no_los_filter_20260616_060330_summary.csv`

- 理论需用过载汇总：`logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_summary.csv`
- 理论需用过载逐帧文件：
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r100_h20.csv`
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r110_h20.csv`
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r120_h20.csv`
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r130_h20.csv`
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r140_h20.csv`
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r150_h20.csv`
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r160_h20.csv`
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r30_h20.csv`
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r40_h20.csv`
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r50_h20.csv`
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r60_h20.csv`
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r70_h20.csv`
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r80_h20.csv`
  - `logs/strapdown_accuracy/truth_required_load/strapdown_clock0p2_no_los_filter_truth_theory_N3_strapdown_clock0p2_no_los_filter_20260616_060330_r90_h20.csv`
