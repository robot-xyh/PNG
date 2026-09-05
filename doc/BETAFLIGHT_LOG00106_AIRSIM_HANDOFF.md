# Betaflight LOG00106 到 AirSim 的数据交接说明

## 1. 交接目的与证据边界

本文只归档并解释 `2026-09-04` 最后一组真实外场实验，不混入此前台架、无桨或其他飞行日志。
该组数据由一架 Betaflight 拦截机、一架 PX4 靶机和 Orange Pi RK3588 主机日志组成。交接目标是
让另一位工程师在 AirSim 中复现相同控制链，检查主要信号的方向、相对幅值、时序和饱和趋势是否
一致，不要求 AirSim 与真实飞行逐点数值相等。

本架次能够证明：PNG 控制链在单次近似静止目标场景中建立了闭合轨迹，并最终发生物理接触；
Roll/Pitch 角速度链路和油门传输链路工作。它不能证明真实命中率达到 `80%`，也不能提供可信的
双机绝对 GPS 脱靶距离。

本文不复制或移动任何大型原始文件。所有路径均相对于仓库根目录
`/home/linux/Documents/PNG-betaflight-upward-camera`。

## 2. 实验场景

| 项目 | 真实实验状态 |
| --- | --- |
| 拦截机 | Betaflight 多旋翼，飞控 `BTFL 25.12.2`，Orange Pi RK3588 运行视觉、导引和 MSP 发布 |
| 靶机 | PX4 多旋翼，碰撞时保持 Position Control，无 failsafe |
| 相机 | 拦截机机体固定上视相机，输出 `640 x 512`，目标位于拦截机上方 |
| 目标状态 | PNG 发布窗口内平移近似静止；末端碰撞前约 `0.27 s` 靶机飞手开始左滚 |
| 导引律 | `velocity_establishing_png`，纯视觉 LOS/LOS rate 加 MSP NED 速度建立 |
| 拦截机模式 | Acro/Rate 是主动控制门槛；MSP OVERRIDE mode ID `50` |
| 接管权限 | `msp_override_channels_mask=15`，接管 Roll、Pitch、Throttle、Yaw；Yaw 固定中位 |
| 控制发布 | MSP RAW RC，`50 Hz`，AETR1234；本架次平均 `49.682 Hz` |
| 算法发布 | `1.670804 s`，84 个主机控制记录行、85 个算法发送帧 |
| 退出与接触 | `aux_disabled` 后约 `21 ms` 切到 passthrough；PNG 停止后约 `0.902 s` 发生接触 |
| 实验结果 | 两机日志独立记录到同一物理冲击；碰撞后均出现飞控恢复响应 |

靶机在 PNG 发布窗口内的合速度 P50/P95/max 为
`0.042/0.056/0.058 m/s`，N/E/D 端点位移为
`-0.043/+0.035/-0.038 m`，三轴人工杆量均为零。因此，“近似静止目标”适用于主要闭合阶段。
但在主冲击前约 `0.270 s`，靶机接收机 Roll 已离开中位；主冲击前约 `0.127 s`，PX4 Roll
指令已明显变化。AirSim 的理想基线应保持靶机完全静止；如要复现末端姿态，可另设敏感性工况，
不得把这段末端左滚误写为全程目标机动。

## 3. 坐标系与相机安装

### 3.1 坐标系约定

- `NED` 惯性/导航系：`+X` 北、`+Y` 东、`+Z` 下；上升速度和向上加速度的 D 分量为负。
- Betaflight 原始机体约定在项目原始说明中记为 `FLU`：前、左、上。
- PX4 和本仓库内部控制计算统一使用 `FRD`：前、右、下。
- 对一般三维几何向量，FLU 到 FRD 的理想轴变换是 `diag(1,-1,-1)`。但 MSP 传感器字段不得
  仅凭该通用矩阵重解释；本固件绑定的 `MSP_RAW_IMU` 转换必须使用日志 meta 中记录的比例和
  轴符号：`scale=0.0625 deg/s/LSB`、`axis_order=[x,y,z]`、
  `axis_sign=[+1,-1,+1]`，输出为 `body_frd`。
- `R_IB` 把 FRD 机体系向量变换到 NED：`v_I = R_IB @ v_B`。
- MSP 姿态记录的 Pitch 使用 Betaflight 显示符号；仓库转换到 FRD/NED 时执行
  `pitch_FRD = -pitch_MSP`。AirSim 初始化姿态时要使用 FRD/NED 符号。

### 3.2 相机系与外参

相机使用 OpenCV 坐标：`x` 向图像右、`y` 向图像下、`z` 沿光轴向前。实机固定外参为：

```text
R_BC = [[0, 1,  0],
        [1, 0,  0],
        [0, 0, -1]]
```

该矩阵把相机向量变换到 FRD 机体系：

- 相机 `+x` -> 机体 `+y`，即图像右对应机体右；
- 相机 `+y` -> 机体 `+x`，即图像下对应机头方向；
- 相机 `+z` -> 机体 `-z`，即光轴指向机体上方。

`R_BC` 已记录为 `verified=true`，正交误差为 0、行列式为 `+1`、光轴误差为 `0 deg`。惯性
视线计算必须采用：

```text
lambda_C = normalize([(u-cx)/fx, (v-cy)/fy, 1])
lambda_I = normalize(R_IB @ R_BC @ lambda_C)
```

仓库的固定相机 runner 用命令行参数 `--camera-pitch-deg -90` 表示相对机体上视；
`_fixed_camera_pose()` 会把该值取负后，以 `+90 deg` pitch 传给 AirSim API。算法侧的
`_fixed_camera_R_BC()` 则使用命令行的 `-90 deg` 构造外参。复现时必须同时核对最终 AirSim
相机画面和上述 `R_BC`，不能把命令行参数直接当成 AirSim API 欧拉角，也不能只依赖欧拉角约定。

### 3.3 相机内参

| 参数 | 数值 |
| --- | ---: |
| 输出宽高 | `640 x 512` |
| `fx`, `fy` | `530.8443137412`, `532.2954942356` px |
| `cx`, `cy` | `321.0278689412`, `247.2573194658` px |
| 畸变 `k1,k2,p1,p2,k3` | `0.0780556731, -0.9515884756, -0.0068803531, -0.0002146133, 1.4612166831` |
| 水平/垂直半视场 | `31.000689 / 24.915405 deg` |
| 采集请求 | `1280 x 1024`、MJPG、`180 fps`；运行时输出 `640 x 512` |
| 感知更新 | `30 Hz` |
| 标定 ID | `unarchived_candidate_640x512` |

实机先用上述畸变参数执行 `cv2.undistort`，再使用相同内参计算 LOS。AirSim 可以渲染理想针孔
图像，但 LOS 计算必须使用相同的 `640 x 512` 内参和主点；若仿真显式注入镜头畸变，则还必须
执行与实机相同的去畸变步骤。真实相机时间戳来自 `capture_return_monotonic`，不是硬件曝光
时刻，这会给端到端视觉时延留下额外不确定性。

## 4. 控制器和运行参数

### 4.1 导引器

| 参数 | 数值 |
| --- | ---: |
| 导引律 | `velocity_establishing_png` |
| 导航系数 `N` | `3.0` |
| 固定速度 `fixed_vm` | `10.0 m/s` |
| 速度建立增益 | `1.2 s^-1` |
| 速度建立加速度上限 | `7.0 m/s2` |
| PNG 加速度上限 | `7.0 m/s2` |
| FOV 回中增益 | `16.0 s^-2` |
| FOV 回中加速度上限 | `7.0 m/s2` |
| 总加速度上限 | `7.0 m/s2` |
| 垂直参考速度上限 | `6.0 m/s` |
| 进入 PNG_TRACK 的速度比例 | `0.8`，即 LOS 方向速度达到 `8 m/s` |
| LOS 最大预测时间 | `0.15 s` |
| 检测/速度超时 | `0.35 / 0.5 s` |
| 连续获取帧数 | `1` |
| FOV 优先级 | 启用；半视场占比 `0.75` 开始、`0.95` 满权重 |
| FOV 锥约束 | `0 deg`，即该附加约束关闭 |

生产控制器的主要公式如下，AirSim 应复用仓库实现
`vision_guidance/betaflight_intercept_controller.py`，不要复制后另写一套近似公式：

```text
control_los = normalize(lambda_I + min(detection_age, 0.15) * lambda_dot_I)
v_ref       = 10 * control_los，且 v_ref_D 限制到 [-6,+6] m/s
a_speed     = clip_norm(1.2 * (v_ref - v_NED), 7)
a_png       = clip_norm(3 * 10 * lambda_dot_I, 7)
a_fov_B     = clip_norm([16*los_Bx, 16*los_By, 0], 7)
a_fov_I     = R_IB @ a_fov_B
a_total     = clip_norm(a_speed + a_png + a_fov, 7)
```

实际实现还会在目标进入矩形视场的 `75%--95%` 边缘带时，逐渐压制
`a_speed+a_png` 中与 `a_fov` 相反的分量。最终总加速度仍限制为 `7 m/s2`。

### 4.2 加速度到飞控命令

| 参数 | 数值 |
| --- | ---: |
| 映射 | `accel_tilt_rate`；按当前 yaw 把 NED 加速度转到航向对齐坐标，再求期望 Roll/Pitch |
| Roll/Pitch 姿态比例增益 | `4.0 s^-1` |
| Roll/Pitch 角速度限制 | `60 / 60 deg/s` |
| Roll/Pitch 姿态硬限制 | `35 / 35 deg` |
| 速率符号 | Roll `+1`，Pitch `-1` |
| Yaw | `0 deg/s`，发送 `1500 us` |
| 角速度入场交接 | `0.8 s` smoothstep，起点取当前实测 gyro |
| 悬停油门 | `1275 us` 对应 `1.0 g` |
| 上端标定 | `1500 us` 对应模型 `2.37 g` |
| 油门交接 | `0.8 s`，最大 slew `600 us/s` |
| 油门范围 | `1200--1500 us`；切入参考要求 `1200--1400 us` |

本架次的模型负载要求为 `1.548--1.736 g`，模型目标油门为 `1367--1396 us`。实际发送油门
从切入前 `1303 us` 经 `0.8 s` 平滑交接到最高 `1396 us`，没有下跳或超过 `1500 us`。

### 4.3 运行调度和检测

| 参数 | 数值 |
| --- | ---: |
| 主控制循环 / MSP 发布 | `50 / 50 Hz` |
| 姿态轮询 | `20 Hz` |
| GPS、气压、RC、状态轮询 | 各 `5 Hz` |
| 模型 | `drone_v8n_v21_kd_relu_lambda008_640_640-rk3588.rknn` |
| 模型 SHA256 | `ad905c19e3e2b5386fa1a5d562285a02d5e5a75ad02d89ce2f1d344810c60f59` |
| 检测最低分数 | `0.25` |
| ByteTrack 确认 | 连续 `3` 帧，buffer `0.5 s` |
| 串口 | `/dev/ttyS1`，`115200 baud`，单 UART 异步流水线 |

算法发布窗口内目标始终为 `track_id=5`，新感知结果 40 个，confirmed 为 `100%`，无 ID switch
和 fragment。结果年龄 P50/P95/max 为 `49.18/78.47/84.58 ms`，RKNN 总耗时
P50/P95/max 为 `6.24/6.56/6.73 ms`，姿态融合等待 P95/max 为 `40.12/52.85 ms`。

## 5. 接管和碰撞时间线

以下 UTC 由三套日志对齐得到。名义毫秒差不能视为硬件同步精度，误差说明见第 7 节。

| UTC | 相对靶机主冲击 | 事件 |
| --- | ---: | --- |
| `10:38:00.723373` | `-2.626968 s` | RC7/MSP OVERRIDE 生效，状态进入 ACTIVE |
| `10:38:00.756168` | `-2.594173 s` | 首次实际 `publish=algorithm` |
| `10:38:02.426972` | `-0.923369 s` | 飞行模式门控变为 `aux_disabled` |
| `10:38:02.448019` | `-0.902322 s` | 发布切为 `passthrough`，PNG 指令停止 |
| `10:38:03.079877` | `-0.270464 s` | 靶机接收机 Roll 离开中位 |
| `10:38:03.223838` | `-0.126503 s` | 靶机 PX4 Roll 指令明显变化 |
| `10:38:03.295687` | `-0.054654 s` | 最后有效目标结果所对应的软件估计曝光时刻 |
| `10:38:03.350341` | `0 s` | 靶机 IMU 主冲击，首次超过 `20 m/s2` |
| `10:38:03.352392` | `+0.002051 s` | Orange Pi 收到最后一个 measured 检测结果 |
| `10:38:03.352876` | `+0.002535 s` | 拦截机 Blackbox 首次出现电机边界响应 |
| `10:38:04.121307` | `+0.770966 s` | RC7 退出 MSP OVERRIDE，恢复物理 RC |
| `10:38:05.890330` | `+2.539989 s` | 拦截机 DISARM |

PNG 只在 `10:38:00.756168--10:38:02.426972` 附近主动发布算法指令。碰撞发生时已经是
`passthrough`，但 RC7 仍保持 MSP OVERRIDE；此时 passthrough 发送的是切入前冻结的
`AETR=1500/1500/1303/1500`，而不是飞手的实时摇杆。算法停止时拦截机已经建立明显向上速度，
随后约 `0.902 s` 依靠惯性继续闭合并接触目标。

## 6. 真实数据摘要

### 6.1 视觉、导引和饱和

| 指标 | 真实结果 |
| --- | ---: |
| 首个 measured 框 | `[318.303,166.230,367.189,210.729]` px |
| 首个框中心 / 面积比 | `(342.746,188.480)` px / `0.006639` |
| 发布窗口框中心范围 | `u=292.342--374.924`，`v=172.195--279.338` px |
| 发布窗口框面积比 | `0.005807--0.014160` |
| 发布窗口 `lambda_I` | X `-0.0340--+0.0641`，Y `+0.0172--+0.0682`，Z `-0.9998---0.9967` |
| 速度建立项饱和 | `55/84 = 65.48%` |
| PNG 项饱和 | `0/84 = 0%` |
| FOV 项饱和 | `0/84 = 0%` |
| 总加速度饱和 | `61/84 = 72.62%` |
| 总加速度 D 分量 | `-6.934---5.368 m/s2`，持续向上 |

本架次一开始就要求把拦截机速度建立到沿 LOS 的 `10 m/s`，而当时实际速度远低于参考速度，
所以速度建立项和总加速度很早、很频繁地达到 `7 m/s2`。这不是数值溢出，而是配置上限正在
主导控制。PNG 和 FOV 分项未单独达到其上限。

### 6.2 角速度响应

| 信号 | Roll | Pitch |
| --- | ---: | ---: |
| RK3588 期望范围 | `-31.74--+22.87 deg/s` | `-34.14--+15.64 deg/s` |
| Betaflight setpoint | `-32--+25 deg/s` | `-34--+18 deg/s` |
| Blackbox gyro | `-39--+32 deg/s` | `-36--+19 deg/s` |
| 同钟最佳滞后 | `15 ms` | `15 ms` |
| 滞后后相关系数 | `0.9910` | `0.9944` |
| 拟合增益 | `1.0255` | `1.0085` |
| P95 绝对误差 | `4.41 deg/s` | `3.51 deg/s` |

两轴都完成了正负反转，未达到 `60 deg/s` 命令限制，也未触发 `35 deg` 姿态硬限幅。该结果支持
“轴向、Rate 映射和飞控闭环跟随正确”。`15 ms` 只由同一 Blackbox 时钟中的 setpoint 和 gyro
计算，可信度高于跨设备延迟估计。

### 6.3 油门、比力和动力响应

| 指标 | 真实结果 |
| --- | ---: |
| 切入前油门 | `1303 us` |
| 模型目标油门 | `1367--1396 us`，中位 `1382 us` |
| 实际发送油门 | `1305--1396 us`，中位 `1375 us` |
| 模型期望负载 | `1.548--1.736 g`，中位 `1.650 g` |
| Blackbox 实测比力 | P50 `1.298 g`，P95 `1.450 g`，max `1.468 g` |
| 交接完成后实测/模型比 | P50 `0.809`，P95 `0.847` |
| 电压 | `22.65--22.95 V` |
| 电流 | P95 `13.68 A`，峰值 `21.01 A` |

发送油门与 Betaflight 内部 throttle 波形相关系数约 `0.9995`，说明油门传输和飞控接收正确。
实际瞬态比力比模型要求低约 `15%--20%`，可能来自油门到推力的非线性、电池电压、桨叶入流、
电机动态及前 `0.8 s` 交接。AirSim 敏感性工况应显式注入这一偏差，不应修改真实 CSV 使其看似
符合模型。

### 6.4 相对运动与接触

双机本地 NED 原点不同。联合分析以已确认的物理接触时刻为零，只比较两机各自相对接触点的
位置增量。按 Orange Pi NED，算法开始时尚需闭合约 `5.06 m`；按拦截机 Blackbox 气压高度，
垂直尺度约为 `6.34 m`。算法停止时两种来源均给出约 `3.27--3.39 m` 尚待闭合。碰撞前向上
闭合速度按不同估计器约为 `2.1--3.5 m/s`。

接触锚定计算在算法开始时给出的靶机相对拦截机 NED 向量约为：

```text
[-0.7259, +0.2841, -4.9998] m
```

但该向量的水平角与视觉 LOS 并不完全一致，反映了 GPS/状态估计和接触锚定的误差。AirSim
初始化应优先保持视觉方向，再用 `5.06 m` 和 `6.34 m` 做距离敏感性：首次 measured LOS 为
`[-0.030416,+0.066726,-0.997308]`，所以两组推荐相对位置为：

```text
range=5.060 m: [-0.1539, +0.3376, -5.0466] m
range=6.340 m: [-0.1928, +0.4230, -6.3227] m
```

这两组都应被标为“LOS 约束、距离估计”，不能称为实测机体中心间距。

## 7. 原始文件、读取方法与可信范围

### 7.1 一级原始证据

| 文件 | 内容和读取方法 | 可信字段 | 不应怎样使用 |
| --- | --- | --- | --- |
| `logs/flight_active_supervised/FLIGHT_ACTIVE_05S_VIDEO_20260904_183721_20260904_183722.csv` | Orange Pi 50 Hz 主 CSV；用 Python `csv.DictReader` 或 pandas 读取 | 视觉框、跟踪、LOS、各导引分项、饱和状态、期望角速度/推力、MSP 发布状态、低频 MSP 姿态/GPS | 低频 gyro/电流不能替代 Blackbox 冲击分析；轮询字段会在多行间保持旧值 |
| 同名前缀 `_meta.json` | 运行时配置快照、启动参数、相机/模型/飞控身份和批准信息；用 JSON 解析 | 本架次真正生效的参数和 SHA256 | `repository_commit` 为空，不能用它恢复精确代码提交 |
| 同名前缀 `_events.jsonl` | 稀疏状态事件；逐行 JSON 读取 | 文件截断前的启动和早期状态事件 | 文件固定为 16384 B 且尾部有 NUL，未覆盖 ACTIVE、退出和碰撞；关键时间线要从主 CSV 重建 |
| `...183721_console.log` | 启动控制台和早期状态文本 | 模型加载、导引参数、批准和外参启动自检 | ARM 后日志缺失，尾部有 NUL，不能作为完整事件记录 |
| `logs/blackbox_import/LOG00106.BFL` | 拦截机高频 Blackbox；使用 Betaflight `blackbox_decode --unit-rotation raw --merge-gps --save-headers` | 同钟 setpoint、gyro、加速度、姿态、内部 throttle、电机 raw、电压电流和冲击 | 当前定制固件 `gyro_scale=1`，不要用 `--unit-rotation deg/s` 再次缩放；电机 raw 不是物理 PWM；模式名称文本枚举可能错位 |
| `logs/target-log/10_35_19.ulg` | 靶机 PX4 ULog；用 `pyulog`/Flight Review 读取 | PX4 本地位置/速度、姿态、人工输入、IMU、执行器、电源、nav_state、failsafe、UTC 锚 | 靶机本地 NED 原点不能直接与拦截机本地 NED 相减 |

两个中文 CSV 都是从完整主 CSV 派生的便捷视图，不是新增原始证据：

- `_essential_zh.csv`：全架次中文关键字段，便于人工浏览。
- `_png_takeover_zh.csv`：从接管切入开始的中文视图。第一行可能仍是 `passthrough`；判定真实算法
  发布必须使用 `MSP发布模式=algorithm`，不能只看 `安全状态=ACTIVE`。

### 7.2 二级派生证据

| 路径 | 用途 | 边界 |
| --- | --- | --- |
| `logs/analysis/LOG00106_target_joint/metrics.json` | 双机配对、UTC、目标静止性、接触和相对运动指标 | 相对位置是接触锚定派生量，不是绝对测距 |
| `logs/analysis/LOG00106_target_joint/joint_event_timeline.csv` | 关键事件及事件时刻的双机状态 | 用于事件级比较，不用于高频 PID 分析 |
| `logs/analysis/LOG00106_target_joint/joint_timeseries_50hz.csv` | 双机统一 50 Hz 时间轴 | 插值和重采样后的派生表；冲击峰值仍以各自原始高频日志为准 |
| `joint_approach_timeline.png` / `joint_impact_timeline.png` | 接近过程和双机冲击可视化 | 图中接触锚定距离不是绝对机体中心距离 |
| `doc/evidence/BETAFLIGHT_LOG00106_CONTROL_RESPONSE_metrics.json` | 期望/实际角速度、油门、比力和同钟延迟指标 | 统计窗口排除了碰撞后恢复 |
| `doc/figures/log00106_control_response/*.png` | 角速度、油门/比力、导引闭合三张专题图 | 主机和 Blackbox 曲线各自按算法开始归零，不能从图上读取跨机绝对毫秒延迟 |
| 两份现有 LOG00106 报告 | 完整碰撞解释及期望/实际响应解释 | 结论来源仍是上述原始和派生数据 |

## 8. 时钟对齐方法与误差

1. Orange Pi：`UTC = meta.created_unix_s + CSV.elapsed_s`，其中
   `created_unix_s=1788518242.513792`。
2. 拦截机 Blackbox：使用日志头 `Log start datetime` 加 `time (us)` 构造 UTC；同一文件内的
   setpoint、gyro、电机和 IMU 共用飞控时钟。
3. 靶机 ULog：对 `vehicle_gps_position.time_utc_usec - ULog timestamp` 取中位数，把 PX4
   单调时间映射到 UTC。该中位偏移为 `1788517723731847 us`。
4. 以靶机首次主冲击和拦截机首次电机边界响应交叉检查，名义差为 `2.535 ms`。

靶机 GNSS 发布偏移残差 P5/P95 为 `-92.8/+70.8 ms`。因此跨设备事件应使用保守
`+/-0.1 s` 窗口；`2.535 ms` 只说明两侧冲击高度一致，不能声称系统完成了 2.5 ms 硬件同步。
唯一可作为严格控制延迟的数据是 Blackbox 内部同钟 setpoint 到 gyro 的约 `15 ms`。

AirSim 对比建议同时输出三条时间轴：

- `t_algorithm_s`：以第一条算法控制命令为零，比较控制建立过程；
- `t_contact_s`：以真实或模拟最近接触为零，比较退出后的惯性闭合；
- `sample_time_s` 与 `available_time_s`：分别表示传感器真值/曝光时刻和控制器拿到结果的时刻。

不要通过人为移动真实曲线来追求逐点重合。角速度响应可在同一模拟器时钟中估计 lag；跨设备
趋势只比较正负反转、峰值顺序、上升/下降阶段和归一化幅值。

## 9. 绝对 GPS 和碰撞后数据的禁用规则

### 9.1 绝对 GPS 不能计算脱靶距离

在双机已经物理接触的时刻，直接相减两台 GNSS 仍得到：北向 `-5.18 m`、东向 `+1.31 m`、
水平差 `5.34 m`、高度差约 `-8.25 m`。这显然不是接触时的机体间距，而是独立 GNSS 接收机、
不同本地原点和高度基准、天线位置、发布延迟及估计器差异的组合误差。

因此：

- 禁止把双机绝对经纬度/NED 直接相减后作为 miss distance；
- 可以比较各自在本地坐标中的位置增量和速度趋势；
- 可以用接触时刻归零估算此前“尚待闭合位移”，但必须标注米级不确定性；
- AirSim 的真值 miss distance 只用于仿真内部评价，不能反推本架次真实脱靶距离精度。

### 9.2 碰撞后饱和不能当作 PNG 输出

PNG 发布窗口内没有电机边界饱和，命令角速度也未触及 `60 deg/s`。碰撞后拦截机出现约
`963 deg/s` 合成角速度、电机 `158/2047` 边界和 `82.06 A` 峰值；靶机出现 `5.84 g` 冲击。
这些是物理接触后 Betaflight/PX4 的姿态恢复动作，不是 PNG 直接发送的控制幅值。

AirSim 做控制器趋势拟合时必须在物理接触前截断；如另行模拟碰撞，只能把碰撞后数据作为接触
模型和飞控恢复的独立验证，不得纳入 PNG 增益、油门模型或饱和率统计。

## 10. AirSim 复现工况

### 10.1 推荐初始条件

将拦截机算法开始位置设为 AirSim NED 原点。下列“真值”和“控制器观测值”应分开保存：

| 项目 | 推荐值 |
| --- | --- |
| 拦截机位置 | `[0,0,0] m` |
| 靶机相对位置主工况 | `[-0.1539,+0.3376,-5.0466] m`，由首个 measured LOS 和 5.06 m 距离估计构造 |
| 靶机相对位置距离敏感性 | `[-0.1928,+0.4230,-6.3227] m`，同一 LOS、6.34 m 距离 |
| 仅供运动增量核对的接触锚定向量 | `[-0.7259,+0.2841,-4.9998] m`，不得强迫其同时匹配相机框 |
| 拦截机物理初速度 | `[-0.7603,+0.1741,+0.1700] m/s` |
| 控制器初始滤波速度 | `[-0.7748,+0.1850,+0.1438] m/s` |
| 靶机初速度 | `[-0.0177,-0.0041,-0.0436] m/s`；理想基线可设为全零 |
| 拦截机 MSP 姿态 | Roll/Pitch/Yaw=`+0.2/+3.0/333.0 deg` |
| 拦截机 FRD/NED 姿态 | Roll/Pitch/Yaw=`+0.2/-3.0/333.0 deg` |
| 靶机姿态参考 | 发布窗口约 Roll `1.18--2.04 deg`、Pitch `-5.80---5.32 deg`、Yaw `-3.59---3.45 deg` |
| 首个图像目标框 | `[318.303,166.230,367.189,210.729]` px，中心 `(342.746,188.480)` |
| 初始油门 | `1303 us` |
| 电池电压参考 | `22.65--22.95 V` |

真实目标尺寸没有在本数据组中形成可信标定。AirSim 目标模型尺寸应调整到首帧投影框，而不是从
框面积反推后宣称是真实机体尺寸。

### 10.2 工况 A：理想静止目标基线

- 靶机位置和姿态保持不变，速度为零。
- 使用相同 `R_BC`、内参、`640 x 512` 图像和上视安装。
- 视觉测量 30 Hz、控制 50 Hz；不注入漏检、ID switch、测量噪声或额外时延。
- 保留生产控制器自身的 LOS 预测、0.8 s 角速度入场平滑、0.8 s 油门交接和命令上限。
- 推力模型按 `1275 us=1 g`、`1500 us=2.37 g` 理想执行。
- 在 `t=1.670804 s` 停止算法输出，并继续无新 PNG 加速度仿真至少 `1.2 s`，观察惯性闭合；
  同时另保存“不提前退出、持续闭环”结果，避免把退出逻辑和导引能力混为一项。

### 10.3 工况 B：实测延迟和推力偏差敏感性

在工况 A 基础上注入：

- 感知结果年龄：优先重放主 CSV 中 40 个 measured 样本的真实 `sample/available` 间隔；无法
  重放时使用 P50/P95/max=`49.18/78.47/84.58 ms` 的有界分布。
- 姿态融合等待：重放主 CSV；近似时使用 P95/max=`40.12/52.85 ms`，注意它与结果年龄不是
  简单独立相加。
- 角速度执行：纯延迟 `15 ms`，再用一阶响应或 AirSim 飞控动态；校准后期望 setpoint/gyro
  增益约 Roll `1.026`、Pitch `1.009`。
- 推力偏差：交接完成后将模型比力乘以 `0.809` 作为主敏感性，另用 `0.847` 作为较乐观边界；
  保留电压 `22.65--22.95 V` 标签。
- 控制器观测速度使用 `0.25 s` 一阶滤波和 5 Hz MSP 更新；物理真值保持 AirSim 高频输出。
- 可选末端姿态敏感性：只在预计接触前 `0.27 s` 给靶机短左滚，平移轨迹仍保持近似静止。

两个工况都必须保持 `N=3`、`fixed_vm=10 m/s`、`max_guidance_accel=7 m/s2`。敏感性工况的
目的不是把仿真调到必然碰撞，而是检查延迟和推力低估后，各信号趋势、闭合速度和接触时刻如何
变化。

## 11. AirSim 输出 CSV 字段合同

所有向量字段必须显式带坐标系和单位；禁止使用无后缀的 `x/y/z`。至少输出以下字段：

### 11.1 时间、工况和状态

```text
case_id, run_id, seed, t_sim_s, t_algorithm_s, t_contact_s,
sample_time_s, available_time_s, measurement_age_ms,
controller_phase, guidance_valid, guidance_reason,
algorithm_active, contact_detected, post_contact
```

### 11.2 双机真值和控制器观测

```text
interceptor_position_n_m, interceptor_position_e_m, interceptor_position_d_m,
interceptor_velocity_n_m_s, interceptor_velocity_e_m_s, interceptor_velocity_d_m_s,
interceptor_velocity_observed_n_m_s, interceptor_velocity_observed_e_m_s,
interceptor_velocity_observed_d_m_s,
target_position_n_m, target_position_e_m, target_position_d_m,
target_velocity_n_m_s, target_velocity_e_m_s, target_velocity_d_m_s,
relative_position_n_m, relative_position_e_m, relative_position_d_m,
relative_range_m, closing_speed_m_s, miss_distance_truth_m
```

### 11.3 相机、框和 LOS

```text
bbox_x1_px, bbox_y1_px, bbox_x2_px, bbox_y2_px,
bbox_center_u_px, bbox_center_v_px, bbox_area_ratio, bbox_in_fov,
lambda_truth_n, lambda_truth_e, lambda_truth_d,
lambda_measured_n, lambda_measured_e, lambda_measured_d,
lambda_filtered_n, lambda_filtered_e, lambda_filtered_d,
lambda_dot_n_s, lambda_dot_e_s, lambda_dot_d_s,
omega_los_n_rad_s, omega_los_e_rad_s, omega_los_d_rad_s
```

### 11.4 导引分项和饱和

```text
velocity_reference_n_m_s, velocity_reference_e_m_s, velocity_reference_d_m_s,
speed_accel_n_m_s2, speed_accel_e_m_s2, speed_accel_d_m_s2,
png_accel_n_m_s2, png_accel_e_m_s2, png_accel_d_m_s2,
fov_accel_n_m_s2, fov_accel_e_m_s2, fov_accel_d_m_s2,
total_accel_n_m_s2, total_accel_e_m_s2, total_accel_d_m_s2,
speed_saturated, png_saturated, fov_saturated,
fov_priority_active, fov_priority_weight, total_saturated
```

### 11.5 姿态、角速度、油门和比力

```text
roll_frd_deg, pitch_frd_deg, yaw_ned_deg,
desired_roll_frd_deg, desired_pitch_frd_deg,
roll_rate_setpoint_deg_s, pitch_rate_setpoint_deg_s, yaw_rate_setpoint_deg_s,
roll_rate_actual_deg_s, pitch_rate_actual_deg_s, yaw_rate_actual_deg_s,
throttle_model_target_us, throttle_handover_output_us, throttle_applied_us,
throttle_handover_alpha, thrust_model_load_factor_g,
specific_force_actual_g, thrust_model_ratio,
rate_limited, tilt_limited, throttle_limited
```

建议另输出一个 JSON metrics 文件，至少包含各工况命中/最近距离、接触时刻、算法退出时剩余距离、
LOS rate 峰值、各导引分项峰值和饱和占比、Rate lag/相关系数/增益、油门和比力分布。每个派生
指标必须写明统计窗口，尤其要明确是否排除碰撞后样本。

## 12. 趋势一致性验收表

数值允许因 AirSim 气动、目标模型尺寸和距离不确定性而变化。以下验收关注趋势，不把真实单架次
数值设成强制逐点门槛。

| 对比项 | 真实参考 | AirSim 趋势验收 |
| --- | --- | --- |
| 靶机状态 | PNG 窗口速度 P95 `0.056 m/s` | 理想基线应近似静止；不能用目标平移解释主要 LOS 变化 |
| 初始 LOS | 主要指向 NED `-D`，Z 约 `-0.997` | 上视相机中心目标必须得到负 D；不得轴反或上下颠倒 |
| 框中心 | 从中心上方附近移动并穿越/靠近中心区域 | `u/v` 运动方向与 `R_BC`、姿态和 LOS 一致 |
| 框面积 | 发布窗口约 `0.0058 -> 0.0142`，接触前继续快速增大 | 闭合时面积总体增大；不要求绝对面积相等 |
| LOS/LOS rate | LOS X 和 LOS rate X 均发生正负变化 | 转向后相关分量应反转；LOS rate 不要求单调趋零 |
| 参考速度 | `10*LOS`，D 限为 `-6 m/s` | 靶机在上方时参考 D 应为负并触及 `-6 m/s` 限制 |
| 速度建立项 | `65.48%` 行饱和，D 持续为负 | 初段应成为主导项并较早饱和；速度建立后应有释放趋势 |
| PNG 项 | 本架次未单项饱和 | 应随 `lambda_dot` 变号，不应无故固定在 `7 m/s2` |
| FOV 项 | 未单项饱和 | 应把目标推向画面中心；符号必须与框偏移纠正方向一致 |
| 总加速度 | `72.62%` 行达到 `7 m/s2`，D `-6.93---5.37` | 初段应频繁达到总上限且主要向上；不得超过 `7 m/s2` |
| 期望姿态/Rate | Roll/Pitch 都有正负反转，未到 `60 deg/s` | 方向和反转顺序一致；不得因坐标误用长期反向 |
| Rate 跟踪 | 同钟约 `15 ms`，相关 `0.991/0.994` | 敏感性工况应有约 15 ms 滞后和高正相关；不要求逐点相等 |
| 油门交接 | `1303 -> 1305...1396 us`，约 `0.8 s` | 单调平滑趋向目标，无切入下跳、无超过 `1500 us` |
| 比力 | 真实约为模型的 `0.81`，P95 边界约 `0.85` | 理想工况接近模型；偏差工况应产生较低爬升加速度和更晚接触 |
| 闭合 | 算法停止时仍约有 `3.3 m`，随后惯性接触 | 提前停止工况应保留明显闭合速度；是否接触作为模型结果而非硬编码 |
| 碰撞后 | 大角速度和电机饱和来自接触恢复 | 必须从 PNG 指令统计中排除；不能用于调大控制器增益 |

建议对幅值使用以下归一化：

- 时间：以算法开始为 0；另以接触为 0 画末端图。
- LOS：直接比较单位向量和 LOS rate，不使用双机绝对 GPS 距离。
- 加速度：除以 `7 m/s2`，比较各分项和总项的符号、占比和饱和时段。
- 角速度：除以 `60 deg/s`，并分别比较 setpoint 和实际 gyro。
- 油门：比较 `(us-1275)/(1500-1275)`；比力比较 `actual/model`。
- 框：中心使用 `(u-cx)/fx`、`(v-cy)/fy`，面积除以 `640*512`。
- 距离：对真实数据只使用接触锚定的相对变化；对 AirSim 可同时报告真值距离，但不要混成同一
  “实测脱靶”指标。

## 13. 已知问题与交接注意事项

1. 本架次只有一个静止目标接触样本，不能验证 `>=80%` 命中率，也不能外推到横移或规避目标。
2. 初始距离只能约束到 `5.06--6.34 m` 量级；视觉方向比双机 GPS 差分更可信。
3. 推力模型在本次瞬态高估实际比力约 `15%--20%`，AirSim 必须保留敏感性工况。
4. 速度建立项和总加速度很早饱和，说明结果强依赖 `fixed_vm=10 m/s` 和 `7 m/s2` 上限。
5. 目标面积比在最后 measured 结果已约 `0.274`，目标丢失后候选导引还能保留约 `0.3 s`；这是
   后续近距终端门控问题，不应由 AirSim 静默修正。
6. 飞手切换飞行模式导致 `aux_disabled`，但 RC7 未立即退出，passthrough 仍发送冻结 RC。AirSim
   应分别建模“算法停止”和“物理接管退出”，不要把它们合并成同一时刻。
7. `_events.jsonl` 和 `console.log` 尾部含 NUL 且提前结束；完整时序以主 CSV 为准。
8. meta 的 `repository_commit` 为空；配置内容和 SHA256 可追溯，但无法仅凭 meta 锁定代码提交。
9. 相机标定 ID 明确写着 `unarchived_candidate`；参数已用于本架次，但缺少原始标定图片归档。
10. 真实 target actor 尺寸未知；AirSim 只应按首帧投影框调视觉尺度。

## 14. 路径、大小和 SHA256

已在当前工作区执行逐文件存在性检查和 `sha256sum`。下列 20 个文件全部存在；console 的真实
文件名不带第二个 `_20260904_183722` 后缀。大型原始文件未复制或移动。

| 文件 | 字节 | SHA256 |
| --- | ---: | --- |
| `logs/flight_active_supervised/FLIGHT_ACTIVE_05S_VIDEO_20260904_183721_20260904_183722.csv` | 8060890 | `2f4cba58655be4d142237fc5e05c7ad16b0c3c9131a838d1413e9676cdc557a6` |
| `logs/flight_active_supervised/FLIGHT_ACTIVE_05S_VIDEO_20260904_183721_20260904_183722_meta.json` | 40623 | `109766c2c003e6c67060dc06e538c2c33944cf66b661b22e204a2708d7292238` |
| `logs/flight_active_supervised/FLIGHT_ACTIVE_05S_VIDEO_20260904_183721_20260904_183722_events.jsonl` | 16384 | `fe819c9918466eed1eb6572563e70d3a78779765efc48edfe8fc5f4e56c42b59` |
| `logs/flight_active_supervised/FLIGHT_ACTIVE_05S_VIDEO_20260904_183721_20260904_183722_essential_zh.csv` | 2346983 | `cee6752dacd97eab8a3e6467e890f1c474d965aabe43f4104c94d9b74b4b72da` |
| `logs/flight_active_supervised/FLIGHT_ACTIVE_05S_VIDEO_20260904_183721_20260904_183722_png_takeover_zh.csv` | 219095 | `c887bdbe9cd3f4a3b98ef9589177e3e87e7d3ef7935776ed66dd731f68ebd46d` |
| `logs/flight_active_supervised/FLIGHT_ACTIVE_05S_VIDEO_20260904_183721_console.log` | 2648 | `215366b858c513d15c967cbeb1f2461b7afb3117bce9c989f32e713a766aff79` |
| `logs/blackbox_import/LOG00106.BFL` | 651894 | `fc58d049df776a6a771f312ec5ad71bbd96763b5db7ed60f1b00116f6ed748ec` |
| `logs/target-log/10_35_19.ulg` | 8563174 | `366fbea3ab9d322efe7e597161a32ad48e1caaf3a627b9bed990f715ced7ec96` |
| `logs/analysis/LOG00106_target_joint/metrics.json` | 8067 | `9b9a4c9c9d39bdd54d45c9279d3e75ef6363a4700675d24a0c09d10ef1334d59` |
| `logs/analysis/LOG00106_target_joint/joint_event_timeline.csv` | 4039 | `ff67b81eb712353024c0459936842ff2b01d8ce03ec80161f36332455cb09c2c` |
| `logs/analysis/LOG00106_target_joint/joint_timeseries_50hz.csv` | 107807 | `fee380cfc5fa12e753bf4440f2f026b11836baf7669066ed7164d6daa2da128c` |
| `logs/analysis/LOG00106_target_joint/joint_approach_timeline.png` | 257206 | `1f62c4fb5001dd684abf7ec17184a57ad498c2b7328a43a60182fbbf3da06f54` |
| `logs/analysis/LOG00106_target_joint/joint_impact_timeline.png` | 217243 | `384a7a50c0f1419e8a4b531653352c6d668c2f295ce4d9bba28d97577f5c77b7` |
| `doc/evidence/BETAFLIGHT_LOG00106_CONTROL_RESPONSE_metrics.json` | 4816 | `b37bff26e3250117d9a8664a6bccddad625360587570e1940bb18adf012ea2a4` |
| `doc/figures/log00106_control_response/01_angle_rate_tracking.png` | 578327 | `219a3921355c0e693476124dc2632e30c7b09b060604c6e2911fb4d2e0e80feb` |
| `doc/figures/log00106_control_response/02_throttle_and_load_response.png` | 587044 | `6aee4a635ea303686e9ac8b5d94cea34bb933cff25e4e7aff0efadb32f6d65db` |
| `doc/figures/log00106_control_response/03_guidance_closure_timeline.png` | 249859 | `e79e6644c43543defae653a40ca6236df31ca89679184f78daacb5245ea15a0d` |
| `doc/BETAFLIGHT_FLIGHT_ACTIVE_LOG00106_COLLISION_ANALYSIS.md` | 23499 | `272dc8b57f3da8a8ac2ff65198c0a547d2e6c95f639b5ae3be87e2562100ee55` |
| `doc/BETAFLIGHT_LOG00106_EXPECTED_ACTUAL_RESPONSE_REPORT.md` | 9439 | `1b694d7831f6219b4d4ec68a9a35067b18d641f592da299e747af935de7a22b4` |
| `config/betaflight.rk3588.velocity_png.flight_supervised.json` | 10718 | `4032482c47496c89d52f3cb09bfcef04b6c028de9012d10433d0aed4751865ff` |

配置文件当前 SHA256 与本架次 meta 记录一致。模型文件未作为本次 PC 归档清单重新哈希；本文第
4.3 节的模型 SHA256 来自本架次 meta。

## 15. 现有图表和报告入口

- [完整碰撞联合分析](BETAFLIGHT_FLIGHT_ACTIVE_LOG00106_COLLISION_ANALYSIS.md)
- [期望指令与实际响应报告](BETAFLIGHT_LOG00106_EXPECTED_ACTUAL_RESPONSE_REPORT.md)
- [角速度跟踪图](figures/log00106_control_response/01_angle_rate_tracking.png)
- [油门、比力与动力响应图](figures/log00106_control_response/02_throttle_and_load_response.png)
- [导引、惯性闭合与接触时序图](figures/log00106_control_response/03_guidance_closure_timeline.png)
- [双机接近过程图](../logs/analysis/LOG00106_target_joint/joint_approach_timeline.png)
- [双机冲击图](../logs/analysis/LOG00106_target_joint/joint_impact_timeline.png)

## 16. 可直接交给编码智能体的提示词

```text
你是 AirSim/飞行控制数据对比工程师。参考工作目录：
/home/linux/Documents/PNG-betaflight-upward-camera

目标：基于真实外场最后一架次 LOG00106，建立只作用于 AirSim 的 LOG_ONLY 趋势复现和对比。
先阅读 doc/BETAFLIGHT_LOG00106_AIRSIM_HANDOFF.md、
doc/BETAFLIGHT_FLIGHT_ACTIVE_LOG00106_COLLISION_ANALYSIS.md、
doc/BETAFLIGHT_LOG00106_EXPECTED_ACTUAL_RESPONSE_REPORT.md，随后读取文档列出的主 CSV、meta、
LOG00106.BFL、靶机 10_35_19.ulg 和 logs/analysis/LOG00106_target_joint 产物。

硬约束：
1. 先只做 LOG_ONLY 仿真。LOG_ONLY 在这里表示不得连接或写入真实飞控、不得发送
   MSP_SET_RAW_RC、不得使用 --allow-control；AirSim 内的模拟飞行器可以闭环运动。
2. 不擅自修改实机 runner、实机控制代码、实机配置、批准文件或原始日志。若现有 AirSim runner
   不足，新增独立的 AirSim 配置/脚本/报告，并尽量复用 vision_guidance 中的生产几何、LOS、
   VelocityEstablishingPngController 和 accel_tilt_rate 映射，避免复制一套公式后产生漂移。
3. 固定使用：N=3、fixed_vm=10 m/s、max_guidance_accel=7 m/s2、图像 640x512。
4. 相机必须使用实际参数：fx=530.8443137412、fy=532.2954942356、
   cx=321.0278689412、cy=247.2573194658；OpenCV 相机系 x右/y下/z光轴；
   R_BC=[[0,1,0],[1,0,0],[0,0,-1]]，光轴指向 FRD 机体 -Z。使用实际畸变参数时必须执行与
   实机相同的去畸变；若用理想针孔图像，也要使用上述 LOS 内参。
5. 统一输出 NED 惯性系和 FRD 机体系。NED 为北/东/下，上升是 D<0；不得混用 Betaflight FLU、
   MSP Pitch 显示符号和 FRD Pitch。`accel_tilt_rate` 必须复用生产实现：它按当前 yaw 旋转
   NED 加速度后计算期望 Roll/Pitch，并非直接用完整 `R_IB.T @ a_NED` 计算姿态。
6. 不使用两机绝对 GPS 差计算真实脱靶距离。AirSim 可以报告自身 truth miss distance，但必须与
   真实数据的接触锚定位置增量分栏。
7. 碰撞后的 963 deg/s、电机 158/2047 和 82.06 A 是接触后的飞控恢复，不得当成 PNG 输出，
   不得纳入 PNG 增益或饱和率拟合。

实现两个主工况：
A. 理想静止目标基线：靶机平移和姿态固定；视觉 30 Hz、控制 50 Hz；无漏检、无额外噪声和
   额外时延；保留控制器自带的 0.8 s 角速度入场和 0.8 s 油门交接；推力理想遵循
   1275 us=1 g、1500 us=2.37 g。
B. 实测敏感性：相同静止目标，注入真实感知结果年龄（优先重放主 CSV，近似时使用
   P50/P95/max=49.18/78.47/84.58 ms）、姿态融合等待（P95/max=40.12/52.85 ms，不能与结果
   年龄简单独立相加）、Blackbox 同钟 setpoint->gyro 15 ms 延迟、5 Hz MSP 运动学更新和
   0.25 s 速度滤波。交接完成后的实际比力使用模型的 0.809 倍为主工况、0.847 倍为乐观边界，
   保留 22.65--22.95 V 标签。

初始条件：拦截机位置设为 NED [0,0,0]。主工况用首次 measured LOS
[-0.030416,+0.066726,-0.997308] 和约 5.06 m 距离，靶机相对位置
[-0.1539,+0.3376,-5.0466] m；距离敏感性另跑约 6.34 m，即
[-0.1928,+0.4230,-6.3227] m。拦截机物理初速度
[-0.7603,+0.1741,+0.1700] m/s，控制器初始滤波速度
[-0.7748,+0.1850,+0.1438] m/s。真实靶机初速度
[-0.0177,-0.0041,-0.0436] m/s，理想基线可设零。拦截机 FRD/NED 初始姿态约
Roll=+0.2 deg、Pitch=-3.0 deg、Yaw=333 deg。调整 AirSim 目标模型视觉尺度，使首帧投影框接近
[318.303,166.230,367.189,210.729]，但不要把调整后的模型尺寸宣称为真实靶机尺寸。
固定上视相机若使用仓库 `run_airsim_strapdown_vision_png.py`，命令行传
`--camera-pitch-deg -90`；该 runner 内部会以相反符号调用 AirSim API。最终必须用投影方向和
`R_BC=[[0,1,0],[1,0,0],[0,0,-1]]` 做一致性断言。

每个工况都要分别运行两种退出方式：
1. 在 t=1.670804 s 停止算法，再继续至少 1.2 s，观察惯性闭合；
2. 不提前停止，保持模拟闭环直到接触、最近点或安全超时。
不要把两种结果混在一个命中结论中。

CSV 必须按交接文档第 11 节输出字段，至少包括：双机 truth NED 位置/速度、控制器观测速度、
图像框和面积、truth/measured/filtered LOS、LOS rate、速度参考、速度建立/PNG/FOV/总加速度
N/E/D、每项饱和状态、FOV priority、期望姿态、角速度设定和实际角速度、模型目标油门、交接
油门、实际油门、模型负载、实际比力、接触和最近距离。字段名必须带坐标系和单位。

必须画并对比：
- Roll/Pitch 角速度 setpoint 与实际响应；
- 模型目标油门、交接/实际油门和实际比力；
- NED 速度参考、实测/模拟速度；
- 速度建立、PNG、FOV、总加速度的 N/E/D 分量和模长；
- LOS、LOS rate、框中心和框面积；
- 各项饱和状态、算法 ACTIVE/退出、接触时刻和接触前剩余距离。

趋势验收：不要求数值完全相等。应验证上视目标产生 NED D<0 的参考速度/加速度；速度建立项
初段主导并较早饱和；总加速度频繁达到但不超过 7 m/s2；PNG/FOV 分项按 LOS/框偏移改变方向；
Roll/Pitch 命令和响应同号、高正相关并有约 15 ms 滞后；油门从 1303 us 平滑上升且不超过
1500 us；推力偏差工况相对理想工况具有更低比力、更慢速度建立和更晚接触/更大最近距离。
LOS rate 不必单调趋零，目标静止时仍会受拦截机平移、姿态变化、延迟和终端几何影响。

输出：独立 AirSim 配置、运行脚本、逐时刻 CSV、metrics JSON、中文趋势对比报告和图。报告必须
列出运行命令、seed、配置 SHA256、原始输入 SHA256、AirSim 版本/模式、是否使用真值框或真实
检测器，并严格区分实测、仿真、推断。先运行单元测试和一个短时 smoke test，再运行完整工况。

结论限制：LOG00106 只有一个物理接触样本。无论 AirSim 单次是否命中，都不得宣称真实命中率
达到 80%；概率命中率需要预定义场景分布、多个 seed 和独立真实飞行样本另行验证。
```
