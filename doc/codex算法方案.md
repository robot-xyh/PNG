# 算法方案优化版（安全审查与非武器化验证）

## 1. 文档定位

本文基于原始“算法方案”和后续隐患分析，对系统架构进行安全化、工程化重构。

本版文档的目标是：

- 修正原方案中容易导致系统发散、误控和失效的导航与视觉融合缺陷。
- 将 RK3588 侧职责限定为感知、时间戳对齐、状态估计评估和安全监督。
- 以 Pixhawk 内部 EKF 作为姿态、速度和位置的权威来源。
- 支持仿真、台架、HIL 和非武器化飞行数据回放验证。

本版文档不提供可直接用于自主攻击、动能撞击或末端追撞的部署步骤、参数和控制闭环。

---

## 2. 主要架构调整

原方案的核心问题不是单一参数设置不当，而是导航、视觉和控制边界划分不清：

- Python 侧重复实现惯导对准，和 Pixhawk EKF 输出互相竞争。
- 视觉检测结果没有按曝光时间和姿态历史对齐。
- LOS 滤波器将 bbox 尺寸、距离和距离变化率强耦合，容易被检测框抖动污染。
- 外部程序将加速度积分成速度指令，存在 wind-up 风险。
- 跟踪目标按最高置信度切换，容易被瞬时误检扰动。

优化后的系统边界如下：

```text
Camera / Sensor
    |
    | frame + exposure timestamp
    v
RK3588
    - YOLO / ByteTrack
    - 姿态历史时间戳对齐
    - LOS 稳定性估计
    - 跟踪质量评估
    - 安全状态机
    |
    | advisory output / logged setpoint / supervised command
    v
Pixhawk
    - EKF2 / EKF3 姿态、速度、位置融合
    - 飞行模式、安全保护、人工接管
```

工程原则：

- Pixhawk EKF 是唯一导航真值源。
- 视觉链路必须显式处理延迟。
- bbox 不作为高可信测距源。
- 控制输出必须经过飞控、限幅、模式和安全监督。
- 任何视觉失效、姿态失配或通信异常都应进入安全状态，而不是继续闭环追踪。

---

## 3. 删除 Python 侧自定义惯导对准

### 3.1 原设计问题

原方案中包含 `CoarseAlignment` 和 `FineAlignment`：

- 粗对准使用加速度计和磁力计估计姿态。
- 精对准使用 GPS 速度辅助修正航向。

在使用 Pixhawk 的系统中，这种设计不应作为主导航链路：

- Pixhawk 内部 EKF2/EKF3 已融合 IMU、GPS、磁罗盘、气压计等多源数据。
- 飞控内部 IMU 频率和时间同步质量显著优于 Python 侧串口采样。
- RK3588 通过 UART 接收的数据存在传输延迟、丢包和调度抖动。
- Python 侧手写对准结果可能和 Pixhawk 姿态解算不一致，造成坐标系冲突。

### 3.2 优化设计

删除 Python 侧惯导对准模块，将启动流程改为等待 Pixhawk 状态满足条件：

- MAVLink 链路稳定。
- Pixhawk EKF 状态 healthy。
- GPS 至少达到 3D Fix，且位置/速度估计稳定。
- 姿态消息连续更新，无明显时间戳跳变。
- 本机时钟和飞控时间戳关系已完成初始化。

推荐状态机：

```text
BOOT
  -> CONNECT_MAVLINK
  -> WAIT_EKF_HEALTHY
  -> WAIT_SENSOR_SYNC
  -> SEARCH_OR_OBSERVE
  -> TRACK_EVALUATE
  -> SAFE_HOLD / ABORT
```

### 3.3 MAVLink 数据源

RK3588 侧应订阅并记录以下数据：

- 姿态：`ATTITUDE` 或 `ATTITUDE_QUATERNION`
- 本地位置与速度：`LOCAL_POSITION_NED`
- GPS 状态：GPS fix type、卫星数、HDOP/VDOP 或等效质量指标
- EKF 健康状态：飞控提供的 estimator status / health flags
- 飞行模式、解锁状态和 failsafe 状态

注意：

- 原方案中 `UART 57600` 对高频姿态和控制数据偏紧，应在实际工程中评估更高波特率或更可靠链路。
- 文档层面只要求记录和健康判断，不将其写成自主攻击控制通道。

---

## 4. 姿态历史 Ring Buffer 与图像时间戳对齐

### 4.1 原设计问题

原方案在 LOS 更新中直接使用当前姿态：

```text
los_I_measured = R_IB(now) * R_BC * los_C
```

但相机曝光、ISP、NPU 推理和跟踪都会引入延迟。若用当前姿态转换几十毫秒前曝光的图像，在高机动条件下会制造虚假的 LOS 角速度。

这种问题不是简单低通滤波能解决的，因为错误来自时间坐标不一致。

### 4.2 优化设计

建立姿态历史环形缓冲区：

- 每次收到 Pixhawk 姿态时，保存 `timestamp`、姿态四元数或旋转矩阵、速度估计和质量标志。
- 缓冲长度至少覆盖最近 1 秒。
- YOLO / ByteTrack 输出检测结果时，必须携带对应图像帧的曝光时间戳。
- LOS 坐标转换时，使用曝光时刻插值得到的历史姿态。

数据流：

```text
Pixhawk attitude stream
    -> AttitudeHistoryBuffer[t, R_IB, velocity, quality]

Camera exposure
    -> frame_id + exposure_timestamp
    -> detection / tracking
    -> bbox + track_id + exposure_timestamp
    -> lookup R_IB(exposure_timestamp)
    -> los_I
```

### 4.3 插值与异常处理

姿态插值要求：

- 四元数姿态使用球面线性插值或等效稳定方法。
- 速度可使用线性插值。
- 禁止使用未来姿态补过去图像。
- 若曝光时间戳早于缓冲区最早数据或晚于最新姿态数据，应丢弃该视觉更新。

异常处理：

- 姿态缓存断流：进入安全状态。
- 时间戳跳变：暂停视觉更新，重新同步。
- 图像延迟超过阈值：只记录，不进入闭环评估。
- 姿态质量不健康：不使用视觉测量更新。

---

## 5. LOS 滤波器降维与 bbox 解耦

### 5.1 原设计问题

原方案使用 8 维状态：

```text
X = [lambda_x, lambda_y, lambda_z,
     lambda_dot_x, lambda_dot_y, lambda_dot_z,
     r, r_dot]
```

并将 `bbox_size` 放入观测方程估计距离。

这种做法的问题：

- YOLO bbox 会因姿态、遮挡、曝光、模型抖动产生高频 jitter。
- bbox 与真实距离并非稳定单调关系，尤其在目标姿态变化时。
- 非线性测距误差会污染 `r` 和 `r_dot`。
- `r/r_dot` 进一步污染 LOS 估计，使 LOS 角速度输出振荡。

### 5.2 优化设计

主 LOS 滤波器改为 6 维：

```text
X = [lambda_x, lambda_y, lambda_z,
     lambda_dot_x, lambda_dot_y, lambda_dot_z]
```

其中：

- `lambda` 是惯性系单位视线向量。
- `lambda_dot` 是视线单位向量变化率。
- 测量量是经过曝光时间姿态对齐后的 `lambda_I_measured`。
- 每步预测和更新后都必须归一化 `lambda`。
- `lambda_dot` 应投影到 `lambda` 的垂直平面，避免出现非物理径向分量。

### 5.3 bbox 的保守用途

bbox 不进入主 LOS 滤波器。

允许将 bbox 作为低可信辅助信号，用于：

- 视觉质量评估。
- 粗略接近趋势判断。
- 目标尺度异常检测。
- 近距离视觉失效风险判断。
- 日志分析和离线评估。

bbox 使用要求：

- 使用面积或高度前先低通滤波。
- 不将单帧 bbox 变化解释为真实距离变化。
- bbox 尺度突变时应降低视觉测量权重或丢弃该帧。
- bbox 只能作为状态机的辅助输入，不能作为高精度测距主来源。

---

## 6. 跟踪目标管理

### 6.1 原设计问题

原方案使用类似逻辑：

```text
target_track = max(tracks, key=lambda t: t.score)
```

这种策略会在多目标、误检、云层边缘、鸟类或噪声框出现时发生目标跳变。

目标跳变会导致：

- LOS 向量突变。
- LOS 角速度尖峰。
- 滤波器协方差异常。
- 下游控制或评估输出不可信。

### 6.2 优化设计

目标管理应使用 ID 锁定：

```text
SEARCH:
  选择满足质量门限的候选 track
  连续确认 M 帧后锁定 target_id

TRACK:
  只接受 target_id 对应的 track
  不因其他目标 score 更高而切换

COAST:
  target_id 短时丢失时，保持预测但不接受其他目标接管

REACQUIRE:
  连续丢失超过阈值后，解除 target_id
  重新进入 SEARCH
```

质量门限建议包含：

- 检测置信度。
- track age。
- bbox 尺度连续性。
- 像素速度连续性。
- 类别一致性。
- 画面边缘裁切状态。

安全要求：

- 任何目标切换都必须重置或降权 LOS 滤波器。
- 不能在同一滤波状态中无缝拼接两个不同 track。
- 目标丢失期间不得使用虚构视觉测量更新滤波器。

---

## 7. PNG 计算的安全化边界

### 7.1 原设计问题

原方案将 PNG 加速度积分为速度指令：

```text
v_cmd = current_velocity + a_cmd * dt
```

问题包括：

- 飞控无法瞬时响应速度变化时，外部积分器会 wind-up。
- 外部积分器不知道飞控姿态、推力和角度限制。
- 重力补偿、机体约束和飞控内部控制律边界不清。
- 多个控制环叠加容易造成不可预测响应。

### 7.2 优化原则

在安全审查和非武器化验证中，PNG 输出应优先作为：

- 离线评估量。
- 仿真输入。
- 受监督的 advisory setpoint。
- 安全状态机参考量。

不建议由 RK3588 直接闭环执行未经监督的追踪控制。

### 7.3 若用于受控飞行实验

若在合法、封闭、非武器化测试环境中做飞行控制实验，应满足：

- 飞控保持最终控制权。
- 指令必须经过限幅、模式检查和人工接管保护。
- 地理围栏、速度限制、高度限制和 failsafe 必须启用。
- 视觉失效时进入安全模式，而不是盲目继续追踪。
- 所有输出先记录，再逐级开放，从仿真到 HIL 到低风险飞行。

控制接口原则：

- 避免 RK3588 侧自建速度积分器。
- 优先使用飞控支持的受约束 setpoint 接口。
- 重力、姿态、推力和角度限制应由飞控内环统一处理。
- 外部程序只发送经过安全监督的期望量。

---

## 8. 末端视觉失效处理

### 8.1 原设计问题

近距离时目标可能快速填满画面，YOLO / ByteTrack 会出现：

- bbox 裁切。
- 类别置信度下降。
- 中心点跳变。
- track 丢失。
- 延迟占比变大。

原方案提出的末端“盲飞姿态锁定”不应作为安全设计保留，因为它会在视觉失效后继续执行不可校验的末端动作。

### 8.2 优化设计

近距离视觉失效应触发安全终止或人工监督，而不是继续末端闭环。

触发条件可包括：

- bbox 面积占比超过阈值。
- bbox 被画面边缘裁切。
- track 连续性下降。
- 视觉延迟超过阈值。
- LOS 创新持续异常。
- 姿态缓存不可用。

触发后的允许动作：

- 停止使用视觉更新。
- 冻结并记录最后可信状态用于离线分析。
- 退出自主视觉闭环。
- 切换到安全飞行模式或人工接管。

---

## 9. 推荐数据结构

### 9.1 图像帧

```text
FramePacket:
  frame_id
  exposure_timestamp
  receive_timestamp
  width
  height
  camera_id
```

### 9.2 检测跟踪结果

```text
TrackMeasurement:
  frame_id
  exposure_timestamp
  track_id
  class_id
  score
  bbox_xyxy
  bbox_area_ratio
  is_clipped
  tracker_state
```

### 9.3 姿态历史

```text
AttitudeSample:
  timestamp
  quaternion_IB
  R_IB
  velocity_ned
  ekf_healthy
  gps_fix_ok
```

### 9.4 LOS 估计输出

```text
LOSEstimate:
  timestamp
  lambda_I
  lambda_dot_I
  innovation_norm
  quality
  valid_for_control
```

其中 `valid_for_control` 在本安全审查版本中只表示“可用于受监督评估”，不表示可直接用于自主追踪控制。

---

## 10. 状态机

推荐状态机：

```text
BOOT
  系统启动，加载配置。

CONNECT_MAVLINK
  建立飞控通信，检查消息频率和时间戳。

WAIT_EKF_HEALTHY
  等待 Pixhawk EKF、GPS、姿态和速度估计稳定。

WAIT_SENSOR_SYNC
  初始化相机时间戳、姿态缓存和延迟统计。

OBSERVE
  运行检测、跟踪和日志记录，不输出控制。

TRACK_EVALUATE
  锁定 track_id，进行 LOS 稳定性评估。

SUPERVISED_OUTPUT
  仅在测试许可和安全条件满足时输出受监督 setpoint。

SAFE_HOLD
  停止视觉闭环输出，保持安全飞行模式。

ABORT
  链路、姿态、视觉或飞控状态异常时进入。
```

状态切换关键条件：

- EKF unhealthy：任何状态立即退出到 `SAFE_HOLD` 或 `ABORT`。
- MAVLink 超时：立即退出自主视觉输出。
- 姿态缓存查找失败：丢弃视觉测量，连续失败则退出。
- 目标 ID 丢失：进入 `COAST` 或回到 `OBSERVE`。
- 视觉质量低：停止更新 LOS 滤波器。
- 人工接管：最高优先级。

---

## 11. 参数建议（安全审查用途）

以下参数只用于稳定性评估和日志验证，不构成实飞攻击控制参数。

```text
attitude_buffer_duration: 1.0 s
max_visual_latency_for_update: 100 ms
min_track_confirm_frames: 3-5
max_track_lost_frames: 5-10
los_filter_rate: match valid visual updates
control_eval_rate: 20-50 Hz for logging only
bbox_lowpass_alpha: 0.1-0.3
innovation_reject_threshold: based on calibration data
```

标定参数：

- 相机内参必须通过实测标定获得。
- `R_BC` 必须通过安装标定确认，不能只使用名义 45 度。
- 相机曝光时间戳和系统单调时钟必须建立固定关系。
- Pixhawk 时间戳和 RK3588 时间戳需要记录偏移和漂移。

---

## 12. 拦截场景双方案评估

本节中的“拦截场景”仅指仿真、回放、台架、HIL 和非武器化受监督飞行数据评估，不表示允许将视觉或雷达输出直接接入自主末端追撞控制。

### 12.1 方案一：纯视觉方案

纯视觉方案只依赖捷联相机、YOLO/ByteTrack、Pixhawk 姿态和时间戳对齐。核心原则是将“角度通道”和“接近进程通道”解耦。

#### 8D 卡尔曼滤波：角度与距离强耦合

- 原理：状态同时估计 `lambda`、`lambda_dot`、`r`、`r_dot`，观测同时使用 bbox 中心点和 bbox 尺度。
- 致命缺陷：单目相机对角度敏感，对距离迟钝；bbox 抖动会被误解释为距离和速度突变。
- 污染路径：`r/r_dot` 的高噪声会通过协方差非对角项污染本来稳定的 `lambda_dot`。
- 工程结论：坚决不作为主方案，只保留为离线对照实验。

#### 6D 卡尔曼滤波：纯角度解耦

原理：

- 把单目相机当作测角传感器，只估计惯性系视线单位向量 `lambda_I` 和变化率 `lambda_dot_I`。
- 状态为 `[lambda_x, lambda_y, lambda_z, lambda_dot_x, lambda_dot_y, lambda_dot_z]`。
- 观测只使用 bbox 中心点 `(u, v)`，不使用 bbox 尺度、面积或目标真实尺寸。
- bbox 中心经相机内参得到 `los_C`，再经 `R_BC` 和曝光时刻历史姿态 `R_IB(t_exposure)` 得到 `lambda_I_measured`。
- 更新后归一化 `lambda`，并把 `lambda_dot` 投影到 `lambda` 的垂直平面。

优势：

- 与相机测角本质匹配。
- 不受 bbox 尺度抖动直接污染。
- 不需要假设目标真实大小。
- 能输出稳定的 LOS 角速度、innovation 和质量指标。

劣势：

- 不知道绝对距离，也不直接知道闭合速度。
- 不能区分远处高速目标和近处低速目标造成的相似角速度。
- 纯横穿或面积变化弱时，不能可靠判断接近进程。
- 强依赖曝光时间戳、Pixhawk 历史姿态、相机内参和外参标定。
- 目标太小、画面边缘畸变、遮挡、裁切或 track_id 切换都会降低可信度。

边界限制：

- 目标必须持续可见，且 bbox 中心稳定。
- 高机动目标会让 `lambda_dot` 快速变化，滤波器需要更高过程噪声，但会更敏感于检测抖动。
- 高速迎近目标需要 TTC、雷达或其他距离/速度来源补强，因为 6D LOS 本身不给接近时间。
- 姿态缓存失败、时间戳跳变、LOS innovation 超限时，该帧必须拒绝更新。

工程结论：

- 必须保留，作为纯视觉方案的角度主通道。
- 不能单独承担完整接近进程估计。
- 必须与 TTC 通道、track_id 管理、时间戳健康检查和安全状态机组合使用。

#### 纯角度制导与尺度膨胀：Scale Expansion / TTC

- 原理：不估计绝对距离，只估计接近时间趋势 `TTC`。
- 面积模型：目标接近时图像面积 `A` 膨胀，若 `A_dot > 0`，可用：

```text
TTC ~= 2 * A_filt / max(A_dot_filt, epsilon)
```

- 优势：不依赖目标真实尺寸，使用面积相对变化获得接近进程。
- 难点：`A_dot` 对 bbox jitter 极敏感，必须做低通滤波、异常剔除和裁切检查。
- 工程结论：这是单目视觉估计接近进程的优先方法，推荐与 6D LOS 滤波组合。

#### 纯视觉推荐融合架构

纯视觉融合架构应把相机最擅长的测角能力和 bbox 面积的相对膨胀趋势分开使用。角度通道只回答“目标方向如何变化”，尺度通道只回答“目标是否在快速接近”，二者通过质量门控组合，不互相污染状态。

```text
输入层：
  frame_id + exposure_timestamp
  bbox center (u, v)
  bbox area A
  track_id + track_quality
  Pixhawk historical attitude R_IB(exposure_timestamp)

角度通道：
  bbox center -> 曝光时间戳对齐 -> R_IB 去旋转 -> 6D LOS -> lambda_I, lambda_dot_I

接近进程通道：
  bbox area -> 低通滤波 -> 裁切/突变剔除 -> 面积导数 -> TTC -> TTC 质量

评估融合：
  质量门控通过后记录 g_eval = K(TTC) * lambda_dot_I
```

`g_eval` 只作为仿真、日志和受监督评估量。禁止将该候选量绕过安全状态机直接连接到高风险实飞控制输出。

主要特点：

- 角度与尺度解耦，bbox 面积抖动不会污染 `lambda_dot_I`。
- 不依赖目标真实尺寸，TTC 使用面积相对变化。
- 强依赖曝光时间戳、Pixhawk 历史姿态和 target_id 连续性。
- 输出是质量受控的评估量，不是无条件控制指令。

适用边界：

- 目标必须在视场内保持足够帧数，且 bbox 中心可稳定测量。
- 远距离小目标只有几像素时，TTC 不可信。
- 目标速度过低或纯横穿时，面积膨胀弱，TTC 会退化。
- 目标速度过高或交会时间过短时，可用帧数不足，面积导数容易被延迟和抖动主导。
- 目标大幅机动、强滚转、姿态剧变、遮挡或 bbox 裁切时，面积不再只反映距离变化，TTC 必须降权。

敌方无人机速度与运动限制：

- 匀速或低机动目标：最适合该架构。
- 中等机动目标：角度通道仍可工作，TTC 需要更严格质量门控。
- 高机动目标：纯视觉可做短时 LOS 评估，但不能可靠估计接近进程，需要雷达、双目或其他距离/速度来源补强。
- 快速迎近目标：面积膨胀明显但时间窗口短，要求高帧率、低延迟和稳定曝光时间戳。

工程边界：

- 纯视觉方案不能保证所有速度和机动包线下都有可靠闭合率估计。
- 不能把 `TTC` 当作真实距离，也不能把 `K(TTC)` 当作实飞控制增益直接使用。
- 当 `TTC_quality` 低、LOS innovation 高、track_id 不连续或姿态缓存失败时，系统必须进入记录/安全状态。

### 12.2 方案二：机载毫米波雷达无人机探测方案

本项目讨论的毫米波雷达不是人体存在检测、生命体征检测或高度计，而是安装在穿越机/无人机机体上的前向空中目标探测雷达。它应输出距离、径向速度、多普勒、方位/俯仰和质量指标，用于非武器化日志、融合评估和安全状态判断。

选型优先级：

```text
无人机探测能力 > 机载 SWaP > 视场/刷新率/接口/时间戳 > 价格
```

当前项目机载约束：

| 约束 | 推荐目标 | 说明 |
| --- | --- | --- |
| 重量 | 小穿越机 <150-200 g；大载重平台可到 700-900 g | EchoFlight / Fortem R20 更接近中大型无人机载荷 |
| 功耗 | 小穿越机 <10-15 W；大载重平台可到 40-50 W | 40 W 级雷达会显著影响续航、电源和散热 |
| 视场 | 前向至少覆盖相机主视场，优先 100° 级方位视场 | 窄视场需要严格安装对准 |
| 输出 | range、range_rate、azimuth、elevation、SNR/quality、timestamp | 只输出 presence 的模块不合格 |
| 安装 | 前向无遮挡，远离碳纤维遮挡和强电噪声 | 碳纤维、金属件、电池和桨叶会影响波束 |

适用于本项目的硬件选型：

| 推荐等级 | 代表硬件 | 公开价格参考 | 机载 SWaP | 无人机探测能力 | 技术成熟度 | 本项目判断 |
| --- | --- | --- | --- | --- | --- | --- |
| A：技术最匹配，但平台要求高 | Echodyne EchoFlight | RFQ，公开通常不标价 | 18.7 x 12 x 4 cm；817 g；12-28 V；45 W 工作，<10 W 热待机 | Phantom 4 级目标 >750 m，Matrice 600 级目标 >1 km；方位精度 <1°，俯仰 <1.5°，距离精度 <3.25 m | 专用机载 DAA 雷达 | 适合大载重穿越机或中大型无人机；普通 5-7 寸穿越机基本不适合 |
| A-：无人机探测明确，重量/功耗仍偏高 | Fortem TrueView R20 / R20i | RFQ | 200 x 75 x 38.3 mm；748 g；18-36 V 供电，功耗需询厂商 | 0.1 m² RCS 目标约 800 m；120° 方位视场，40° 俯仰视场；输出距离、速度、RCS 等 | 专用 C-UAS / DAA 雷达 | 若能采购 R20i 空中版本，是机载无人机探测候选；对小穿越机仍偏重 |
| B：研发验证首选 | TI AWR1843BOOST / AWR1843AOP 定制板 | TI 标价约 USD 313.95；DigiKey/Mouser 当前约 USD 469-483 | 开发板形态，需自制外壳、减振、供电和散热 | 车规中距雷达芯片，TI 资料称中距目标可到约 150 m，距离分辨率 <4 cm；空中无人机检测需自研算法 | 芯片成熟，空中小目标方案不成熟 | 适合先做机载数据采集和算法研究；不能视为成品无人机探测雷达 |
| B-：近距点云验证 | TI IWR6843ISK / IWR6843ISK-ODS | IWR6843ISK 约 USD 282.98；常需 MMWAVEICBOOST 约 USD 402.63 | 开发板形态，需外壳、减振、供电和数据链 | ISK 人员级目标约 75 m；ODS 广角约 12 m；对空中小无人机需重写检测跟踪 | 工业开发生态成熟 | 更适合台架和近距飞行数据验证，不适合当前项目作为远距无人机探测主传感器 |
| C：明确排除 | DFRobot SEN0395、Seeed MR60BHA1、Ainstein US-D1 / LR-D1 等 | USD 十几到数百不等 | 轻、小、低功耗 | 存在检测、生命体征或高度计；不是空中无人机探测雷达 | 各自领域成熟 | 不作为目标探测雷达，只能用于入门、测高或避障实验 |
| D：地面/车载外部真值 | EchoShield、EchoGuard、Robin IRIS 等 | RFQ，通常高价 | 多为地面/车载，不适合装在穿越机上 | 公里级 C-UAS 探测和分类 | 成熟 | 可作为外部真值/测试场传感器，不作为机载载荷 |

推荐结论：

- 当前 MVP 不挂成熟 DAA 雷达，继续以纯视觉 6D + TTC 为主，预留 `RadarMeasurement` 接口。
- 机载雷达研发验证优先 AWR1843BOOST/AOP 或 IWR6843ISK，只验证时间同步、点云和距离/径向速度质量。
- 若平台升级到可承载 800 g、45 W 级载荷，并且预算/合规允许，优先评估 EchoFlight；备选 Fortem TrueView R20/R20i。
- 低成本存在检测、生命体征和高度计模块从无人机探测候选中删除。

非武器化评估链路建议：

```text
视觉：bbox center -> 6D LOS -> lambda_I, lambda_dot_I
视觉尺度：bbox area -> TTC -> 接近趋势
雷达：range, range_rate, doppler, point cloud -> 距离/径向速度质量
融合评估：对比 TTC 与 radar range_rate，输出日志、置信度和安全状态
```

公开资料来源：

- TI IWR6843ISK：<https://www.ti.com/tool/IWR6843ISK>
- TI IWR6843ISK-ODS：<https://www.ti.com/tool/IWR6843ISK-ODS>
- DigiKey IWR6843ISK：<https://www.digikey.com/en/products/detail/texas-instruments/IWR6843ISK/10434492>
- TI AWR1843BOOST：<https://www.ti.com/tool/AWR1843BOOST>
- DigiKey AWR1843BOOST：<https://www.digikey.com/en/products/detail/texas-instruments/AWR1843BOOST/10445300>
- TI AWR1843 中距雷达说明：<https://www.ti.com/video/5990261460001>
- Echodyne Airborne Radar：<https://www.echodyne.com/uas-aam/airborne-radar>
- EchoFlight 第三方规格：<https://www.defenseadvancement.com/company/echodyne/echoflight-radar/>
- Fortem TrueView R20：<https://www.fortemtech.com/products/trueview-r20/>
- Fortem R20 AERPAW 规格：<https://sites.google.com/ncsu.edu/aerpaw-wiki/aerpaw-user-manual/5-future-platform-features/5-6-fortem-radar>
- DFRobot SEN0395：<https://www.dfrobot.com/product-2282.html>
- Seeed MR60BHA1：<https://wiki.seeedstudio.com/Radar_MR60BHA1/>
- Newark MR60BHA1：<https://www.newark.com/seeed-studio/101990886/respiratory-heartbeat-detection/dp/74AK7835>
- Ainstein US-D1：<https://ainstein.ai/us-d1-all-weather-radar-altimeter/>
- Echodyne EchoFlight：<https://www.echodyne.com/radar-systems/echoflight>
- Echodyne EchoShield：<https://www.echodyne.com/radar-systems/echoshield>

---

## 13. 测试与验收

### 13.1 单元测试

- 姿态 Ring Buffer 插值正确。
- 时间戳越界时拒绝视觉更新。
- `R_BC` 安装矩阵方向正确。
- LOS 向量始终归一化。
- `lambda_dot` 不含非物理径向分量。
- bbox 抖动不会进入主 LOS 滤波状态。
- track_id 切换会重置或降权滤波器。

### 13.2 回放测试

使用记录数据回放：

- 固定姿态场景。
- 高机动姿态场景。
- 人为注入 30-50ms 视觉延迟。
- 人为注入 bbox jitter。
- 多目标误检和短时遮挡。
- MAVLink 丢包和姿态缓存断流。

验收标准：

- 时间对齐开启后，LOS 角速度尖峰明显降低。
- bbox 抖动不会导致 LOS 主状态发散。
- 目标误检不会触发无约束目标切换。
- EKF 或视觉异常时系统进入安全状态。

### 13.3 HIL / 台架测试

台架测试只验证数据链和状态机：

- Pixhawk EKF 状态读取。
- MAVLink 消息频率统计。
- 相机帧时间戳一致性。
- 检测延迟统计。
- 姿态缓存命中率。
- 安全状态切换。

任何飞行测试都必须先通过仿真、回放和 HIL，且必须采用非武器化目标、人工接管和封闭测试区域。

---

## 14. 与原方案的对应修改清单

| 原方案内容 | 修改结果 |
| --- | --- |
| Python 粗对准 / 精对准 | 删除，改为等待 Pixhawk EKF healthy |
| 当前姿态直接转换图像 LOS | 改为曝光时间戳查姿态历史 |
| 8 维 LOS + bbox 测距 EKF | 改为 6 维 LOS 主滤波，bbox 解耦 |
| bbox_size 估计距离和距离率 | 仅作为低可信辅助质量信号 |
| 最高 score 选择目标 | 改为 track_id 锁定和丢失重捕获 |
| 加速度积分为速度指令 | 禁止外部 wind-up 积分，改为受监督 setpoint 原则 |
| 末端盲飞撞击 | 改为视觉失效安全退出 |
| 50Hz 自主追踪闭环 | 改为日志评估、仿真、HIL 和受监督输出 |

---

## 15. 结论

优化后的方案将重点从“直接自主拦截控制”转为“可靠感知、时间对齐、状态估计和安全监督”。

关键收益：

- 避免 Python 侧重复惯导对准造成坐标系冲突。
- 消除视觉延迟和当前姿态混用导致的伪 LOS 角速度。
- 防止 bbox 抖动污染主 LOS 状态。
- 降低目标误检造成的目标跳变风险。
- 避免外部速度积分 wind-up。
- 在视觉、飞控或通信异常时优先进入安全状态。

该版本适合作为后续仿真、台架、数据回放和非武器化飞行验证的设计基线。
