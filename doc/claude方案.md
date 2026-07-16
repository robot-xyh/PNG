# Claude 统一方案：视觉导航安全评估与非武器化验证系统

> 来源：整合“最终统一方案”各分片、`codex算法方案.md` 与 `codex实施方案.md` 的可用工程内容。  
> 版本：v1.0 Safe Unified  
> 日期：2026-06-11  

---

## 0. 文档定位

本文是最终统一版方案，将原有多个分片中的架构、算法、工程实施、安全机制和测试验证合并为一个完整文档。

本版方案明确限定为：

- 视觉导航算法评估。
- 飞控状态接入与日志审计。
- 姿态时间戳对齐验证。
- LOS 稳定性评估。
- 目标跟踪鲁棒性评估。
- 仿真、台架、HIL 和非武器化飞行数据采集。

本方案不提供可直接用于自主攻击、末端追撞、动能撞击或规避安全机制的部署步骤。

---

## 1. 总体目标

### 1.1 目标

构建一个运行在 RK3588 上的视觉导航评估系统，接入 Pixhawk 飞控状态、捷联相机图像、YOLO 检测和 ByteTrack 跟踪结果，用于验证以下关键问题：

- 相机曝光时间与飞控姿态时间是否准确对齐。
- 捷联相机在机体姿态变化下的 LOS 估计是否稳定。
- bbox 抖动是否会污染 LOS 主状态。
- 目标 ID 管理是否能抵抗误检和短时丢失。
- MAVLink、相机、检测和日志链路是否满足可审计要求。
- 异常状态是否能及时退出到安全状态。

### 1.2 非目标

本方案不实现以下能力：

- 不实现自主末端追撞。
- 不提供攻击目标选择策略。
- 不提供动能拦截部署参数。
- 不绕过 Pixhawk 安全机制直接控制执行机构。
- 不把未经验证的视觉输出直接接入高风险闭环控制。

---

## 2. 系统架构

### 2.1 硬件组成

```text
捷联相机
  - 可见光或红外图像输入
  - 输出 frame_id 与曝光时间戳

RK3588
  - 图像采集
  - YOLO / ByteTrack
  - 姿态时间戳对齐
  - LOS 估计
  - 安全状态机
  - 日志记录与回放

Pixhawk
  - EKF2 / EKF3
  - 姿态、速度、位置估计
  - GPS / IMU / 磁罗盘融合
  - 飞行模式和 failsafe
```

### 2.2 软件模块

```text
drone_vision_eval/
├── config/
│   ├── camera.yaml
│   ├── mavlink.yaml
│   ├── tracking.yaml
│   ├── los_filter.yaml
│   └── safety.yaml
├── src/
│   ├── main.py
│   ├── timebase.py
│   ├── mavlink_state.py
│   ├── attitude_buffer.py
│   ├── camera_source.py
│   ├── detector.py
│   ├── tracker.py
│   ├── target_manager.py
│   ├── los_estimator.py
│   ├── safety_state_machine.py
│   └── logger.py
├── tools/
│   ├── replay_log.py
│   ├── inspect_timestamps.py
│   └── plot_los.py
└── tests/
    ├── test_attitude_buffer.py
    ├── test_timebase.py
    ├── test_target_manager.py
    └── test_los_estimator.py
```

### 2.3 责任边界

| 模块 | 责任 | 禁止事项 |
| --- | --- | --- |
| Pixhawk | 姿态、速度、位置估计；飞行模式；failsafe | 不由外部程序替代 EKF |
| RK3588 | 感知、估计、日志、安全监督 | 不直接绕过飞控安全机制 |
| 相机 | 图像和曝光时间戳 | 不使用无时间戳图像进入闭环评估 |
| YOLO / ByteTrack | 检测与跟踪 | 不按单帧最高分随意切换目标 |
| LOS 模块 | 视线稳定性估计 | 不使用 bbox 强耦合测距污染主状态 |

---

## 3. 核心架构修正

### 3.1 删除 Python 侧自定义惯导对准

原分片中包含粗对准和精对准代码，这类逻辑不应作为主导航链路使用。

原因：

- Pixhawk EKF 的 IMU 频率、传感器融合和时间同步优于 Python 串口侧处理。
- UART 传输存在延迟、丢包和系统调度抖动。
- RK3588 自研姿态结果可能和 Pixhawk EKF 冲突。
- 坐标系冲突会直接污染视觉 LOS 转换。

优化后启动条件：

```text
BOOT
  -> CONNECT_MAVLINK
  -> WAIT_EKF_HEALTHY
  -> WAIT_SENSOR_SYNC
  -> OBSERVE
  -> TRACK_EVALUATE
  -> SAFE_HOLD / ABORT
```

RK3588 仅等待并记录：

- Pixhawk EKF healthy。
- GPS fix 满足测试要求。
- 姿态、速度、位置消息频率稳定。
- 飞控模式和 failsafe 状态可读。
- 相机和飞控时间基准已完成同步或偏移估计。

### 3.2 使用 Pixhawk 作为唯一导航真值源

RK3588 应订阅：

- `ATTITUDE` 或 `ATTITUDE_QUATERNION`
- `LOCAL_POSITION_NED`
- GPS fix 状态
- estimator / EKF health
- 飞行模式、解锁状态、failsafe 状态

MAVLink 链路必须记录：

- 消息频率。
- 丢包率。
- 时间戳抖动。
- 最近有效消息时间。
- EKF 状态变化。

---

## 4. 时间戳与姿态对齐

### 4.1 问题

捷联相机的图像并不代表“当前时刻”。图像曝光、ISP、NPU 推理和跟踪输出之间存在延迟。

如果用当前姿态转换几十毫秒前曝光的图像，会产生伪 LOS 角速度，使后续估计不稳定。

### 4.2 姿态历史 Ring Buffer

建立姿态历史缓存：

```text
AttitudeSample:
  timestamp_mono
  timestamp_autopilot
  quaternion_IB
  R_IB
  velocity_ned
  position_ned
  ekf_healthy
  gps_fix_ok
```

接口：

```text
AttitudeHistoryBuffer:
  push(VehicleState)
  lookup(timestamp_mono) -> InterpolatedVehicleState
  stats() -> buffer_age, sample_rate, miss_count
```

要求：

- 缓存最近至少 1 秒姿态数据。
- 姿态插值使用四元数插值。
- 查询失败时丢弃该视觉更新。
- 不允许用未来姿态修正过去图像。
- 时间戳跳变时暂停视觉更新并重新同步。

### 4.3 图像时间戳

图像帧结构：

```text
FramePacket:
  frame_id
  exposure_timestamp_mono
  receive_timestamp_mono
  width
  height
  camera_id
```

检测结果必须继承原始图像的曝光时间戳：

```text
TrackMeasurement:
  frame_id
  exposure_timestamp_mono
  track_id
  class_id
  score
  bbox_xyxy
  bbox_area_ratio
  is_clipped
  tracker_state
```

若相机不支持真实曝光时间戳，只能进行固定延迟标定，并将该数据标记为低可信。

---

## 5. 目标检测与跟踪管理

### 5.1 检测链路

检测链路用于输出候选目标，不直接决定高风险动作。

基本流程：

```text
camera frame
  -> detector
  -> ByteTrack
  -> TargetManager
  -> TrackMeasurement
  -> log / LOS evaluation
```

### 5.2 目标 ID 锁定

禁止使用“当前最高 score 目标”作为持续目标。应使用目标 ID 锁定机制。

状态：

```text
SEARCH
  未锁定目标，只评估候选 track。

CONFIRM
  候选 track 连续满足质量门限。

LOCKED
  只接受已锁定 target_id。

COAST
  目标短时丢失，只做预测和日志，不接管其他目标。

REACQUIRE
  丢失超时后解除 target_id，重新搜索。
```

质量门限：

- 检测置信度。
- track age。
- bbox 未严重裁切。
- bbox 尺度连续。
- 像素中心运动连续。
- class_id 一致。
- 视觉延迟未超限。

目标切换要求：

- 目标 ID 改变时，LOS 滤波器必须重置或显著降权。
- 不允许把两个不同目标的观测拼接进同一滤波状态。
- 短时丢失期间不接受其他目标抢占。

---

## 6. LOS 估计

### 6.1 坐标定义

```text
I: 惯性系 / NED
B: 机体系
C: 相机系
```

转换流程：

```text
pixel center
  -> normalized image ray
  -> los_C
  -> los_B = R_BC * los_C
  -> los_I = R_IB(exposure_time) * los_B
```

要求：

- 相机内参必须通过标定获得。
- `R_BC` 必须通过安装标定确认。
- 坐标定义写入配置文件，避免硬编码。

### 6.2 主 LOS 滤波器

主滤波器使用 6 维状态：

```text
X = [lambda_x, lambda_y, lambda_z,
     lambda_dot_x, lambda_dot_y, lambda_dot_z]
```

输入：

- 时间对齐后的 `lambda_I_measured`
- `measurement_timestamp`
- track 质量指标

输出：

- `lambda_I`
- `lambda_dot_I`
- innovation norm
- quality
- valid_for_supervised_eval

约束：

- 每次更新后归一化 `lambda`。
- `lambda_dot` 投影到 `lambda` 的垂直平面。
- bbox 不进入主 LOS 滤波状态。
- 姿态缓存查找失败时拒绝该帧。

### 6.3 bbox 的保守用途

bbox 只作为辅助质量信号：

- 视觉质量评估。
- 尺度连续性检查。
- 近距离视觉失效风险判断。
- 日志分析。

bbox 不用于高可信测距，不用于主 LOS 状态更新，不用于直接生成高风险控制输出。

---

## 7. 飞控接口与输出边界

### 7.1 原方案风险

原分片中包含外部程序计算导引指令、积分成速度指令并发送给飞控的逻辑。这类设计存在：

- 外部积分 wind-up。
- 飞控内外环职责冲突。
- 未处理重力、推力、姿态角和模式约束。
- 视觉异常时继续输出不可校验指令的风险。

### 7.2 本方案边界

RK3588 输出限定为：

- 日志。
- 离线评估量。
- 仿真输入。
- 受监督 setpoint 候选值。
- 安全状态机事件。

不将视觉输出直接接入未经验证的高风险闭环控制。

### 7.3 受监督输出原则

若在合法、封闭、非武器化测试环境中进行受监督 setpoint 试验，必须满足：

- Pixhawk 保持最终控制权。
- 人工接管最高优先级。
- 地理围栏和 failsafe 已启用。
- 指令必须经过限幅、模式检查和安全状态机门控。
- 任一健康检查失败立即停止输出。
- 所有输出先记录、可回放、可审计。

---

## 8. 安全状态机

### 8.1 状态定义

```text
BOOT
CONNECT_MAVLINK
WAIT_EKF_HEALTHY
WAIT_SENSOR_SYNC
OBSERVE
TRACK_EVALUATE
SUPERVISED_OUTPUT
SAFE_HOLD
ABORT
```

### 8.2 状态说明

| 状态 | 含义 |
| --- | --- |
| BOOT | 系统启动，加载配置 |
| CONNECT_MAVLINK | 建立飞控通信 |
| WAIT_EKF_HEALTHY | 等待 EKF、GPS、姿态和速度状态稳定 |
| WAIT_SENSOR_SYNC | 初始化相机时间戳、姿态缓存和延迟统计 |
| OBSERVE | 只检测、跟踪、记录，不输出控制 |
| TRACK_EVALUATE | 锁定目标 ID 并评估 LOS 稳定性 |
| SUPERVISED_OUTPUT | 仅在受控测试中输出受监督候选 setpoint |
| SAFE_HOLD | 停止视觉输出，保持安全飞行模式 |
| ABORT | 链路、姿态、视觉或飞控异常 |

### 8.3 强制退出条件

任一条件成立，立即停止视觉闭环输出：

- EKF unhealthy。
- GPS fix 不满足测试条件。
- MAVLink 超时。
- 姿态缓存查询连续失败。
- 图像时间戳异常。
- 视觉延迟超限。
- 目标 ID 丢失超时。
- bbox 严重裁切。
- LOS 创新量持续超限。
- 人工接管。
- failsafe 触发。

### 8.4 日志要求

每次状态切换记录：

- 旧状态。
- 新状态。
- 触发原因。
- 时间戳。
- 飞控健康状态。
- 视觉质量状态。
- 最近一次有效目标状态。

---

## 9. 配置文件

### 9.1 camera.yaml

```yaml
camera:
  device_id: 0
  fps: 60
  resolution:
    width: 640
    height: 480
  timestamp:
    source: exposure
    fallback_fixed_delay_ms: null
  intrinsics:
    fx: null
    fy: null
    cx: null
    cy: null
  distortion:
    k1: null
    k2: null
    p1: null
    p2: null
    k3: null
  extrinsics:
    R_BC_source: calibration
```

### 9.2 mavlink.yaml

```yaml
mavlink:
  connection: "/dev/ttyAMA0"
  baudrate: 921600
  required_messages:
    - ATTITUDE_QUATERNION
    - LOCAL_POSITION_NED
    - GPS_RAW_INT
    - SYS_STATUS
  health:
    require_ekf_healthy: true
    require_gps_fix: true
  timeout_ms: 200
```

### 9.3 tracking.yaml

```yaml
tracking:
  detector:
    confidence_threshold: 0.6
    nms_threshold: 0.4
  target_manager:
    confirm_frames: 3
    max_lost_frames: 8
    require_same_class: true
    reject_clipped_bbox: true
```

### 9.4 los_filter.yaml

```yaml
los_filter:
  state_dim: 6
  use_bbox_in_state: false
  normalize_lambda: true
  project_lambda_dot: true
  max_visual_latency_ms: 100
  attitude_buffer_duration_s: 1.0
```

### 9.5 safety.yaml

```yaml
safety:
  modes:
    default: OBSERVE
    allow_supervised_output: false
  abort_on:
    ekf_unhealthy: true
    mavlink_timeout: true
    timestamp_jump: true
    target_lost_timeout: true
    los_innovation_limit: true
  logging:
    record_all_rejections: true
```

---

## 10. 日志与回放

### 10.1 必须记录的数据

- 系统单调时间。
- 相机 frame_id。
- 图像曝光时间戳。
- 图像接收时间戳。
- 检测输出时间戳。
- bbox、score、class_id。
- track_id 和 track 状态。
- Pixhawk 姿态、速度、位置。
- EKF、GPS、failsafe 状态。
- 姿态缓存 lookup 结果。
- LOS 测量、LOS 滤波状态、创新量。
- 状态机状态与切换原因。
- 所有拒绝更新原因。

### 10.2 日志格式

建议：

- 高频数据使用 Parquet、MCAP、ROS bag 或二进制结构化格式。
- 低频事件使用 JSONL。
- 每次运行保存完整配置快照。
- 每条记录包含版本号和配置 hash。

### 10.3 回放能力

回放工具应支持：

- 按时间重建飞控状态。
- 重放相机帧和检测结果。
- 复现目标 ID 管理。
- 复现姿态缓存查询。
- 对比启用/禁用时间对齐的 LOS 结果。
- 输出异常帧和安全状态切换报告。

---

## 11. 实施路线

### 阶段 0：需求冻结与安全边界

交付物：

- 安全边界说明。
- 硬件连接图。
- 消息字段清单。
- 测试范围说明。

退出条件：

- 明确只做安全评估和非武器化验证。
- 明确不绕过 Pixhawk 安全机制。
- 明确人工接管和 failsafe 优先。

### 阶段 1：工程骨架与日志系统

工作：

- 建立工程目录。
- 实现配置加载。
- 实现 mock 模式。
- 实现结构化日志。
- 实现基础回放工具。

退出条件：

- 无相机、无 Pixhawk 时可用 mock 数据跑通。
- 日志可回放。
- 配置随日志保存。

### 阶段 2：MAVLink 与姿态缓存

工作：

- 接入 Pixhawk 状态。
- 实现 `VehicleState`。
- 实现 `AttitudeHistoryBuffer`。
- 统计消息频率、抖动和丢包。

退出条件：

- 姿态插值连续。
- 时间戳跳变可检测。
- 缓存查找失败时正确拒绝视觉更新。

### 阶段 3：相机、检测与目标管理

工作：

- 接入相机帧。
- 接入 YOLO / ByteTrack 输出。
- 实现 `TargetManager`。
- 实现目标确认、锁定、coast 和重捕获。

退出条件：

- 单目标稳定时 target_id 不变。
- 高分误检不抢占已锁定目标。
- 目标丢失超时后正确解除锁定。

### 阶段 4：LOS 估计与离线评估

工作：

- 实现像素到相机系 LOS。
- 使用曝光时间戳查询历史姿态。
- 实现 6 维 LOS 滤波。
- 实现 LOS 评估脚本。

退出条件：

- bbox jitter 不导致 LOS 主状态发散。
- 姿态时间对齐能降低伪 LOS 角速度尖峰。
- 目标 ID 切换时滤波器正确重置或降权。

### 阶段 5：HIL / 台架 / 非武器化飞行数据采集

工作：

- 台架验证 MAVLink、相机、检测和日志链路。
- HIL 验证状态机异常处理。
- 非武器化飞行中只采集数据并进行离线评估。

退出条件：

- 三组以上可复现日志。
- 无未解释时间戳跳变。
- 所有异常均触发预期安全状态。
- 回放结果和现场日志一致。

---

## 12. 测试计划

### 12.1 单元测试

- `AttitudeHistoryBuffer` 插值。
- 时间戳越界拒绝。
- 相机坐标转换。
- LOS 归一化。
- `lambda_dot` 垂直投影。
- bbox 不进入主 LOS 状态。
- `TargetManager` ID 锁定。
- 状态机强制退出。

### 12.2 回放测试

场景：

- 固定姿态。
- 匀速姿态变化。
- 高机动姿态变化。
- 注入 30-50ms 视觉延迟。
- 注入 bbox jitter。
- 多目标误检。
- 短时遮挡。
- MAVLink 丢包。
- 姿态缓存断流。

验收：

- 时间对齐后 LOS 尖峰减少。
- bbox jitter 不污染主状态。
- 误检不导致目标跳变。
- 异常触发安全状态。

### 12.3 HIL 测试

验证：

- EKF healthy / unhealthy 切换。
- MAVLink 超时和重连。
- GPS fix 降级。
- 图像时间戳跳变。
- 目标丢失。
- failsafe 触发。

### 12.4 非武器化飞行数据采集

要求：

- 使用非危险目标或标靶。
- 只采集数据，不进行高风险闭环控制。
- 人工遥控或飞控安全模式保持最终控制权。
- 启用地理围栏和 failsafe。
- 飞后离线评估。

---

## 13. 拦截场景双方案评估

本节中的“拦截场景”仅指仿真、回放、台架、HIL 和非武器化受监督飞行数据评估，不表示允许将视觉或雷达输出直接接入自主末端追撞控制。

### 13.1 方案一：纯视觉方案

纯视觉方案只依赖捷联相机、YOLO/ByteTrack、Pixhawk 姿态和时间戳对齐。它的核心困难是：单目相机天然擅长测角，但不擅长测距。因此应将“角度通道”和“接近进程通道”解耦。

#### 13.1.1 8D 卡尔曼滤波：角度与距离强耦合

原理：

- 状态同时估计 `lambda`、`lambda_dot`、`r`、`r_dot`。
- 观测同时使用 bbox 中心点和 bbox 尺度。
- 试图在一个滤波器中同时求视线角速度、距离和闭合速度。

主要问题：

- 单目图像对角度敏感，对距离迟钝。
- bbox 边缘因姿态、光照、遮挡或检测抖动变化几个像素时，会被错误解释为距离和速度突变。
- `r/r_dot` 的高噪声会通过协方差非对角项污染 `lambda_dot`。
- 为了估计不可靠距离，反而破坏最关键的 LOS 角速度。

工程结论：

- 不作为主方案。
- 可保留为离线对照实验，用于证明 bbox 测距强耦合的发散风险。
- 不进入受监督输出链路。

#### 13.1.2 6D 卡尔曼滤波：纯角度解耦

原理：

- 6D LOS 滤波把单目相机当作“测角传感器”，只估计惯性系视线单位向量 `lambda_I` 和它的变化率 `lambda_dot_I`。
- 状态向量为：

```text
X = [lambda_x, lambda_y, lambda_z,
     lambda_dot_x, lambda_dot_y, lambda_dot_z]
```

- 观测只使用 bbox 中心点 `(u, v)`，不使用 bbox 宽、高、面积或目标真实尺寸。
- 每个检测框中心先转成相机系单位视线，再结合曝光时间戳查到的历史姿态 `R_IB(t_exposure)`，转换成惯性系观测 `lambda_I_measured`。
- 预测时使用近似常角速度模型：

```text
lambda(k+1) ~= normalize(lambda(k) + lambda_dot(k) * dt)
lambda_dot(k+1) ~= lambda_dot(k)
```

- 更新后强制 `lambda` 归一化，并将 `lambda_dot` 投影到 `lambda` 的垂直平面，避免出现非物理的径向分量。

数据流：

```text
bbox center (u, v)
  -> 相机内参归一化
  -> los_C
  -> R_BC 安装外参
  -> los_B
  -> R_IB(t_exposure) 历史姿态去旋转
  -> lambda_I_measured
  -> 6D Kalman Filter
  -> lambda_I, lambda_dot_I, innovation, quality
```

优势：

- 与相机“测角仪”本质匹配。
- 不受 bbox 尺度抖动直接污染。
- 能输出稳定、平滑、物理一致的 `lambda_dot`。
- 不需要假设目标真实大小。
- 不需要单目绝对测距。
- 可以清晰地用 innovation、协方差和时间戳质量判断该帧是否可信。

劣势：

- 不知道绝对距离。
- 不直接知道闭合速度。
- 若在仿真中固定闭合速度，会导致不同交会角下等效增益变化很大。
- 不能区分“远处高速目标”和“近处低速目标”造成的相似角速度。
- 纯角度信息对接近进程不敏感，尤其在目标近似横穿、面积变化弱时。
- 目标短暂丢失后，只靠角速度模型外推会快速失去可信度。
- 强依赖时间戳对齐；曝光时间和姿态时间错位会直接制造伪 `lambda_dot`。
- 强依赖相机内参、外参和安装刚性；安装角误差会变成系统性 LOS 偏差。
- 目标只占少量像素时，中心点量化误差会放大成角度噪声。

边界限制：

- 目标必须持续可见，且 bbox 中心要稳定；严重遮挡、裁切、误检或 track_id 切换时必须重置或降权。
- 目标距离很远、bbox 只有几个像素时，6D 滤波只能输出低可信角度趋势。
- 目标在画面边缘时，畸变和 bbox 裁切会降低 LOS 可信度。
- 高机动目标会让 `lambda_dot` 快速变化，滤波器需要提高过程噪声，但噪声过大又会放大检测抖动。
- 高速迎近目标虽然角度可能稳定，但风险进程很快；6D LOS 本身无法给出接近时间，必须依赖 TTC、雷达或其他距离/速度来源。
- 纯横穿目标的角速度明显，但闭合趋势可能弱；6D LOS 不能单独判断是否正在接近。
- 飞机自身大角速度运动时，必须依赖 Pixhawk 历史姿态去旋转；姿态缓存缺失时该帧不能用于更新。

工程结论：

- 必须保留。
- 作为纯视觉方案的角度主通道。
- 不单独承担完整接近进程估计。
- 必须和 TTC 质量通道、安全状态机、track_id 管理、时间戳健康检查组合使用。

#### 13.1.3 纯角度制导与尺度膨胀：Scale Expansion / TTC

原理：

- 不估计绝对距离，只估计碰撞时间趋势 `TTC`。
- 目标接近时，图像面积 `A` 膨胀。
- 透视近似下，若 `A_dot > 0`，可用：

```text
TTC ~= 2 * A_filt / max(A_dot_filt, epsilon)
```

优势：

- 不依赖目标真实尺寸。
- 使用面积相对变化，而不是绝对距离。
- 可作为接近进程的无量纲指标。
- 与 6D LOS 角度通道天然解耦。

工程难点：

- `A_dot` 对检测抖动非常敏感。
- 必须对面积做低通滤波、异常剔除和裁切检查。
- bbox 被画面边缘裁切时，TTC 必须判为无效。

工程结论：

- 这是单目视觉估计接近进程的优先方法。
- 推荐与 6D LOS 滤波组合，而不是与 8D 距离滤波耦合。

#### 13.1.4 纯视觉推荐融合架构

纯视觉融合架构的核心不是“用 bbox 测距”，而是把相机最擅长的测角能力和 bbox 面积的相对膨胀趋势分开使用。角度通道只回答“目标方向如何变化”，尺度通道只回答“目标是否在快速接近”，二者通过质量门控组合，不互相污染状态。

```text
输入层：
  frame_id + exposure_timestamp
  bbox center (u, v)
  bbox area A
  track_id + track_quality
  Pixhawk historical attitude R_IB(exposure_timestamp)

角度通道：
  输入：bbox 中心点 (u, v)
  处理：曝光时间戳对齐 + R_IB 历史姿态去旋转 + 6D LOS 滤波
  输出：lambda_I, lambda_dot_I, innovation

接近进程通道：
  输入：bbox 面积 A
  处理：面积低通滤波 + 裁切/突变剔除 + 面积导数 + TTC 估计
  输出：TTC, TTC_quality, scale_continuity

融合与门控：
  仅当 track_id 连续、姿态查表成功、LOS innovation 正常、TTC_quality 有效时输出评估量
  任一质量门失败时，只记录日志，不输出受监督候选量
```

受监督评估中可记录如下候选量：

```text
g_eval = K(TTC) * lambda_dot_I
```

其中 `K(TTC)` 只作为仿真和日志分析中的动态增益因子。禁止将该候选量绕过安全状态机直接连接到高风险实飞控制输出。

主要特点：

- 角度与尺度完全解耦：bbox 面积抖动不会通过距离状态污染 `lambda_dot_I`。
- 不依赖目标真实尺寸：TTC 使用面积相对变化，而不是绝对尺寸反推距离。
- 强依赖时间同步：曝光时间戳和 Pixhawk 历史姿态是 LOS 稳定性的前提。
- 强依赖 track 连续性：target_id 变化、短时遮挡或 bbox 裁切都会使 TTC 失效。
- 输出是“质量受控的评估量”：适合日志、仿真和受监督实验，不是无条件控制指令。

适用边界：

- 目标必须在相机视场内保持足够帧数，且 bbox 中心可稳定测量。
- 目标图像尺度需要有可观测变化；远距离小目标只有几像素时，TTC 不可信。
- 目标速度过低或横向通过但几乎不接近时，面积膨胀弱，TTC 会退化。
- 目标速度过高或交会时间过短时，可用帧数不足，面积导数容易被延迟和抖动主导。
- 目标大幅机动、强滚转、姿态剧变或被遮挡时，bbox 面积不再只反映距离变化，TTC 必须降权。
- 背景杂波、曝光变化、运动模糊、压缩拖影和检测框裁切会直接降低尺度通道可信度。

敌方无人机速度与运动限制：

- 匀速或低机动目标：最适合该架构，6D LOS 能稳定给出方向变化，TTC 能提供接近趋势。
- 中等机动目标：角度通道仍可工作，但 TTC 需要更强质量门控，因为目标姿态变化会改变 bbox 面积。
- 高机动目标：纯视觉仍能做短时 LOS 评估，但不能可靠估计接近进程；需要雷达、双目、外部真值或其他距离/速度来源补强。
- 纯横穿目标：初期面积变化弱，TTC 可能显示“接近不明显”；此时只能依赖 LOS 角速度和目标保持在视场内的连续观测。
- 快速迎近目标：面积膨胀明显但时间窗口短，要求高帧率、低延迟和稳定曝光时间戳，否则 TTC 会滞后。

工程边界：

- 纯视觉方案不能保证所有速度和机动包线下都有可靠闭合率估计。
- 不能把 `TTC` 当作真实距离，也不能把 `K(TTC)` 当作实飞控制增益直接使用。
- 当 `TTC_quality` 低、LOS innovation 高、track_id 不连续或姿态缓存失败时，系统必须进入记录/安全状态。

### 13.2 方案二：机载毫米波雷达无人机探测方案

本项目讨论的毫米波雷达不是人体存在检测、生命体征检测或高度计，而是安装在穿越机/无人机机体上的前向空中目标探测雷达。它的任务是发现并跟踪空中小型无人机，输出距离、径向速度、多普勒、方位/俯仰和轨迹质量，用于非武器化日志、融合评估和安全状态判断。

由于雷达必须挂载在穿越机上，选型优先级应改为：

```text
无人机探测能力 > 机载 SWaP > 视场/刷新率/接口/时间戳 > 价格
```

#### 13.2.1 雷达方案收益

- 可直接观测空中目标距离和径向速度。
- 弱光、逆光和部分烟尘条件下比可见光稳定。
- 与视觉角度测量互补。
- 可用于验证 TTC 与真实距离/速度趋势是否一致。
- 对 RF 静默目标仍有探测价值。

#### 13.2.2 雷达方案限制

- 小型空中目标 RCS 小，实际探测距离强依赖目标材料、姿态、雷达截面积和背景杂波。
- 典型 5-7 寸穿越机的有效载荷和供电余量很小，成熟 DAA 雷达通常过重、过耗电。
- 低成本 24GHz/60GHz 人体存在模块、生命体征模块和高度计不适合空中无人机探测。
- TI AWR/IWR 等开发板不是开箱即用无人机探测雷达，需要自研点云处理、聚类、跟踪、虚警抑制和时间同步。
- 真正成熟的机载 DAA / C-UAS 雷达价格高、采购多为 RFQ，且可能有合规、出口管制或频谱许可要求。

#### 13.2.3 当前项目机载约束

| 约束 | 推荐目标 | 说明 |
| --- | --- | --- |
| 重量 | 小穿越机 <150-200 g；大载重平台可到 700-900 g | EchoFlight / Fortem R20 级别更接近中大型无人机载荷，不适合普通 5-7 寸穿越机 |
| 功耗 | 小穿越机 <10-15 W；大载重平台可到 40-50 W | 40 W 级雷达会明显影响续航、电源和散热 |
| 视场 | 前向至少覆盖相机主视场，优先 100° 级方位视场 | 窄视场雷达需要严格安装对准，容易丢失侧向目标 |
| 输出 | range、range_rate、azimuth、elevation、SNR/quality、timestamp | 只输出 presence/occupied 的模块不合格 |
| 接口 | Ethernet / UART / CAN 均可，必须可时间戳同步 | 高带宽点云优先 Ethernet；简单串口仅适合低速结果流 |
| 安装 | 前向无遮挡，远离碳纤维遮挡和强电噪声 | 碳纤维、金属件、电池和桨叶会影响波束和虚警 |

#### 13.2.4 适用于本项目的硬件选型

公开价格和参数会变化，以下仅作为 2026-06-12 的选型参考，采购前必须重新询价和复核 datasheet。

| 推荐等级 | 代表硬件 | 公开价格参考 | 机载 SWaP | 无人机探测能力 | 技术成熟度 | 本项目判断 |
| --- | --- | --- | --- | --- | --- | --- |
| A：技术最匹配，但平台要求高 | Echodyne EchoFlight | RFQ，公开通常不标价 | 18.7 x 12 x 4 cm；817 g；12-28 V；45 W 工作，<10 W 热待机 | Phantom 4 级目标 >750 m，Matrice 600 级目标 >1 km；方位精度 <1°，俯仰 <1.5°，距离精度 <3.25 m | 专用机载 DAA 雷达，成熟度最高 | 适合大载重穿越机或中大型无人机；普通 5-7 寸穿越机基本不适合 |
| A-：无人机探测明确，重量/功耗仍偏高 | Fortem TrueView R20 / R20i | RFQ | 200 x 75 x 38.3 mm；748 g；18-36 V 供电，功耗需询厂商 | 0.1 m² RCS 目标约 800 m；120° 方位视场，40° 俯仰视场；输出距离、速度、RCS 等 | 专用 C-UAS / DAA 雷达，工程成熟 | 若能采购 R20i 空中版本，是机载无人机探测候选；对小穿越机仍偏重 |
| B：研发验证首选 | TI AWR1843BOOST / AWR1843AOP 定制板 | TI 标价约 USD 313.95；DigiKey/Mouser 当前约 USD 469-483 | 开发板形态，需自制外壳、减振、供电和散热 | 车规中距雷达芯片，TI 资料称中距目标可到约 150 m，距离分辨率 <4 cm；空中无人机检测需自研算法 | 芯片成熟，空中小目标方案不成熟 | 适合先做机载数据采集和算法研究；不能视为成品无人机探测雷达 |
| B-：近距点云验证 | TI IWR6843ISK / IWR6843ISK-ODS | IWR6843ISK 约 USD 282.98；常需 MMWAVEICBOOST 约 USD 402.63 | 开发板形态，需外壳、减振、供电和数据链 | ISK 人员级目标约 75 m；ODS 广角约 12 m；对空中小无人机需重写检测跟踪 | 工业开发生态成熟 | 更适合台架和近距飞行数据验证，不适合当前项目作为远距无人机探测主传感器 |
| C：明确排除 | DFRobot SEN0395、Seeed MR60BHA1、Ainstein US-D1 / LR-D1 等 | USD 十几到数百不等 | 轻、小、低功耗 | 存在检测、生命体征或高度计；不是空中无人机探测雷达 | 各自领域成熟 | 不作为目标探测雷达，只能用于入门、测高或避障实验 |
| D：地面/车载外部真值 | EchoShield、EchoGuard、Robin IRIS 等 | RFQ，通常高价 | 多为地面/车载，不适合装在穿越机上 | 公里级 C-UAS 探测和分类 | 成熟 | 可作为外部真值/测试场传感器，不作为机载载荷 |

#### 13.2.5 推荐结论

1. 当前 MVP 不挂成熟 DAA 雷达，继续以纯视觉 6D + TTC 为主，预留 `RadarMeasurement` 接口。
2. 若必须做机载雷达数据采集，优先使用 AWR1843BOOST/AOP 或 IWR6843ISK 做研究载荷，只验证时间同步、点云、距离/径向速度质量，不承诺稳定探测无人机。
3. 若平台升级到可承载 800 g、45 W 级载荷，并且预算/合规允许，优先评估 EchoFlight；备选 Fortem TrueView R20/R20i。
4. 低成本存在检测、生命体征和高度计模块从无人机探测候选中删除。

#### 13.2.6 雷达融合建议

非武器化评估链路中，雷达只进入独立测距/测速通道：

```text
视觉：
  bbox center -> 6D LOS -> lambda_I, lambda_dot_I

视觉尺度：
  bbox area -> TTC -> 接近趋势

雷达：
  range, range_rate, doppler, point cloud -> 距离/径向速度质量评估

融合评估：
  对比 TTC 与 radar range_rate
  对比视觉 LOS 与 radar angle/point cloud
  输出日志、置信度和安全状态
```

工程推荐：

- MVP 仍以纯视觉 6D + TTC 为主，并预留雷达接口。
- 研发阶段优先用 TI AWR/IWR 做机载记录和台架/HIL。
- 工程化无人机探测优先看 EchoFlight / Fortem R20 级别，但必须先确认穿越机载荷、供电和散热余量。
- 地面 C-UAS 雷达只用于外部真值，不作为机载选型。

### 13.3 公开资料来源

- TI IWR6843ISK 官方页面：<https://www.ti.com/tool/IWR6843ISK>
- TI IWR6843ISK-ODS 官方页面：<https://www.ti.com/tool/IWR6843ISK-ODS>
- DigiKey IWR6843ISK 价格页：<https://www.digikey.com/en/products/detail/texas-instruments/IWR6843ISK/10434492>
- TI AWR1843BOOST 官方页面：<https://www.ti.com/tool/AWR1843BOOST>
- DigiKey AWR1843BOOST 价格页：<https://www.digikey.com/en/products/detail/texas-instruments/AWR1843BOOST/10445300>
- TI AWR1843 中距雷达说明：<https://www.ti.com/video/5990261460001>
- Echodyne Airborne Radar 页面：<https://www.echodyne.com/uas-aam/airborne-radar>
- EchoFlight 第三方规格页：<https://www.defenseadvancement.com/company/echodyne/echoflight-radar/>
- Fortem TrueView R20 页面：<https://www.fortemtech.com/products/trueview-r20/>
- Fortem R20 AERPAW 规格说明：<https://sites.google.com/ncsu.edu/aerpaw-wiki/aerpaw-user-manual/5-future-platform-features/5-6-fortem-radar>
- DFRobot SEN0395 页面：<https://www.dfrobot.com/product-2282.html>
- Seeed MR60BHA1 文档：<https://wiki.seeedstudio.com/Radar_MR60BHA1/>
- Newark MR60BHA1 价格页：<https://www.newark.com/seeed-studio/101990886/respiratory-heartbeat-detection/dp/74AK7835>
- Ainstein US-D1 页面：<https://ainstein.ai/us-d1-all-weather-radar-altimeter/>
- Echodyne EchoFlight 页面：<https://www.echodyne.com/radar-systems/echoflight>
- Echodyne EchoShield 页面：<https://www.echodyne.com/radar-systems/echoshield>

---

## 14. 风险与对策

| 风险 | 影响 | 对策 |
| --- | --- | --- |
| 相机无真实曝光时间戳 | 时间对齐误差 | 标定固定延迟并降低可信度；优先硬件时间戳 |
| MAVLink 频率不足 | 姿态缓存精度下降 | 提高链路速率，减少无关消息，记录丢包 |
| EKF 状态不可读 | 无法判断导航健康 | 使用 estimator status 或等效 health flags |
| bbox 抖动严重 | 视觉质量下降 | bbox 仅作为质量信号，不进主 LOS 状态 |
| 目标误检 | 目标跳变 | target_id 锁定、连续确认、丢失重捕获 |
| 时间戳跳变 | 伪 LOS 角速度 | 使用单调时钟，异常帧丢弃 |
| 状态机遗漏异常 | 错误状态持续运行 | 所有拒绝条件日志化并进入 SAFE_HOLD / ABORT |
| 外部输出越权 | 高风险行为 | Pixhawk 保持最终控制权，人工接管最高优先级 |

---

## 15. MVP 范围

MVP 包含：

- MAVLink 状态接入。
- Pixhawk EKF / GPS 健康判断。
- 姿态历史缓存。
- 相机帧时间戳记录。
- YOLO / ByteTrack 结果适配。
- target_id 锁定。
- 时间对齐 LOS 计算。
- 6 维 LOS 滤波。
- 安全状态机。
- 结构化日志。
- 回放和评估工具。

MVP 不包含：

- 自主末端追撞。
- 动能拦截部署。
- 未经监督的视觉闭环控制。
- 绕过 Pixhawk 的执行机构控制。

---

## 16. 排期建议

| 时间 | 内容 | 交付物 |
| --- | --- | --- |
| 第 1-2 天 | 阶段 0-1 | 工程骨架、配置、mock、日志 |
| 第 3-5 天 | 阶段 2 | MAVLink 接入、姿态缓存、时间戳统计 |
| 第 6-8 天 | 阶段 3 | 相机/检测适配、TargetManager |
| 第 9-11 天 | 阶段 4 | LOS 转换、6 维滤波、离线评估 |
| 第 12-15 天 | 阶段 5 | 台架、HIL、非武器化数据采集准备 |

---

## 17. 后续扩展

在 MVP 通过后，可逐步增加：

- 可见光 / 红外融合质量评估。
- 硬件触发或 PTP 时间同步。
- 相机外参在线一致性检查。
- 更完整的标注数据集。
- 仿真环境中的闭环研究。
- 受监督 setpoint 输出的安全门控实验。

所有扩展遵循同一流程：

```text
先记录
  -> 再回放
  -> 再仿真
  -> 再 HIL
  -> 最后才进行受监督、低风险、非武器化飞行验证
```

---

## 18. 与原分片的合并结果

| 原分片内容 | 合并后的处理 |
| --- | --- |
| 系统总体架构 | 保留，改为视觉导航评估系统 |
| 粗对准 / 精对准代码 | 删除，改为等待 Pixhawk EKF healthy |
| 8 维 bbox 测距 EKF | 改为 6 维 LOS 滤波，bbox 解耦 |
| 当前姿态直接转换图像 | 改为曝光时间戳查姿态历史 |
| PNG 闭环指令代码 | 不作为部署实现，仅保留安全边界说明 |
| 速度积分输出 | 删除，标记为 wind-up 风险 |
| 最高分目标选择 | 改为 target_id 锁定 |
| 末端追撞逻辑 | 删除，改为视觉失效安全退出 |
| 配置和测试章节 | 保留安全相关部分并统一配置格式 |
| 部署指南 | 改为台架、HIL、非武器化数据采集流程 |

---

## 19. 结论

本统一方案将原有多个分片整合为一个安全、可验证、可审计的工程基线。

核心改动：

- Pixhawk EKF 作为唯一导航真值源。
- RK3588 专注视觉、时间戳对齐、LOS 估计和日志。
- 姿态 Ring Buffer 解决图像延迟与姿态错位问题。
- 6 维 LOS 滤波避免 bbox 测距污染主状态。
- target_id 锁定降低误检目标切换风险。
- 安全状态机保证异常时退出到 SAFE_HOLD 或 ABORT。
- 实施流程从 mock、回放、台架、HIL 到非武器化数据采集逐级推进。

该文档可作为后续工程拆分、代码实现、测试验证和安全审查的统一依据。
