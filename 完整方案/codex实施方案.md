# 实施方案（安全审查与非武器化验证版）

## 1. 实施目标

本文给出基于《算法方案.md》的工程实施路线。目标是搭建一个可验证、可回放、可审计的视觉导航评估系统，用于发现并修正时间戳、姿态对齐、LOS 估计、目标跟踪和飞控接口中的工程风险。

本实施方案限定在以下用途：

- 仿真验证。
- 数据回放。
- 台架测试。
- HIL 测试。
- 非武器化、受监督飞行数据采集。

本方案不提供可直接用于自主攻击、末端追撞或动能拦截的部署流程。

---

## 2. 总体实施路线

实施分为 6 个阶段：

```text
阶段 0：需求冻结与安全边界确认
阶段 1：工程骨架与日志系统
阶段 2：MAVLink 状态接入与姿态历史缓存
阶段 3：相机时间戳、检测跟踪与目标管理
阶段 4：LOS 估计与离线评估
阶段 5：HIL / 台架 / 非武器化飞行数据验证
```

每一阶段必须有明确退出条件。未通过上一阶段验收，不进入下一阶段。

---

## 3. 阶段 0：需求冻结与安全边界确认

### 3.1 工作内容

- 明确系统仅用于安全审查、算法验证和非武器化测试。
- 明确 RK3588 不绕过 Pixhawk 直接控制执行机构。
- 明确人工接管、地理围栏、飞控 failsafe 和日志记录为强制要求。
- 确认相机、Pixhawk、RK3588、MAVLink 链路和供电方案。
- 确认时间戳来源：相机曝光时间、RK3588 单调时钟、Pixhawk 时间戳。

### 3.2 交付物

- 测试边界说明。
- 硬件连接图。
- 消息与日志字段清单。
- 安全检查清单。

### 3.3 退出条件

- 所有参与模块的责任边界明确。
- 禁止项明确：不做自主末端追撞、不绕开飞控安全机制、不用未经验证的视觉闭环做实飞控制。
- 形成可审计的配置与日志目录规范。

---

## 4. 阶段 1：工程骨架与日志系统

### 4.1 推荐目录结构

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
├── tests/
│   ├── test_attitude_buffer.py
│   ├── test_timebase.py
│   ├── test_target_manager.py
│   └── test_los_estimator.py
└── logs/
```

### 4.2 日志字段

必须记录：

- 系统单调时间。
- 相机 `frame_id`。
- 图像曝光时间戳。
- 检测输出时间戳。
- bbox、score、class_id。
- ByteTrack `track_id` 和 track 状态。
- Pixhawk 姿态、速度、位置、EKF 健康状态。
- 姿态缓存查找结果。
- LOS 测量、LOS 滤波状态、创新量。
- 状态机状态和状态切换原因。
- 所有安全拒绝原因。

日志格式建议：

- 高频结构化数据使用 Parquet、MCAP、ROS bag 或二进制格式。
- 低频事件使用 JSONL。
- 配置文件随每次日志拷贝保存，保证可复现。

### 4.3 退出条件

- 程序可在无相机、无 Pixhawk 的 mock 模式启动。
- mock 数据能写入完整日志。
- 日志可被 `replay_log.py` 回放。
- 每条记录都能追溯到配置版本。

---

## 5. 阶段 2：MAVLink 状态接入与姿态历史缓存

### 5.1 工作内容

- 建立 MAVLink 连接。
- 订阅 Pixhawk 姿态、位置、速度、GPS 和 EKF 健康状态。
- 统一转换为内部 `VehicleState`。
- 建立 `AttitudeHistoryBuffer`。
- 处理飞控时间戳和 RK3588 单调时钟之间的偏移。

### 5.2 关键接口

```text
VehicleState:
  timestamp_mono
  timestamp_autopilot
  quaternion_IB
  R_IB
  velocity_ned
  position_ned
  ekf_healthy
  gps_fix_ok
  flight_mode
  armed
```

```text
AttitudeHistoryBuffer:
  push(VehicleState)
  lookup(timestamp_mono) -> InterpolatedVehicleState
  stats() -> buffer_age, sample_rate, miss_count
```

### 5.3 实现要点

- 姿态插值使用四元数插值。
- 缓冲区保留最近至少 1 秒。
- lookup 不允许使用未来数据修正过去图像。
- 若查询时间戳超出缓存范围，必须返回失败。
- 记录姿态消息频率、抖动、丢包和缓存命中率。

### 5.4 验收测试

- 输入固定姿态序列，lookup 结果不漂移。
- 输入匀速旋转姿态，插值结果连续。
- 人为制造时间戳跳变，系统进入拒绝更新状态。
- 缓冲区不足时，视觉测量被丢弃而不是使用当前姿态替代。

---

## 6. 阶段 3：相机时间戳、检测跟踪与目标管理

### 6.1 工作内容

- 接入相机帧。
- 获取或估算曝光时间戳。
- 接入 YOLO / RKNN 推理输出。
- 接入 ByteTrack。
- 实现 `TargetManager`，用 `track_id` 锁定目标。

### 6.2 数据接口

```text
FramePacket:
  frame_id
  exposure_timestamp_mono
  receive_timestamp_mono
  width
  height
  camera_id
```

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

```text
TargetState:
  mode
  target_id
  age_frames
  lost_frames
  quality
  last_measurement
```

### 6.3 目标管理逻辑

状态：

```text
SEARCH
  未锁定目标，只评估候选 track。

CONFIRM
  候选目标连续满足质量门限，准备锁定。

LOCKED
  只接受已锁定 target_id。

COAST
  目标短时丢失，只做预测和日志，不接管其他目标。

REACQUIRE
  丢失超时后解除 target_id，重新搜索。
```

质量检查：

- score 超过门限。
- bbox 未严重裁切。
- bbox 尺度变化连续。
- 像素中心变化连续。
- track age 足够。
- class_id 一致。

### 6.4 验收测试

- 单目标稳定跟踪时，target_id 不变化。
- 插入高分误检框时，不切换目标。
- 目标短时丢失时进入 COAST。
- 丢失超过阈值后解除锁定。
- target_id 变化时，LOS 滤波器被重置或降权。

---

## 7. 阶段 4：LOS 估计与离线评估

### 7.1 工作内容

- 根据相机内参将 bbox 中心转为相机系单位视线。
- 使用曝光时间戳查询历史姿态。
- 将相机系 LOS 转换到惯性系。
- 实现 6 维 LOS 滤波器。
- 输出 LOS 质量指标和创新量。

### 7.2 坐标转换

```text
pixel center -> normalized image ray -> los_C
los_B = R_BC * los_C
los_I = R_IB(exposure_time) * los_B
```

要求：

- 相机内参必须来自标定。
- `R_BC` 必须来自安装标定或可靠测量。
- 坐标系定义写入配置，不在代码中硬编码。

### 7.3 LOS 滤波器

状态：

```text
X = [lambda_x, lambda_y, lambda_z,
     lambda_dot_x, lambda_dot_y, lambda_dot_z]
```

输入：

- `lambda_I_measured`
- `measurement_timestamp`
- `track_quality`

输出：

- `lambda_I`
- `lambda_dot_I`
- innovation norm
- quality
- valid_for_supervised_eval

约束：

- 每次更新后归一化 `lambda`。
- `lambda_dot` 投影到 `lambda` 垂直平面。
- bbox 不进入主滤波状态。
- 目标 ID 改变时，滤波器重置或降权。

### 7.4 离线评估指标

- 视觉端到端延迟。
- 姿态缓存命中率。
- LOS 创新量分布。
- LOS 角速度尖峰数量。
- bbox jitter 与 LOS 状态相关性。
- track_id 稳定性。
- 状态机安全退出次数和原因。

### 7.5 验收测试

- 注入 30-50ms 视觉延迟，对比时间对齐开启/关闭的 LOS 角速度差异。
- 注入 bbox jitter，确认 LOS 主状态不发散。
- 注入目标切换，确认滤波状态不会无缝拼接错误目标。
- 姿态缓存查找失败时，该帧被拒绝。

---

## 8. 阶段 5：HIL、台架与非武器化飞行数据验证

### 8.1 台架测试

验证内容：

- Pixhawk MAVLink 数据频率。
- EKF healthy 判断。
- 姿态缓存稳定性。
- 相机帧时间戳稳定性。
- YOLO / ByteTrack 延迟统计。
- 日志完整性。

不做内容：

- 不接入危险执行机构。
- 不运行未经验证的自主追踪控制。
- 不执行末端追撞测试。

### 8.2 HIL 测试

验证内容：

- 飞控状态变化是否能正确驱动状态机。
- MAVLink 丢包、延迟、重连时是否安全退出。
- 时间戳异常是否被识别。
- 视觉断流和目标丢失是否被正确处理。

### 8.3 非武器化飞行数据采集

要求：

- 使用非危险目标或标靶。
- 只采集视觉、飞控和 LOS 评估日志。
- 人工遥控或飞控安全模式保持最终控制权。
- 测试区域封闭，启用地理围栏和 failsafe。
- 飞行后只做离线评估，不现场开放高风险控制闭环。

### 8.4 退出条件

- 至少完成三组稳定日志。
- 日志中无未解释时间戳跳变。
- 姿态缓存命中率满足设计要求。
- 视觉丢失、EKF unhealthy 和 MAVLink 异常均触发安全状态。
- 回放结果可复现。

---

## 9. 安全状态机实施

### 9.1 状态定义

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

### 9.2 强制退出条件

任一条件成立时，停止视觉闭环输出并进入安全状态：

- Pixhawk EKF unhealthy。
- GPS fix 不满足测试要求。
- MAVLink 超时。
- 姿态缓存 lookup 连续失败。
- 图像时间戳异常。
- 目标 ID 丢失超时。
- bbox 严重裁切或视觉质量过低。
- LOS 创新量持续超限。
- 人工接管。

### 9.3 日志要求

每次状态切换必须记录：

- 旧状态。
- 新状态。
- 触发原因。
- 当前时间戳。
- 关键传感器质量。
- 最近一次有效目标状态。

---

## 10. 开发排期建议

### 第 1-2 天：阶段 0-1

- 冻结安全边界和接口。
- 搭建工程目录。
- 实现配置加载和日志框架。
- 完成 mock 模式。

### 第 3-5 天：阶段 2

- 接入 MAVLink。
- 实现 `VehicleState`。
- 实现姿态历史缓存。
- 完成时间戳统计工具。

### 第 6-8 天：阶段 3

- 接入相机和检测结果。
- 实现 ByteTrack 输出适配。
- 实现 `TargetManager`。
- 完成目标锁定和丢失测试。

### 第 9-11 天：阶段 4

- 实现相机 LOS 转换。
- 实现 6 维 LOS 滤波。
- 实现离线评估脚本。
- 完成延迟、bbox jitter、目标切换测试。

### 第 12-15 天：阶段 5

- 台架测试。
- HIL 测试。
- 非武器化飞行数据采集准备。
- 汇总日志和问题清单。

---

## 11. 拦截场景双方案实施评估

本节中的“拦截场景”仅指仿真、回放、台架、HIL 和非武器化受监督飞行数据评估，不表示允许将视觉或雷达输出直接接入自主末端追撞控制。

### 11.1 方案一：纯视觉方案

纯视觉实施不应在 8D、6D、TTC 中三选一，而应采用“6D 角度通道 + Scale Expansion/TTC 接近进程通道”的解耦组合。

#### 8D 卡尔曼滤波对照组

实施方式：

- 输入 bbox 中心点和 bbox 尺度。
- 状态包含 `lambda`、`lambda_dot`、`r`、`r_dot`。
- 只用于离线 replay 对照。

验收目的：

- 证明 bbox 尺度 jitter 会导致 `r/r_dot` 剧烈波动。
- 证明距离噪声会污染 `lambda_dot`。
- 输出结论应为“不进入主链路”。

#### 6D 卡尔曼滤波主通道

实施方式：

- 输入 bbox 中心点 `(u, v)`，不输入 bbox 尺度、面积或目标真实尺寸。
- 使用曝光时间戳查询 Pixhawk 姿态历史，得到 `R_IB(t_exposure)`。
- bbox 中心经相机内参得到 `los_C`，经安装外参 `R_BC` 转到机体系，再经历史姿态去旋转得到 `lambda_I_measured`。
- 状态只估计 `lambda` 和 `lambda_dot`：

```text
X = [lambda_x, lambda_y, lambda_z,
     lambda_dot_x, lambda_dot_y, lambda_dot_z]
```

- 更新后归一化 `lambda`，并将 `lambda_dot` 投影到 `lambda` 的垂直平面。

验收目的：

- 输出稳定 LOS 角速度。
- bbox 尺度抖动不进入主状态。
- 姿态缓存失败时拒绝该帧。
- 目标 ID 切换时重置或降权滤波器。
- 输出 `innovation`、协方差和 `quality`，供融合层门控。

主要劣势：

- 不输出绝对距离，也不输出闭合速度。
- 无法区分远处高速目标和近处低速目标造成的相似角速度。
- 纯横穿、面积变化弱或目标远距离像素过小时，无法可靠判断接近进程。
- 高机动目标会要求更大的过程噪声，导致对检测抖动更敏感。
- 强依赖曝光时间戳、姿态历史、相机内参、安装外参和机体安装刚性。

实施边界：

- 姿态缓存 miss、时间戳跳变、bbox 严重裁切、track_id 不连续、LOS innovation 超限时，不更新 6D 滤波器。
- 高速迎近目标必须结合 TTC 或雷达距离/径向速度通道评估接近进程。
- 6D LOS 输出只能作为角度主通道，不单独作为完整接近进程估计。

#### Scale Expansion / TTC 通道

实施方式：

- 输入 bbox 面积 `A`。
- 对 `A` 做低通滤波。
- 差分得到 `A_dot_filt`。
- 若 `A_dot_filt > 0`，计算：

```text
TTC ~= 2 * A_filt / max(A_dot_filt, epsilon)
```

验收目的：

- 作为接近进程指标。
- 不依赖目标真实尺寸。
- bbox 裁切、面积突变、track_id 切换时 TTC 标记为无效。

#### 纯视觉融合输出

纯视觉融合架构应将 6D LOS 和 TTC 作为两条独立通道实施。6D 通道负责稳定测角，TTC 通道负责估计接近趋势；融合层只做质量门控和日志/仿真评估，不把尺度噪声反馈进 LOS 主状态。

```text
输入层：
  frame_id + exposure_timestamp
  bbox center (u, v)
  bbox area A
  track_id + track_quality
  Pixhawk historical attitude R_IB(exposure_timestamp)

角度通道输出：
  lambda_I, lambda_dot_I, innovation

尺度通道输出：
  TTC, TTC_quality, scale_continuity

融合输出：
  质量门控通过后记录 g_eval = K(TTC) * lambda_dot_I
```

`g_eval` 只进入日志、仿真和受监督评估，不得绕过安全状态机直接连接到高风险实飞控制输出。

实施特点：

- bbox 面积不进入 6D LOS 主滤波器。
- target_id 改变、bbox 裁切、面积突变、姿态查表失败时，TTC 标记无效。
- LOS innovation 超限时，融合输出无效，只保留日志。
- `g_eval` 只用于 replay、plot、仿真和受监督评估。

速度与运动边界：

- 匀速或低机动目标最适合验证，LOS 与 TTC 都容易稳定。
- 中等机动目标需要更严格的 bbox 连续性、track 连续性和面积突变门控。
- 高机动目标只能依赖短时 LOS 评估，TTC 容易被目标姿态变化污染。
- 纯横穿目标的面积膨胀弱，TTC 可能无效。
- 快速迎近目标的面积膨胀明显，但可用帧数少，对低延迟和曝光时间戳要求最高。

验收边界：

- 纯视觉链路不得承诺所有速度/机动场景下的可靠闭合率估计。
- `TTC` 不等价于真实距离，`K(TTC)` 不等价于可直接执行的控制增益。
- 当 `TTC_quality` 低、LOS innovation 高、track_id 不连续或姿态缓存失败时，系统必须进入记录/安全状态。

### 11.2 方案二：机载毫米波雷达无人机探测方案

毫米波雷达实施目标是机载探测空中无人机，提供距离、径向速度、多普勒、方位/俯仰和质量评估，用于校验纯视觉 TTC 和 LOS 结果。人体存在检测、生命体征检测和高度计不作为本项目目标探测雷达。

机载约束：

| 约束 | 推荐目标 | 说明 |
| --- | --- | --- |
| 重量 | 小穿越机 <150-200 g；大载重平台可到 700-900 g | EchoFlight / Fortem R20 更接近中大型无人机载荷 |
| 功耗 | 小穿越机 <10-15 W；大载重平台可到 40-50 W | 40 W 级雷达会显著影响续航、电源和散热 |
| 视场 | 前向至少覆盖相机主视场，优先 100° 级方位视场 | 窄视场需要严格安装对准 |
| 输出 | range、range_rate、azimuth、elevation、SNR/quality、timestamp | 只输出 presence 的模块不合格 |
| 安装 | 前向无遮挡，远离碳纤维遮挡和强电噪声 | 碳纤维、金属件、电池和桨叶会影响波束 |

适用于本项目的硬件选型：

| 推荐等级 | 代表硬件 | 公开价格参考 | 机载 SWaP | 无人机探测能力 | 技术成熟度 | 实施建议 |
| --- | --- | --- | --- | --- | --- | --- |
| A：技术最匹配，但平台要求高 | Echodyne EchoFlight | RFQ，公开通常不标价 | 18.7 x 12 x 4 cm；817 g；12-28 V；45 W 工作，<10 W 热待机 | Phantom 4 级目标 >750 m，Matrice 600 级目标 >1 km；方位精度 <1°，俯仰 <1.5°，距离精度 <3.25 m | 专用机载 DAA 雷达 | 仅在大载重平台上评估；普通 5-7 寸穿越机不建议 |
| A-：无人机探测明确，重量/功耗仍偏高 | Fortem TrueView R20 / R20i | RFQ | 200 x 75 x 38.3 mm；748 g；18-36 V 供电，功耗需询厂商 | 0.1 m² RCS 目标约 800 m；120° 方位视场，40° 俯仰视场；输出距离、速度、RCS 等 | 专用 C-UAS / DAA 雷达 | 若能采购 R20i 空中版本，可作为机载无人机探测候选；对小穿越机仍偏重 |
| B：研发验证首选 | TI AWR1843BOOST / AWR1843AOP 定制板 | TI 标价约 USD 313.95；DigiKey/Mouser 当前约 USD 469-483 | 开发板形态，需自制外壳、减振、供电和散热 | 车规中距雷达芯片，TI 资料称中距目标可到约 150 m，距离分辨率 <4 cm；空中无人机检测需自研算法 | 芯片成熟，空中小目标方案不成熟 | 推荐作为机载数据采集和算法研究载荷 |
| B-：近距点云验证 | TI IWR6843ISK / IWR6843ISK-ODS | IWR6843ISK 约 USD 282.98；常需 MMWAVEICBOOST 约 USD 402.63 | 开发板形态，需外壳、减振、供电和数据链 | ISK 人员级目标约 75 m；ODS 广角约 12 m；对空中小无人机需重写检测跟踪 | 工业开发生态成熟 | 推荐台架和近距飞行数据验证，不建议作为远距主传感器 |
| C：明确排除 | DFRobot SEN0395、Seeed MR60BHA1、Ainstein US-D1 / LR-D1 等 | USD 十几到数百不等 | 轻、小、低功耗 | 存在检测、生命体征或高度计；不是空中无人机探测雷达 | 各自领域成熟 | 不进入目标探测链路 |
| D：外部真值 | EchoShield、EchoGuard、Robin IRIS 等 | RFQ，通常高价 | 多为地面/车载，不适合装在穿越机上 | 公里级 C-UAS 探测和分类 | 成熟 | 用作测试场外部真值，不作为机载载荷 |

实施结论：

- 当前 MVP 不挂成熟 DAA 雷达，继续以纯视觉 6D + TTC 为主，预留 `RadarMeasurement` 接口。
- 机载雷达研发验证优先 AWR1843BOOST/AOP 或 IWR6843ISK，只验证时间同步、点云和距离/径向速度质量。
- 若平台升级到可承载 800 g、45 W 级载荷，并且预算/合规允许，优先评估 EchoFlight；备选 Fortem TrueView R20/R20i。
- 低成本存在检测、生命体征和高度计模块从无人机探测候选中删除。

雷达实施接口：

```text
RadarMeasurement:
  timestamp_mono
  range
  range_rate
  doppler
  azimuth
  elevation
  snr
  point_count
  track_id_optional
```

融合验证：

- 对比 `TTC` 与 `range / -range_rate` 的趋势一致性。
- 对比视觉 LOS 与雷达角度/点云聚类方向。
- 雷达和视觉时间戳必须进入统一单调时钟。
- 雷达异常只降低融合可信度，不触发危险动作。

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

## 12. 风险与对策

| 风险 | 影响 | 对策 |
| --- | --- | --- |
| 相机没有真实曝光时间戳 | 时间对齐误差 | 标定固定延迟，只用于评估；优先选择支持硬件时间戳的相机 |
| MAVLink 频率不足 | 姿态缓存精度下降 | 提高链路速率，降低不必要消息，记录丢包率 |
| Pixhawk EKF 状态不可用 | 无法判断导航健康 | 使用飞控 estimator status 或等效 health flags |
| bbox 抖动严重 | 视觉质量下降 | bbox 只做辅助质量信号，不进入主 LOS 滤波 |
| 目标误检 | 目标跳变 | track_id 锁定、连续确认、丢失重捕获 |
| 时间戳跳变 | LOS 伪机动 | 统一单调时钟，异常帧丢弃 |
| 状态机遗漏异常 | 系统继续错误运行 | 所有拒绝条件必须日志化并进入 SAFE_HOLD / ABORT |

---

## 13. 最小可交付版本

MVP 不包含实飞自主闭环控制，只包含：

- MAVLink 状态接入。
- 姿态历史缓存。
- 相机帧时间戳记录。
- YOLO / ByteTrack 结果适配。
- target_id 锁定。
- 时间对齐 LOS 计算。
- 6 维 LOS 滤波。
- 安全状态机。
- 日志回放和评估工具。

MVP 验收：

- 可用 mock 数据完整跑通。
- 可用真实 Pixhawk 数据记录并回放。
- 可用相机数据计算时间对齐 LOS。
- 在注入延迟、丢包、误检、目标丢失时进入预期状态。

---

## 14. 后续扩展

在 MVP 通过后，可逐步增加：

- 多相机或可见光/红外融合质量评估。
- 更严格的时间同步，例如硬件触发或 PTP。
- 更完善的相机外参在线检查。
- 更高质量的离线标注和评估集。
- 仿真环境中的闭环控制研究。
- 受监督 setpoint 输出的安全门控实验。

所有扩展都应保持同一原则：先记录、再回放、再 HIL，最后才考虑受监督、低风险、非武器化飞行验证。
