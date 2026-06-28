# BodyRate 三问题线测试方案

本文基于 `BodyRate_五组诊断实验报告` 的 A/B/C/D/E 结果，目标是把当前问题拆成三条可验证链路：

- A：真值位置 PNG 已经接近目标但没有 collision，确认是评价/碰撞路径问题还是控制问题。
- B：云台相机能看见目标但命中率差，确认是云台-机体解耦、末端 bbox 失真还是 TTC 门控问题。
- CDE：捷联 baseline 的失败来自识别、LOS 滤波、控制频率还是推力饱和，寻找可提升的稳定配置。

## 0. 统一实验约束

默认工况：

- 距离：`50 60 70 80 90 100 m`
- 高度差：`20 m`
- 横向偏置：`-20 m`
- 目标：`IntruderActor`
- 目标模型：`Quadrotor1`
- 目标速度：`5 m/s`
- 拦截速度系数：`speed_ratio=2`
- 控制链路：`guidance_output_mode=accel_body_rate`
- PX4 指令：`px4_command_mode=mavlink_body_rate`
- 基准频率：`rate_hz=8`
- 每个工况重启 PX4 SITL 和 Blocks。

核心记录指标：

- `hit`
- `min_range`
- `final_range`
- `geometric_hit(range < 1.0m / 1.5m / 2.0m)`
- `collision_object`
- `detected_rate`
- `valid_rate`
- `body_rate_control_active_rate`
- `thrust_saturation_rate`
- `los_innovation_reject_count`
- `terminal_state`
- `frame_centering_state`
- 最近点前后 `0.5s / 1s / 2s` 的距离变化
- 最近点前 `1s` 的 `target_body_bearing_deg`

## 1. A 线：真值位置 PNG 碰撞评价验证

### A-目标

确认 A 组 `min_range=0.07-0.33m` 但 `hit=0` 的原因。优先判断：

- AirSim collision 没有报 `IntruderActor/Quadrotor1`
- actor 碰撞体没有参与 collision
- actor teleport 路径导致 collision event 不可靠
- truth 脚本记录的目标实体和碰撞检查实体不一致

### A-前置修改

建议先补日志字段：

- `geometric_hit_1m`
- `geometric_hit_15m`
- `geometric_hit_2m`
- `collision_raw_hit`
- `collision_accepted`
- `collision_target_patterns`
- `collision_interceptor_object`
- `collision_intruder_object`
- `collision_time_delta_s`
- `target_entity`
- `target_position_source`

### A-实验矩阵

|编号|目标|检测/目标|成功判据|距离|导引|预期用途|
|---|---|---|---|---|---|---|
|A0|复现当前 A|truth + actor|AirSim collision|50-100|TTC/VM|确认原始现象|
|A1|几何命中评价|truth + actor|`range < 1.0m`|50-100|TTC/VM|判断导引是否几何命中|
|A2|放宽几何门限|truth + actor|`range < 1.5m/2.0m`|50-100|TTC/VM|和 B/C collision 距离对齐|
|A3|actor 碰撞路径|truth + actor|AirSim collision + object log|50/80/100|TTC/VM|确认碰撞对象是否可报目标|
|A4|车辆目标对照|truth + second vehicle|AirSim vehicle pair collision|50/80/100|TTC/VM|隔离 actor collision 问题|

### A-判读标准

- 如果 A1/A2 全部几何命中，而 A0/A3 仍不 collision，则 A 失败是评价/碰撞路径问题。
- 如果 A4 能 collision，A3 不能 collision，则 actor collision 机制或 actor 碰撞体是主因。
- 如果 A1 也不稳定，才回到 body-rate 控制链路排查。

## 2. B 线：云台相机控制解耦验证

### B-目标

解释为什么云台相机理论上视场更稳定，但命中率仍低于 C。重点验证：

- 云台只让相机看见目标，没有让机体航向/速度矢量对准目标。
- 末端 `bbox_top_clipped`、`bbox_area_large` 后导引失效。
- TTC 面积通道在云台近距大框下不可靠。
- LOS filter 是否增加额外门控。

### B-前置指标

每个工况补充或离线统计：

- 最近点前 `1s` 平均 `target_body_bearing_deg`
- 最近点前 `1s` 平均 `gimbal_yaw_deg`
- 最近点前检测率
- 最近点后 `0.5s/1s/2s` 检测率
- 最近点后 `0.5s/1s/2s` 距离
- `bbox_top_clipped` 首次出现时间
- `bbox_area_large` 首次出现时间
- `terminal_lost` 首次出现时间

### B-实验矩阵

|编号|改动|距离|导引|预期验证|
|---|---|---|---|---|
|B0|复现 B 原始设置|50-100|TTC/VM|基线|
|B1|关闭 LOS filter|50-100|TTC/VM|隔离 LOS 门控影响|
|B2|云台 yaw 误差反馈机体 yaw-rate|50-100|TTC/VM|验证机体对准是否提升命中|
|B3|目标接近边缘时降速 + frame guard|50-100|TTC/VM|验证末端看住优先|
|B4|bbox clipped/area large 后不清零，切 VM fallback|50-100|TTC|验证 TTC 面积失败处理|
|B5|bbox clipped/area large 后 terminal blind push 0.3-0.5s|50-100|TTC/VM|验证末端惯性盲推|

### B-判读标准

- 如果 B2 后最近点前 `target_body_bearing_deg` 从 `40-55deg` 降到 `5-10deg`，且最小距离接近 C，则云台-机体解耦是主因。
- 如果 B1 提升明显，则 LOS filter 是重要因素。
- 如果 B4/B5 提升 TTC，但 VM 变化不大，则 TTC 面积通道是主要短板。
- 如果所有 B 改动仍停在 `2.4-2.6m`，需要检查碰撞几何或目标/拦截机碰撞半径。

## 3. CDE 线：识别、LOS 滤波、频率、推力饱和验证

### CDE-目标

以 C 组为内部最优 baseline：

```text
C = strapdown + AirSim detect + no LOS filter + 8Hz + 原始权限
TTC/VM = 6/6
```

逐项加回复杂度，找出历史 YOLO body-rate baseline `4/6` 的损失来源。

### CDE-实验矩阵

|编号|检测|LOS filter|频率|控制权限|导引|预期验证|
|---|---|---|---:|---|---|---|
|C0|AirSim detect|关|8Hz|原始|TTC/VM|复现 C，内部上限|
|C1|YOLO + ByteTrack|关|8Hz|原始|TTC/VM|识别连续性单独影响|
|C2|YOLO + ByteTrack|relaxed，末端 bypass|8Hz|原始|TTC/VM|LOS 平滑但不硬拒绝|
|C3|YOLO + ByteTrack|relaxed 全程硬门控|8Hz|原始|TTC/VM|复现历史 LOS 门控问题|
|C4|YOLO + ByteTrack|关|8Hz|推力饱和保护|TTC/VM|验证降需求/留余量|
|C5|YOLO + ByteTrack|关|12Hz|原始|TTC/VM|小幅频率提升|
|C6|YOLO + ByteTrack|关|20Hz|原始|TTC/VM|验证高频是否仍退化|
|C7|YOLO + ByteTrack|关|8Hz|高权限|TTC/VM|复现 D 类高权限效应|
|C8|YOLO + ByteTrack|关|8Hz|高推力但限加速度|TTC/VM|验证是否可增推力但不饱和|

### CDE-控制权限策略

不建议直接把油门和最大推力全部拉高。测试顺序应为：

1. 原始权限复现 C。
2. 增加推力饱和保护：
   - 推力接近上限时先削 `speed_hold`。
   - 再按比例削 PNG 横向加速度。
   - 保留 `0.10-0.15` 推力余量给角速度控制。
3. 仅在第 2 步仍不足时，测试更高 `max_total_thrust`。
4. 高推力实验必须同时记录：
   - `thrust_saturation_rate`
   - `body_rate_p/q/r`
   - 实际姿态响应
   - 实际过载

### CDE-LOS 滤波策略

建议测试三种：

|模式|说明|是否推荐|
|---|---|---|
|no LOS filter|纯仿真/理论检测下的上限|推荐作为内部 baseline|
|relaxed + terminal bypass|非末端滤波，末端 raw LOS/image KF 接管|推荐作为 YOLO baseline|
|hard gate|创新过大直接 invalid|不推荐作为默认，只用于复现实验|

判读重点：

- 如果 `los_innovation_reject` 出现在最近点前 `1s` 内并导致 `a_cmd=0`，说明滤波硬门控不可保留。
- 如果关闭 LOS filter 后 YOLO 明显提升，说明识别噪声可接受，门控比噪声更危险。
- 如果关闭 LOS filter 后抖动严重，再考虑非末端滤波和末端 bypass。

## 4. 推荐执行顺序

第一轮只跑少量距离，快速定位：

1. A1：truth 几何命中，距离 `50/80/100`，TTC/VM。
2. B1/B2：云台 no LOS filter + yaw feedback，距离 `70/90/100`，TTC/VM。
3. C0/C1/C2：捷联 AirSim detect 和 YOLO 对照，距离 `50/60/70/80/90/100`，TTC。

第二轮完整覆盖：

1. 对第一轮有效配置跑 `50-100m` 全距离。
2. TTC 和 VM 都跑。
3. 生成统一报告，按同一张表比较：
   - hit
   - geometric_hit
   - min_range
   - detection/valid/body-rate rate
   - saturation
   - LOS reject
   - terminal state

第三轮才做控制权限/频率：

1. C4：推力饱和保护。
2. C5：12Hz。
3. C6：20Hz。
4. C8：高推力但限加速度。

## 5. 成功标准

短期目标：

- A：明确 `geometric_hit` 与 collision hit 的差异，给出可靠评价口径。
- B：云台组 TTC/VM 至少达到 `5/6`，最近点前 `target_body_bearing_deg < 10deg`。
- CDE：YOLO body-rate TTC 从历史 `4/6` 提升到 `5/6` 或 `6/6`。

长期目标：

- YOLO + ByteTrack + body-rate TTC 在 `50-100m` 全距离 `6/6`。
- `thrust_saturation_rate < 15%`。
- 最近点前 `0.5s` 内无硬性 `los_innovation_reject -> invalid`。
- 失败工况能区分：
  - 几何未命中
  - collision 未记录
  - 识别断续
  - LOS/KF 门控
  - 推力/角速度饱和
  - 频率/相位滞后

