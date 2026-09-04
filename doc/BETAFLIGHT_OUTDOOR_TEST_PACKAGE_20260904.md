# 2026-09-04 Betaflight 外场实验交接索引

## 1. 交付范围

本索引汇总 `2026-09-04` 外场主动接管实验的归档、最后架次专题分析、技术问答和 AirSim
复现资料。原始大型日志保留在原目录，文档通过路径、时间戳和 SHA256 建立引用关系，不复制或
改写原始证据。

## 2. 建议阅读顺序

1. [全天外场归档与审计](BETAFLIGHT_OUTDOOR_FIELD_TEST_ARCHIVE_20260904.md)：读取当天全部
   主机日志、拦截机 Blackbox 和靶机 ULog 的配对关系、逐架次结果、共性问题及 P0/P1/P2 改进。
2. [LOG00106 碰撞联合分析](BETAFLIGHT_FLIGHT_ACTIVE_LOG00106_COLLISION_ANALYSIS.md)：读取
   最后一次接管、退出、惯性闭合和物理接触的完整时序。
3. [LOG00106 期望与实际响应报告](BETAFLIGHT_LOG00106_EXPECTED_ACTUAL_RESPONSE_REPORT.md)：
   查看角速度、油门、比力和导引曲线及定量结果。
4. [LOG00106 技术问答](BETAFLIGHT_LOG00106_TECHNICAL_QA.md)：回答 MSP 接管、姿态外环、
   Pitch 跟踪、LOS rate、NED 加速度、FOV、饱和和油门交接等 13 项问题。
5. [LOG00106 AirSim 交接说明](BETAFLIGHT_LOG00106_AIRSIM_HANDOFF.md)：提供仿真输入、字段合同、
   趋势验收标准和可直接交给另一位编码智能体的提示词。

## 3. 机器可读证据与图

- [控制响应指标](evidence/BETAFLIGHT_LOG00106_CONTROL_RESPONSE_metrics.json)
- [角速度跟踪图](figures/log00106_control_response/01_angle_rate_tracking.png)
- [油门、比力与动力响应图](figures/log00106_control_response/02_throttle_and_load_response.png)
- [导引、惯性闭合与接触时序图](figures/log00106_control_response/03_guidance_closure_timeline.png)
- 绘图复现脚本：`tools/plot_log00106_control_response.py`
- 双机联合指标：`logs/analysis/LOG00106_target_joint/metrics.json`
- 双机 50 Hz 联合序列：`logs/analysis/LOG00106_target_joint/joint_timeseries_50hz.csv`

## 4. 结论边界

- **实测：** 最后架次 PNG 实际发布 `1.670804 s`，停止发布后 `0.902322 s` 发生物理接触；
  Roll/Pitch 的 Betaflight 同钟 setpoint 到 gyro 延迟均约 `15 ms`。
- **工程判断：** 速度建立项和总加速度长期饱和，推力模型在该瞬态工况高估约 `19%`，是下一轮
  优先改进对象。
- **不能证明：** 单次接触不能证明真实命中率达到 `80%`，也不能证明横移或规避目标下具有相同
  成功率；两机绝对 GPS 不能直接相减得到可信脱靶距离。

## 5. 问题闭环矩阵

状态定义：`代码已修复` 只表示实现和单元测试通过；`回放已验证` 表示旧日志输入经过新实现得到
预期结果；只有 `实测已验证` 才表示当前提交、当前配置和实机链路已经共同验证。

|问题|代码状态|离线证据|仍缺实测/材料|是否阻止主动控制|
|---|---|---|---|---|
|旧控制器速度建立项 `76.85%`、总加速度 `86.45%` 饱和|代码已修复：速度参考斜坡在失效/重获时连续，当前非碰撞控制器提前退出|[十脉冲回放](evidence/BETAFLIGHT_OUTDOOR_PULSE_REPLAY_20260904.json) 覆盖全部 `10` 个算法脉冲；新控制器两项饱和均为 `0%`|当前代码的 LOG_ONLY 与短时闭环架次|是，直到新实测完成|
|LOG00106 近距继续闭合并碰撞|代码已修复：`noncollision` 按 bbox/TTC 进入锁存 ABORT|同一回放在接触前 `1.085447 s` ABORT，高于 `0.75 s` 门槛|新控制器实机 ABORT 后飞手 RC7 退出及安全间隔|是|
|旧推力模型瞬态高估约 `19%`|批准和运行时已强制电压相关 `voltage_throttle_lut`；MC 动力学也只允许同一 LUT，不再回退分段线性载荷因子|单元测试覆盖 LUT 正向比力、反向油门、哈希和电压越界拒绝|合格的 `20.0--25.2 V` 全 6S 推力 LUT，至少 `100` 个留出验证样本|是；当前配置故意指向不存在的 `betaflight.thrust_lut.pending.json`|
|旧 MC 报告为 schema v2、旧配置哈希且混淆 contact/noncollision|代码已修复：MC 输出 schema v3，绑定运行配置、控制器/仿真/runner 源码及 LUT 哈希；contact 命中率与 noncollision 及时 ABORT 分栏|策略分栏和批准端篡改拒绝单元测试通过|装入真实 LUT 后重跑 MC100；contact hit 与 FOV-hit 在三个场景均须 `>=80%`，noncollision 及时 ABORT 率须 `>=99%`|是|
|旧运行缺少可持久复核的图像和完整结束证据|代码已修复：CSV/events/meta/JPEG/frame index/manifest 原子终结，关键文件 `fsync`，Blackbox 固件模式绑定|日志、finalize、runtime evidence 专项测试通过|用当前干净提交采集 finalized LOG_ONLY 架次：至少 `100` 行、`25` 帧、唯一 Blackbox 配对|是|
|旧主动批准可与代码/证据漂移|旧批准链已退役；schema v4 在创建和启动时复核配置、MC、RC 联锁、finalized 架次、LUT 和文件哈希|批准工具专项测试通过|新快照、新 RC 联锁证据、新 MC100、新 finalized 架次，随后重新生成批准文件|是|
|单 UART 主动窗口 50 Hz 调度|当天旧版本主动窗口 ACK `406/406`，写入间隔最大 `20.106--27.772 ms`，未发现写入错误|外场原始 CSV 审计|当前提交 finalized 架次再次确认均值 `>=49 Hz`、P99.9 `<=40 ms`、最大 `<=60 ms`|在新证据完成前仍阻止批准|
|`publish=passthrough` 不等于 RC7 已回人工侧|语义已写入 noncollision 报告：ABORT 后仍要求飞手动作，contact 仿真不得作为接触飞行授权|LOG00106 证明算法停止后 RC7 仍高约 `1.67 s`，其中 `0.90 s` 后发生接触|飞手必须在异常时立即释放 RC7；需新 RC 联锁报告证明释放延迟 `<=200 ms`|是|
|相机时间戳不是硬件曝光时刻|未伪造修复；MC 保留实测和保守延迟场景|现有闭环响应仅能估计 P50/P95，不能称为硬件同步测量|后续硬件时间同步或闪光/光电延迟测量；当前先采用 `154.139 ms` 保守场景|作为模型局限记录，不单独替代上述硬门槛|

当前监督配置 SHA256 为
`fa5e3ee90a0fb7b5657b073da8bd91506712758d3c14fd1bbc7a2440fe4c8550`。由于推力 LUT
仍为 `PENDING_FULL_6S_THRUST_LUT`，正式 MC 配置会在任务创建前明确失败，批准工具也会拒绝；不得
创建假 LUT、使用旧 MC 报告或手工编辑批准 JSON 绕过。

恢复主动监督测试的固定顺序为：完成推力 LUT并更新配置哈希；运行 schema v3 MC100；在当前干净
提交上采集 finalized LOG_ONLY 架次；采集当天含 GPS/电压/运动学的 Betaflight 快照和 RC7 联锁
证据；最后生成新的 schema v4 批准文件。上述任一步缺失时只允许运行 LOG_ONLY。

## 6. 交接约束

AirSim 对比应复用仓库生产几何、LOS、速度建立 PNG 和 `accel_tilt_rate` 实现，只在独立仿真配置
与脚本中工作。仿真结果必须与实测分栏，碰撞后飞控恢复数据不得用于拟合 PNG，且不得修改实机
runner、批准文件或原始日志。
