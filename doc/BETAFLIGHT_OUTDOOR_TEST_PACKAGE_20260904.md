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

## 5. 交接约束

AirSim 对比应复用仓库生产几何、LOS、速度建立 PNG 和 `accel_tilt_rate` 实现，只在独立仿真配置
与脚本中工作。仿真结果必须与实测分栏，碰撞后飞控恢复数据不得用于拟合 PNG，且不得修改实机
runner、批准文件或原始日志。
