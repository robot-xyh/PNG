# Betaflight LOG00106 AirSim LOG_ONLY 趋势对比报告

## 1. 证据边界

本报告基于 LOG00106 单次真实物理接触样本建立 AirSim 趋势复现。`simGetDetections` 提供
`airsim_truth_box`，框中心经 AirSim 渲染内参归一化后映射到实机内参，再进入生产 LOS Kalman、
固定速度 PNG、`accel_tilt_rate` 和 0.8 s 角速度/油门交接。没有运行 YOLO/RKNN，也没有访问
真实飞控。理想针孔表示无额外畸变和噪声，不表示绕过检测函数。

真实数据、仿真数据和推断量严格区分：真实距离仅引用接触锚定位置增量；表内最近距离全部为
AirSim 双机真值；有限差分比力和趋势判定属于推断量。

## 2. 配置与复现

|项目|值|
|---|---|
|run ID|`run_20260904T161821Z`|
|AirSim|Python `1.8.1`，client/server `1/1`|
|模式|`Multirotor/SimpleFlight`，RPC `127.0.0.2:41451`|
|seed|`106`|
|图像/相机|`640x512`，`fx=530.8443`、`fy=532.2955`、固定上视 `R_BC`|
|导引|`N=3`、`fixed_vm=10 m/s`、总加速度 `7 m/s2`|
|目标|静止 Actor；调整尺寸仅是仿真视觉代理，不代表真实靶机尺寸|
|框来源|AirSim `simGetDetections` 真值框；未运行 YOLO/RKNN|
|运行命令|`/home/linux/Documents/PNG-betaflight-upward-camera/examples/run_airsim_log00106_log_only.py --config /home/linux/Documents/PNG-betaflight-upward-camera/config/airsim_log00106_log_only_cases.json --output-root /home/linux/Documents/PNG-betaflight-upward-camera/logs/analysis/LOG00106_airsim_log_only --connection-timeout-s 45`|
|一键复现|`./run_log00106_airsim_log_only.sh test`，再依次运行 `smoke` 和 `full`|
|配置 SHA256|`cfb9ac48dbf327b237b54c973d46f635990f7ac9038659f3074111c900724784`|
|Settings SHA256|`d6c53da85937be5dfa4bc5edc4e4eeb324f6d5c25398b31584e365c11138f898`|
|原始输出|`logs/analysis/LOG00106_airsim_log_only/run_20260904T161821Z`|

理想组使用 30 Hz 即时框和高频真值速度。两组敏感性使用主 CSV 的 40 组
`sample/available/result_age/fusion_wait` 配对时序，结果年龄已经包含融合等待；速度观测为 5 Hz、
`tau=0.25 s`，角速度命令延迟 15 ms。推力比 0.809 为主工况，0.847 为乐观边界，电压只保留
`22.65--22.95 V` 标签。

输入文件校验如下；`joint_*` 是接触锚定派生产物，其余为主 CSV/meta 和原始飞控日志。

|输入标识|SHA256|
|---|---|
|`interceptor_blackbox`|`fc58d049df776a6a771f312ec5ad71bbd96763b5db7ed60f1b00116f6ed748ec`|
|`joint_events`|`ff67b81eb712353024c0459936842ff2b01d8ce03ec80161f36332455cb09c2c`|
|`joint_metrics`|`9b9a4c9c9d39bdd54d45c9279d3e75ef6363a4700675d24a0c09d10ef1334d59`|
|`joint_timeseries`|`fee380cfc5fa12e753bf4440f2f026b11836baf7669066ed7164d6daa2da128c`|
|`main_csv`|`2f4cba58655be4d142237fc5e05c7ad16b0c3c9131a838d1413e9676cdc557a6`|
|`meta_json`|`109766c2c003e6c67060dc06e538c2c33944cf66b661b22e204a2708d7292238`|
|`target_ulog`|`366fbea3ab9d322efe7e597161a32ad48e1caaf3a627b9bed990f715ced7ec96`|

## 3. 实测参考

|实测量|LOG00106 结果|
|---|---:|
|算法发布时长|1.670804 s|
|接触锚定初始剩余位移|约 5.06 m；气压高度敏感性约 6.34 m|
|算法退出时剩余位移|约 3.27--3.39 m|
|退出至物理接触|约 0.902 s|
|速度建立/总加速度饱和|65.48% / 72.62%|
|Rate 同钟滞后和相关|15 ms；Roll/Pitch 0.991/0.994|
|交接后实测/模型比力|P50 0.809；P95 边界 0.847|

以上距离不是两机绝对 GPS 差。碰撞后的 963 deg/s、电机 158/2047 和 82.06 A 已从 PNG 对比窗
排除。

## 4. 仿真结果

|工况|距离|退出|感知时序|比力比例|结果|AirSim 最近距离 (m)|接触时间 (s)|退出剩余距离 (m)|最大总加速度 (m/s2)|最大油门 (us)|
|---|---:|---|---|---:|---|---:|---:|---:|---:|---:|
|ideal_5p06_continuous|5.06m|持续闭环|即时|1.000|接触|0.358|1.905|-|7.000|1351.3|
|optimistic0847_5p06_continuous|5.06m|持续闭环|40 组实测配对时序|0.847|接触|0.404|2.217|-|7.000|1383.5|
|measured0809_5p06_continuous|5.06m|持续闭环|40 组实测配对时序|0.809|接触|0.465|2.379|-|7.000|1382.9|
|ideal_5p06_early|5.06m|提前退出|即时|1.000|接触|0.347|1.917|1.278|7.000|1351.2|
|optimistic0847_5p06_early|5.06m|提前退出|40 组实测配对时序|0.847|接触|0.371|2.340|2.113|7.000|1381.7|
|measured0809_5p06_early|5.06m|提前退出|40 组实测配对时序|0.809|接触|0.490|2.619|2.437|7.000|1383.0|
|ideal_6p34_continuous|6.34m|持续闭环|即时|1.000|接触|0.310|2.241|-|7.000|1349.9|
|optimistic0847_6p34_continuous|6.34m|持续闭环|40 组实测配对时序|0.847|接触|0.442|2.595|-|7.000|1379.6|
|measured0809_6p34_continuous|6.34m|持续闭环|40 组实测配对时序|0.809|接触|0.514|2.850|-|7.000|1381.7|
|ideal_6p34_early|6.34m|提前退出|即时|1.000|接触|0.323|2.256|2.528|7.000|1349.9|
|optimistic0847_6p34_early|6.34m|提前退出|40 组实测配对时序|0.847|接触|0.463|2.877|3.358|7.000|1380.8|
|measured0809_6p34_early|6.34m|提前退出|40 组实测配对时序|0.809|未接触|0.749|-|3.654|7.000|1382.9|

提前退出和持续闭环是两类独立结论，不合并计算命中率。`contact` 只接受 AirSim 返回且对象名匹配
目标 Actor 的碰撞；未接触组只报告最近点或安全超时。

## 5. 趋势验收

- `5.06m / continuous`：推力偏差方向性检查为 **通过**；该判定只比较当前 AirSim 三条曲线，不回填或平移实测数据。
- `5.06m / early`：推力偏差方向性检查为 **通过**；该判定只比较当前 AirSim 三条曲线，不回填或平移实测数据。
- `6.34m / continuous`：推力偏差方向性检查为 **通过**；该判定只比较当前 AirSim 三条曲线，不回填或平移实测数据。
- `6.34m / early`：推力偏差方向性检查为 **通过**；该判定只比较当前 AirSim 三条曲线，不回填或平移实测数据。

每组 `metrics.json` 还给出上视 D 轴、7 m/s2 上限、1500 us 上限、速度建立主导和 Rate 相关性
检查。本轮全部硬检查为 **通过**；12 组平均控制周期范围为
`19.447--19.822 ms`。敏感性组额外注入 15 ms 命令延迟后，
AirSim 飞控端到端最优滞后为 Roll `26--29 ms`、Pitch
`29--31 ms`，最低相关系数 `0.990`；该值包含
SimpleFlight 动态，不能解释成只有 15 ms 的纯延迟。`t_contact_s` 对接触组以 Actor 接触为零，
未接触组以 AirSim 真值最近点为零。LOS rate 不要求单调归零；静止目标下仍受拦截机平移、姿态变化、
感知年龄和终端几何影响。

## 6. 曲线

### 5.06m / 持续闭环
![角速度设定与 AirSim 响应；点线为算法退出，虚线为 Actor 接触。](figures/log00106_airsim_log_only/5p06m_continuous_01_rates.png)

角速度设定与 AirSim 响应；点线为算法退出，虚线为 Actor 接触。
![模型目标油门、交接输出、AirSim 油门及有限差分比力。](figures/log00106_airsim_log_only/5p06m_continuous_02_throttle_force.png)

模型目标油门、交接输出、AirSim 油门及有限差分比力。
![NED 速度参考、SimpleFlight 真值速度及 5 Hz 滤波观测速度。](figures/log00106_airsim_log_only/5p06m_continuous_03_velocity.png)

NED 速度参考、SimpleFlight 真值速度及 5 Hz 滤波观测速度。
![速度建立、PNG、FOV 和总加速度的 N/E/D 分量与模长。](figures/log00106_airsim_log_only/5p06m_continuous_04_acceleration.png)

速度建立、PNG、FOV 和总加速度的 N/E/D 分量与模长。
![真值 LOS、检测框针孔 LOS、Kalman LOS/LOS rate 和框尺度。](figures/log00106_airsim_log_only/5p06m_continuous_05_los_bbox.png)

真值 LOS、检测框针孔 LOS、Kalman LOS/LOS rate 和框尺度。
![饱和、ACTIVE/退出、Actor 接触、真值距离与闭合速度。](figures/log00106_airsim_log_only/5p06m_continuous_06_state_range.png)

饱和、ACTIVE/退出、Actor 接触、真值距离与闭合速度。
### 5.06m / 提前退出
![角速度设定与 AirSim 响应；点线为算法退出，虚线为 Actor 接触。](figures/log00106_airsim_log_only/5p06m_early_01_rates.png)

角速度设定与 AirSim 响应；点线为算法退出，虚线为 Actor 接触。
![模型目标油门、交接输出、AirSim 油门及有限差分比力。](figures/log00106_airsim_log_only/5p06m_early_02_throttle_force.png)

模型目标油门、交接输出、AirSim 油门及有限差分比力。
![NED 速度参考、SimpleFlight 真值速度及 5 Hz 滤波观测速度。](figures/log00106_airsim_log_only/5p06m_early_03_velocity.png)

NED 速度参考、SimpleFlight 真值速度及 5 Hz 滤波观测速度。
![速度建立、PNG、FOV 和总加速度的 N/E/D 分量与模长。](figures/log00106_airsim_log_only/5p06m_early_04_acceleration.png)

速度建立、PNG、FOV 和总加速度的 N/E/D 分量与模长。
![真值 LOS、检测框针孔 LOS、Kalman LOS/LOS rate 和框尺度。](figures/log00106_airsim_log_only/5p06m_early_05_los_bbox.png)

真值 LOS、检测框针孔 LOS、Kalman LOS/LOS rate 和框尺度。
![饱和、ACTIVE/退出、Actor 接触、真值距离与闭合速度。](figures/log00106_airsim_log_only/5p06m_early_06_state_range.png)

饱和、ACTIVE/退出、Actor 接触、真值距离与闭合速度。
### 6.34m / 持续闭环
![角速度设定与 AirSim 响应；点线为算法退出，虚线为 Actor 接触。](figures/log00106_airsim_log_only/6p34m_continuous_01_rates.png)

角速度设定与 AirSim 响应；点线为算法退出，虚线为 Actor 接触。
![模型目标油门、交接输出、AirSim 油门及有限差分比力。](figures/log00106_airsim_log_only/6p34m_continuous_02_throttle_force.png)

模型目标油门、交接输出、AirSim 油门及有限差分比力。
![NED 速度参考、SimpleFlight 真值速度及 5 Hz 滤波观测速度。](figures/log00106_airsim_log_only/6p34m_continuous_03_velocity.png)

NED 速度参考、SimpleFlight 真值速度及 5 Hz 滤波观测速度。
![速度建立、PNG、FOV 和总加速度的 N/E/D 分量与模长。](figures/log00106_airsim_log_only/6p34m_continuous_04_acceleration.png)

速度建立、PNG、FOV 和总加速度的 N/E/D 分量与模长。
![真值 LOS、检测框针孔 LOS、Kalman LOS/LOS rate 和框尺度。](figures/log00106_airsim_log_only/6p34m_continuous_05_los_bbox.png)

真值 LOS、检测框针孔 LOS、Kalman LOS/LOS rate 和框尺度。
![饱和、ACTIVE/退出、Actor 接触、真值距离与闭合速度。](figures/log00106_airsim_log_only/6p34m_continuous_06_state_range.png)

饱和、ACTIVE/退出、Actor 接触、真值距离与闭合速度。
### 6.34m / 提前退出
![角速度设定与 AirSim 响应；点线为算法退出，虚线为 Actor 接触。](figures/log00106_airsim_log_only/6p34m_early_01_rates.png)

角速度设定与 AirSim 响应；点线为算法退出，虚线为 Actor 接触。
![模型目标油门、交接输出、AirSim 油门及有限差分比力。](figures/log00106_airsim_log_only/6p34m_early_02_throttle_force.png)

模型目标油门、交接输出、AirSim 油门及有限差分比力。
![NED 速度参考、SimpleFlight 真值速度及 5 Hz 滤波观测速度。](figures/log00106_airsim_log_only/6p34m_early_03_velocity.png)

NED 速度参考、SimpleFlight 真值速度及 5 Hz 滤波观测速度。
![速度建立、PNG、FOV 和总加速度的 N/E/D 分量与模长。](figures/log00106_airsim_log_only/6p34m_early_04_acceleration.png)

速度建立、PNG、FOV 和总加速度的 N/E/D 分量与模长。
![真值 LOS、检测框针孔 LOS、Kalman LOS/LOS rate 和框尺度。](figures/log00106_airsim_log_only/6p34m_early_05_los_bbox.png)

真值 LOS、检测框针孔 LOS、Kalman LOS/LOS rate 和框尺度。
![饱和、ACTIVE/退出、Actor 接触、真值距离与闭合速度。](figures/log00106_airsim_log_only/6p34m_early_06_state_range.png)

饱和、ACTIVE/退出、Actor 接触、真值距离与闭合速度。

## 7. 结论限制

本轮只能说明指定初始几何和单个 seed 下，理想/敏感性模型相对 LOG00106 的方向、时序、幅值及
饱和趋势。LOG00106 只有一个真实接触样本；无论 AirSim 某组是否接触，都不能宣称真实命中率达到
80%。概率命中率需要预定义场景分布、多个 seed 和独立真实飞行样本另行验证。
