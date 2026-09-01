# Betaflight PNG 离线闭环 matrix15 测试

## 目的与结论边界

本测试回答两个独立问题：当前 `accel_tilt_rate` 是否能把固定 Vm PNG 加速度变成闭环机体运动，以及
当前固定油门链相对 matrix15/PX4 完整控制器还缺什么。工具
`tools/simulate_betaflight_png_matrix15.py` 使用真值 LOS、点质量平动模型和一阶 body-rate 响应，
直接复用生产 `guidance_eval_to_setpoint(..., mapping_type="accel_tilt_rate")`。结果不经过相机、
YOLO、ByteTrack、LOS KF、串口或 Betaflight PID，因此是控制权威性上界，不是命中率预测或飞行批准。

## 模型与算法

15个初始位置、目标速度和 `speed_ratio=2` 与
`YOLO_ByteTrack_upward_matrix15_多工况性能测试报告.md` 一致。目标沿惯性系 `+Y` 匀速运动，拦截机
从悬停或已建立 `Vm*lambda` 速度开始。每个100 Hz仿真步执行：

```text
r = p_target - p_interceptor
lambda = r / |r|, omega_LOS = (r x v_rel) / |r|^2
a_png = clip[N * Vm * (omega_LOS x lambda), 20 m/s2]
a_center,B = clip[8 * (lambda_B,x, lambda_B,y, 0), 4 m/s2]
v_ref = Vm * lambda, |v_ref,z| <= 6 m/s
a_speed = clip[1.2 * (v_ref - v), 8 m/s2]
a_total = clip[a_png + a_center + a_speed, 28 m/s2]
```

`a_total` 经当前 NED/FRD 加速度到目标倾角、姿态 P 到 p/q 的实现转换。实际 p/q 使用40 ms一阶响应；
平动模型为 `a = g_NED - (T/m) * body_down_axis`。比较三种控制路径：

- `fixed_thrust`：当前 Betaflight PNG 形态，只输入 `a_png`，比推力固定为 `g`。
- `ideal_altitude_hold`：仍只有 `a_png`，另用理想垂向 PD 固定初始高度，用于证明“只补高度保持”是否足够。
- `speed_hold_variable_thrust`：加入上视居中、三维 LOS 速度保持和按需推力；这是理想力投影参考，
  不是已经实现的 Betaflight 电机模型。

`hit` 使用1.0 m真值半径，`near`使用1.5 m。`FOV feasible hit`要求目标从初始时刻到命中始终位于
上视相机120 deg视场内；它不包含检测置信度，仍比真实视觉条件乐观。

## 总体结果

### matrix15/PX4 包线：35 deg、120 deg/s

|初始状态|控制路径|真值 hit|全程 FOV hit|平均/最差最小距离 m|平均倾角/Rate饱和|
|---|---|---:|---:|---:|---:|
|悬停|固定油门|0/15|0/15|50.328 / 61.695|0.0% / 0.01%|
|悬停|理想高度保持|0/15|0/15|49.052 / 59.473|0.0% / 0.01%|
|悬停|速度保持+可变推力|15/15|12/15|0.957 / 0.999|9.86% / 1.12%|
|已建立速度|固定油门|15/15|10/15|0.980 / 0.995|2.09% / 0.65%|
|已建立速度|理想高度保持|0/15|0/15|35.000 / 45.164|0.0% / 0.0%|
|已建立速度|速度保持+可变推力|15/15|11/15|0.950 / 0.995|3.21% / 0.0%|

### 当前保守候选包线：20 deg、60 deg/s

|初始状态|控制路径|真值 hit|全程 FOV hit|平均/最差最小距离 m|平均倾角/Rate饱和|
|---|---|---:|---:|---:|---:|
|悬停|固定油门|0/15|0/15|50.325 / 61.695|0.53% / 0.13%|
|悬停|理想高度保持|0/15|0/15|49.132 / 59.473|0.53% / 0.13%|
|悬停|速度保持+可变推力|15/15|9/15|0.962 / 0.997|47.39% / 3.09%|
|已建立速度|固定油门|15/15|10/15|0.984 / 0.999|12.15% / 1.46%|
|已建立速度|理想高度保持|0/15|0/15|34.945 / 45.164|1.99% / 0.13%|
|已建立速度|速度保持+可变推力|15/15|9/15|0.959 / 0.998|38.21% / 5.12%|

20 deg完整参考路径从悬停时，FOV失败为 `M03/M06/M07/M12/M13/M14`。其中 `M03/M07/M14`
初始几何离上轴约61.4/63.4/70.0 deg，本来就在60 deg半视场外；其余三项是在闭环中越界。
`M13`最大离轴143.2 deg且倾角饱和80.1%，说明真值 `hit=1` 不能转换为真实视觉可实现结论。

## 实测延迟与视场门控复测

原结果每10 ms直接向控制器提供真值LOS，即使目标出视场仍继续制导，属于明显乐观上界。模拟器现增加
独立测量链：30 Hz曝光采样、可配置固定延迟、120 deg总视场门控和0.35 s目标过期。目标在曝光时刻
出视场则不产生测量；最后有效测量过期后法向制导、上视居中和速度参考全部撤销。延迟测量仍是真值，
不含YOLO误差、ByteTrack错配、漏检和串口抖动，因此结果仍偏乐观。

使用2026-08-30 LOG_ONLY实测结果年龄P50约175 ms、P95约217 ms、最大约225 ms，在20 deg、
60 deg/s包线下得到：

|延迟|当前悬停 fixed-thrust hit / FOV hit|理想悬停速度保持 hit / FOV hit|理想悬停初始可见 hit|已建立速度 fixed-thrust hit / FOV hit|
|---:|---:|---:|---:|---:|
|0 ms|0/15 / 0/15|11/15 / 9/15|11/12|12/15 / 10/15|
|175 ms|0/15 / 0/15|8/15 / 7/15|8/12|12/15 / 10/15|
|200 ms|0/15 / 0/15|8/15 / 7/15|8/12|10/15 / 9/15|
|225 ms|0/15 / 0/15|8/15 / 7/15|8/12|10/15 / 9/15|

`M03/M07/M14`初始即在视场外，当前固定上视相机无法启动跟踪。200 ms理想悬停路径中，`M05/M10`
仅达到1.404/1.265 m近失，`M12/M13`在约4.76/6.49 s后目标过期，最小距离2.357/2.076 m。
延迟使理想悬停路径由零延迟11/15降至8/15；初始可见工况也只有8/12，不能满足稳健拦截要求。
已建立速度路径在175--225 ms间从12/15降到10/15，说明其命中判据对延迟敏感，而且实机当前没有
速度估计与三维速度保持来满足“已建立速度”前提。

## 随机扰动Monte Carlo

确定性延迟结果之后，进一步加入5%--20%独立漏检、0.25--1.0 deg LOS角噪声、0.25--1.0 m/s
相对速度噪声和0.25--1.0 m/s2一阶相关风扰动。4个场景、3条路径、15工况、20 seeds/case共
3600条结果。当前悬停fixed-thrust在初始可见工况中的命中率仅4.6%--8.3%，全程FOV命中率仅
1.3%--6.7%；尚未实装的理想完整控制在100 ms场景也只有70.8%，实测P50为65.8%。所有场景
`release_passed=false`。完整场景、Wilson区间、放行门限和结果见
`doc/BETAFLIGHT_INTERCEPTION_TEST_PLAN.md`。

## 结论

1. 固定 Vm PNG 本身主要给出 LOS 法向加速度，不负责从悬停建立闭合速度。当前固定油门路径从悬停
   `0/15`，所以现有 Betaflight 运行时尚不能复现 matrix15 完整三维控制器。
2. 接管前若已经精确建立三维 `Vm*lambda` 速度，当前固定油门模型可得到 `15/15` 真值命中；但仅
   `10/15` 全程在视场内，且实机没有速度估计/保持来保证此前提。
3. 只加入高度保持仍为 `0/15`。必须提供朝目标高度的垂向速度参考及随倾角、垂向加速度变化的推力，
   不能把固定悬停 PWM 当成三维控制器。
4. 20 deg包线的主要限制是倾角和固定上视可观测性。body-rate响应时间常数20/40/80 ms下，完整参考
   路径均为真值 `15/15`、FOV `9/15`；该范围内 rate响应不是当前主导项。
5. 加入实测视觉延迟和真正的FOV输入门控后，尚未实现的理想完整路径也仅为`8/15`，当前实现从悬停
   仍为`0/15`。因此当前仓库不能宣称具备足够的实机自主拦截能力。

## 速度建立型候选实现边界

新增 `candidate_velocity_hold_variable_thrust` 后，理想 `speed_hold_variable_thrust` 不再作为发布
替身。候选与生产接口一致的部分包括：生产 6D LOS Kalman、30 Hz带噪/延迟/漏检 LOS、5 Hz带
延迟/噪声/丢包自机 NED 速度、连续5帧获取、0.35 s视觉陈旧、0.5 s速度陈旧和锁存 ABORT。候选
控制器只接收 LOS 与自机速度，不接收仿真真值相对速度；Matrix15 仍复用生产
`accel_tilt_rate` 将加速度变为姿态/rate，并用理想比推力投影完成点质量平动。

这仍不是实机控制实现：GPS/DPS310只接到 LOG_ONLY 日志，推力到PWM未标定，噪声分布未用飞行日志
拟合，Betaflight PID/电机/桨/风场未建模。候选为 `required_for_release=true`，旧 fixed-thrust、
理想速度保持和“预先建立速度”路径均改为诊断项；即使离线通过，也不能直接生成飞行批准文件。

100 seeds/case的正式候选评估共6000条。`target_100ms`初始可见命中/FOV命中为
49.67%/47.67%，实测P50为10.00%/9.33%，P95为1.83%/1.58%，压力场景为0.17%/0%。运动状态
平均有效率依次为98.90%、98.33%、94.46%和84.34%；视觉平均有效率依次为56.14%、23.70%、
19.47%和28.81%。全部场景失败。5 Hz状态在150--250 ms延迟、0.5 s陈旧门限下对单次丢包非常
敏感，锁存ABORT是P50以后主导结果；100 ms场景还存在23.73%平均倾角饱和。该证据否定当前参数
放行，不应通过放宽看门狗或忽略出框样本来提高纸面成功率。

正式报告`logs/betaflight_intercept_candidate_mc100_20260830.json`的SHA256为
`eec27fc77d32c8385fac6b2a53b3da9c9fcc06282cd04816a1baef00e7779d5f`，6000行CSV的SHA256为
`b2114e96d7f1f2ca32025324982c1b0173a8de77e4f48eaf0bdccc4b170340a2`。

## 复现与下一步

最终 JSON/CSV 与20/80 ms敏感性结果归档为
`logs/deployment_archives/betaflight_png_matrix15_offline_20260829.tar.gz`，SHA256为
`09a437eba50e4ad50d65ea82f70791663b3c0a64aea9ea944d471a21f44b9a68`。
实测延迟/FOV门控的0/175/200/225 ms JSON与CSV归档为
`logs/deployment_archives/betaflight_png_matrix15_latency_fov_20260830.tar.gz`，SHA256为
`d48373794fe2d104cb8431769bc5d927d1d2584f52aec663df2fec9e22a7f639`。
随机扰动Monte Carlo结果归档为
`logs/deployment_archives/betaflight_intercept_mc20_20260830.tar.gz`，SHA256为
`d744335cf3a5a47292ecc9339ebd169277135d1b2f1c8186faf65f0f5e2a9365`。

```bash
python3 tools/simulate_betaflight_png_matrix15.py \
  --max-tilt-deg 20 --max-rate-deg-s 60 \
  --perception-latency-s 0.20 --perception-rate-hz 30 \
  --perception-stale-timeout-s 0.35 --perception-fov-gate \
  --output logs/betaflight_png_matrix15_bf20_60.json \
  --csv logs/betaflight_png_matrix15_bf20_60.csv
```

下一实现阶段应先增加有独立开关的 `speed_hold_variable_thrust` 候选接口，输入必须来自经过时间同步的
速度/高度状态，并记录 `v_ref/a_speed/thrust_raw/thrust_limited`及各级饱和。实际参数必须使用质量、
推重比、悬停PWM和飞行数据标定。随后在带延迟、检测丢失和FOV门控的仿真中复测；不能直接把本报告的
理想力投影写入无飞行批准的实机配置。
