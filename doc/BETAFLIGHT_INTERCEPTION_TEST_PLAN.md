# Betaflight PNG 拦截能力测试与放行方案

## 目标与当前判定

本方案回答“从悬停接管后，当前RK3588+固定上视视觉+Betaflight PNG链路能否稳定完成拦截”。
固定无桨台架只验证控制和安全联锁，不能证明命中能力。当前Monte Carlo结论为
`release_passed=false`：速度建立型候选在初始可见工况中的命中率由100 ms场景49.7%降至实测
P50场景10.0%，距离95%门限很远，禁止据此进入自主实机拦截。

当前必测路径已改为 `candidate_velocity_hold_variable_thrust`。旧
`current_hover_fixed_thrust`、`speed_hold_variable_thrust` 和预先建立速度路径只用于解释差距，
不再能单独形成发布结论。候选必须在每场景、每个 Matrix15 工况至少运行100个固定seed，并同时
通过命中/FOV、陈旧、状态有效率、最小距离和饱和门限；状态噪声尚未由实飞数据拟合时，即使数学
门限通过，`flight_candidate.parameters.json` 仍保持 `runnable=false`。

候选控制流程为：生产 LOS KF 输出 `lambda/lambda_dot`；只读运动状态提供 NED 速度；控制器分别
计算 `a_speed=Kv(Vm*lambda-v)`、`a_png=N*Vm*lambda_dot` 和上视视场居中加速度，再做逐项及总
加速度限幅。状态机为连续5帧获取、加速、达到0.8 Vm后PNG跟踪；视觉/速度陈旧或姿态/数值错误
进入锁存ABORT。该路径只存在于离线仿真，严禁连接当前实机RC发布函数。

## 已实施的离线评估链

`vision_guidance/betaflight_png_sim.py`直接复用生产`accel_tilt_rate`映射，并加入以下闭环因素：

- 30 Hz LOS曝光采样、固定延迟队列、120 deg总视场和0.35 s目标过期；
- 可重复的随机漏检、LOS角噪声；诊断路径使用相对速度噪声，候选使用独立自机速度噪声/延迟/丢包；
- 一阶相关风加速度、20 deg倾角、60 deg/s角速度及40 ms body-rate响应；
- 1.0 m命中、1.5 m近失、视场保持、倾角/rate饱和和明确失效原因。

每个场景、工况和trial使用稳定随机种子；漏检、测量噪声和风使用独立随机流。评估工具并行运行并
输出逐trial CSV、Wilson 95%置信区间及机器可读门限检查。

## 测试场景

配置文件为`config/betaflight.intercept_eval.example.json`。matrix15包含25--55 m水平距离、20--40 m
高度差、-20--20 m侧向偏置和3--7 m/s目标速度。正式候选每个工况运行100个固定种子，4个场景
共6000条结果；旧三路径20-seed结果仅作为诊断对照。

|场景|视觉延迟/漏检/角噪声|状态延迟/丢包/速度噪声|风标准差|
|---|---:|---:|---:|
|target_100ms|100 ms / 5% / 0.25 deg|100 ms / 5% / 0.25 m/s|0.25 m/s2|
|measured_p50|175 ms / 5% / 0.25 deg|150 ms / 5% / 0.25 m/s|0.25 m/s2|
|measured_p95|217 ms / 10% / 0.50 deg|200 ms / 10% / 0.50 m/s|0.50 m/s2|
|stress_225ms|225 ms / 20% / 1.00 deg|250 ms / 20% / 1.00 m/s|1.00 m/s2|

正式和诊断路径为：

- `candidate_velocity_hold_variable_thrust`：正式必选的离线候选；
- `current_hover_fixed_thrust`：旧实装形态，仅作差距诊断；
- `ideal_hover_speed_variable_thrust`：尚未实装的速度保持+可变推力参考；
- `diagnostic_established_speed_fixed_thrust`：预先精确建立拦截速度的诊断上界。

## 自动放行门限

只对初始位于视场内的工况计算主要成功率，避免把初始不可见目标混入控制能力统计：

- 命中率不低于95%；全程FOV命中率不低于90%；
- 目标过期失败率不高于1%；平均测量有效率和运动状态有效率均不低于90%；
- 最差最小距离不超过1.0 m；
- 平均倾角饱和和rate饱和均不高于10%。

任何必选路径任一门限失败，`release_passed`即为false。理想候选结果不会替代当前实装路径。

## 2026-08-30 历史20-seed诊断对照

下表依次为“初始可见命中率 / 全程FOV命中率 / 目标过期失败率”：

|场景|当前悬停fixed-thrust|理想悬停完整控制|已建立速度诊断|
|---|---:|---:|---:|
|target_100ms|8.3% / 6.7% / 28.7%|70.8% / 64.2% / 12.5%|85.4% / 76.7% / 7.1%|
|measured_p50|7.1% / 5.8% / 27.5%|65.8% / 55.4% / 18.3%|79.2% / 67.1% / 8.3%|
|measured_p95|7.9% / 3.8% / 40.8%|50.8% / 41.2% / 30.4%|61.7% / 52.1% / 17.5%|
|stress_225ms|4.6% / 1.3% / 94.2%|32.1% / 24.6% / 57.5%|22.9% / 17.1% / 65.4%|

当前100 ms场景命中率95% Wilson区间仅5.5%--12.5%；measured_p50为4.5%--11.0%。随机风会让
fixed-thrust路径偶然与目标相交，这些低比例事件不是受控制导能力。即使理想完整控制在100 ms下也只
达到70.8%，说明速度/推力控制之外还必须解决固定上视视场保持和目标过期。按场景名稳定派生seed后，
`target_100ms/trial 0`在完整矩阵和单场景运行中的45行逐字段一致。

## 2026-08-30 候选100-seed正式结果

下表依次给出初始可见的“命中率 / 全程FOV命中率 / 平均视觉有效率 / 平均运动状态有效率”。

|场景|候选结果|平均倾角/rate饱和|最差最小距离|
|---|---:|---:|---:|
|target_100ms|49.67% / 47.67% / 56.14% / 98.90%|23.73% / 1.40%|52.639 m|
|measured_p50|10.00% / 9.33% / 23.70% / 98.33%|8.16% / 0.70%|56.115 m|
|measured_p95|1.83% / 1.58% / 19.47% / 94.46%|4.05% / 0.56%|57.580 m|
|stress_225ms|0.17% / 0.00% / 28.81% / 84.34%|1.73% / 0.49%|58.381 m|

`release_passed=false`。100 ms场景主要受视觉出框/陈旧和20 deg倾角饱和限制；P50以后主要由
锁存ABORT主导。状态以5 Hz采样时，150 ms链路延迟加一个200 ms采样周期后已消耗0.35 s预算，
一次丢包会使样本年龄跨过0.5 s门限。压力场景的250 ms延迟使该余量更小。因此不能简单放宽
watchdog掩盖问题；正式控制状态源应提高更新率/连续性，并用第二UART或独立GNSS实测延迟和丢包。
视觉陈旧失败同时按`outcome_reason=target_stale`以及未命中的
`controller_final_reason=detection_stale/tracking_invalid`统计，避免把锁存ABORT误报为零陈旧失败。
四场景陈旧失败率依次为43.83%、15.33%、6.00%和14.67%；P50/P95比例下降是因为更多试验先被
速度状态ABORT截断，不代表视觉更稳健。

## 复现命令与证据

```bash
python3 tools/run_betaflight_intercept_monte_carlo.py \
  --config config/betaflight.intercept_eval.example.json \
  --evaluation-names candidate_velocity_hold_variable_thrust \
  --trials-per-case 100 --workers 8 \
  --output logs/betaflight_intercept_candidate_mc100_20260830.json \
  --csv logs/betaflight_intercept_candidate_mc100_20260830.csv
```

历史对照包为`logs/deployment_archives/betaflight_intercept_mc20_20260830.tar.gz`，SHA256为
`d744335cf3a5a47292ecc9339ebd169277135d1b2f1c8186faf65f0f5e2a9365`。正式候选包及最终测试数见
本次离线归档清单。正式报告SHA256为
`eec27fc77d32c8385fac6b2a53b3da9c9fcc06282cd04816a1baef00e7779d5f`，逐trial CSV SHA256为
`b2114e96d7f1f2ca32025324982c1b0173a8de77e4f48eaf0bdccc4b170340a2`；完整仓库335/335测试通过。

## 后续实施门

1. **状态源**：确定并时间同步速度、高度和垂向速度来源；没有状态源不得实现实机速度闭环。
2. **完整控制器**：实现有独立开关的三维`Vm*LOS`速度保持、可变推力和倾斜补偿，完整记录各级限幅。
3. **视场策略**：在控制器内实现上视居中、边缘保护和丢目标预测，并复测FOV失败工况。
4. **误差标定**：用动态LOG_ONLY数据拟合真实YOLO/ByteTrack角误差、漏检连续长度和速度误差，替换
   当前假设噪声。
5. **仿真放行**：至少100 seeds/case，所有必选门限通过后，才进入人工主控的非碰撞系留测试。
6. **实飞递进**：静止软目标、1 m/s横移、2--3 m/s直线、侧偏/高度差、最后S形；每级至少10次，
   只评估1--2 m近距通过，不直接以碰撞作为首次实飞成功标准。
