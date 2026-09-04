# Betaflight 监督飞行候选配对 MC100 评估（2026-09-03）

## 1. 结论

本轮在补入运行时关键动态后，对关闭和开启 FOV 优先级的控制器使用完全相同的随机种子进行
`18000` 条闭环仿真。选中的 `start_ratio=0.75, full_ratio=0.95` 在三个时延场景中达到：

|时延场景|基线命中|候选命中|基线 FOV 命中|候选 FOV 命中|
|---|---:|---:|---:|---:|
|软件结果年龄 P95，84.556 ms|95.46%|99.92%|87.77%|99.62%|
|主动飞行观测 P95，127.7 ms|94.73%|99.88%|85.46%|99.12%|
|保守物理预算，154.139 ms|94.62%|99.77%|84.77%|98.62%|

```text
paired_screening_passed           = true
release_passed                    = true
active_control_authorized         = false
```

项目正式发布目标已明确为：每个必需时延场景中，初始可见样本的命中率和全程视场可行命中率
均不低于 `80%`。单次命中仍按最近距离不大于 `1.0 m`判定；最差最近距离继续报告，但不再作为
要求全部样本命中的额外门槛。候选在三个场景中均通过该概率发布目标及其余数据质量、陈旧率和
饱和率门槛。该离线通过不替代真实飞行，也不单独生成主动控制批准文件。

## 2. 场景配置

|项|值|
|---|---|
|模型|NED 点质量、FRD 一阶角速度响应、上视相机矩形 FOV|
|资源/目标|1 架拦截机、1 个直线匀速目标|
|工况|15 个上视几何工况及其左右镜像，共 30 个|
|随机样本|每场景、每工况、每控制器 100 个；`base_seed=20260903`|
|控制更新|50 Hz 零阶保持，11 ms 命令延迟，40 ms 一阶响应常数|
|接管动态|角速度 smoothstep 0.8 s；油门线性交接 0.8 s|
|推力动态|600 us/s；1200/1275/1500 us 对应 0/1/2.37 g|
|相机半视场|水平 31.000689 deg，垂直 24.915405 deg，由内参和有效边界计算|
|约束|总加速度 7 m/s2；Roll/Pitch 60 deg/s、35 deg；油门不超过 1500 us|

运行配置采用严格绑定：

```text
config  config/betaflight.rk3588.velocity_png.flight_supervised.json
SHA256  4032482c47496c89d52f3cb09bfcef04b6c028de9012d10433d0aed4751865ff
```

工具会从该文件推导制导、控制频率、角速度、倾角、交接、油门、感知、运动学及 FOV 参数。
仿真配置重复声明这些字段但数值不一致，或文件哈希变化时，运行会直接失败。

## 3. 算法流程

本评估不是 D1-D7 多模块编排；实际参与链路为：延时/噪声 LOS 采样、生产 LOS Kalman
滤波器、延时 MSP 运动学、`velocity_establishing_png`、`accel_tilt_rate`、角速度保持及
实测负载因子推力映射。

FOV 优先级只在目标越过矩形半视场的 75% 后启用，并在 95% 达到全权重。它只消除“速度建立项
+ PNG 项”中与回中加速度相反的分量，不改变回中项，不增加总加速度、角速度、倾角或油门上限。
视场中心区域输出与原控制器逐值一致。

## 4. 筛选与指标

MC20 先对 9 组候选进行配对筛选。全部候选满足基本门槛，最终按预置评分选择 `0.75/0.95`：

|筛选项|结果|门槛|
|---|---:|---:|
|三个场景最低聚合命中率|99.4%|>=80%|
|三个场景最低聚合 FOV 命中率|98.7%|>=80%|
|外移工况 FOV 命中提升|55.42 个百分点|>=10 个百分点|
|中心/内移单工况最差命中退化|0 个百分点|<=2 个百分点|
|命令包线变化|无|必须无变化|

最终 MC100 中，候选外移工况聚合 FOV 命中率由 `35.83%` 提高到 `94.75%`，提升
`58.92` 个百分点；中心/内移工况最差命中退化仍为 `0`。三种时延下陈旧失败率从基线
`4.54%/5.27%/5.38%` 降到 `0.08%/0.12%/0.23%`。

保守时延下残余弱项仍是 `U06/U06M` 外移目标：命中为 `98%/96%`，全程 FOV 命中为
`89%/83%`。其余失败均为近失，候选没有以控制器 ABORT 结束的可见样本。三个场景最差最近
距离为`1.223/1.257/1.371 m`，作为尾部风险继续保留，不解释为100%命中保证。

## 5. 动态包线

|指标|关闭 FOV 优先级|候选 0.75/0.95|配置上限|
|---|---:|---:|---:|
|最大控制加速度|7.000 m/s2|7.000 m/s2|7 m/s2|
|最大角速度命令|60 deg/s|60 deg/s|60 deg/s|
|最大 Roll|27.42 deg|26.85 deg|35 deg|
|最大 Pitch|29.66 deg|34.89 deg|35 deg|
|最大油门|1394.80 us|1394.80 us|1500 us|
|最大负载因子|1.729 g|1.729 g|2.37 g|
|平均油门 slew 限制占比|3.69%|5.17%|600 us/s|
|候选 FOV 优先级平均启用占比|0%|11.83%|仅边缘区域|

## 6. 模型边界

已补入的模型包括 50 Hz 命令保持、角速度入场平滑、油门交接、PWM slew、实测负载因子曲线、
左右镜像航向和矩形 FOV。仍未建模 Betaflight PID/混控、电机和桨瞬态、电池压降、机架气动、
真实 YOLO 误检、ByteTrack ID 切换及目标主动机动。`2.37 g`是短时平台，不是持续推力证明。

CSV 只有逐条终值和包线统计，没有每步轨迹时序，因此本报告不生成伪造的轨迹曲线。

## 7. 复现与文件

```bash
python3 tools/run_betaflight_intercept_monte_carlo.py \
  --config config/betaflight.intercept_eval.flight_supervised_final_mc100_20260903.json \
  --trials-per-case 100 --workers 8 \
  --output doc/evidence/BETAFLIGHT_INTERCEPT_SUPERVISED_PAIRED_MC100_RELEASE_20260903.json \
  --csv logs/betaflight_intercept_supervised_20260903/betaflight_intercept_supervised_paired_mc100_release_20260903.csv
```

|文件|SHA256|
|---|---|
|MC20 筛选配置|`29c69d85871e8cb9881bf07843d33d3a2ec963be0e76cde228be85bcd35cde35`|
|MC20 筛选 JSON|`1447bb85f0ae91c6932d4d666256deab4fe265f3f7d05dc2e479698f3f7b9f6d`|
|MC20 筛选 CSV|`3c1adae780b0cc14ca138c234f59bb6e2eba088d9ad79e6da89b5b988b063664`|
|最终 MC100 配置|`c742ec4e5aacbb2e325601de8211c4c51005121d7f29036ac802c2b527c6086a`|
|最终 MC100 原始发布证据|`70f771259ff4f68af40af04979980ec1e2fc1d1cd638f19b4e5258f458adc0e8`|
|最终 MC100 CSV|`1830817016be0433a7499ce9df6548ae900be4399e8a9d3f1f66c6ce0c2b8b08`|

机器可读结论见
[`evidence/BETAFLIGHT_INTERCEPT_SUPERVISED_PAIRED_MC100_20260903.json`](evidence/BETAFLIGHT_INTERCEPT_SUPERVISED_PAIRED_MC100_20260903.json)。
批准工具直接校验的原始输出见
[`evidence/BETAFLIGHT_INTERCEPT_SUPERVISED_PAIRED_MC100_RELEASE_20260903.json`](evidence/BETAFLIGHT_INTERCEPT_SUPERVISED_PAIRED_MC100_RELEASE_20260903.json)。
