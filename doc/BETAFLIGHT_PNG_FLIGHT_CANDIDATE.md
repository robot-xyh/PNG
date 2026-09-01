# Betaflight PNG 飞行候选方案

## 状态与边界

本方案用于把当前固定 Vm 比例导引从无桨验证推进到受限飞行标定。它不是飞行批准。机器可读的
待确认项位于 `config/betaflight.rk3588.flight_candidate.parameters.json`，其中关键值保持
`null`，且 `runnable=false`、`control_authorized=false`。无桨批准文件只能批准
`noprop_bench`，不得复用于装桨或自由飞行。

## 实际算法链

真实入口保持当前 RKNN YOLO 修改模型和完整 ByteTrack：

```text
640x512 去畸变图像
  -> RKNN YOLO DFL/NMS 全候选
  -> ByteTrack 高/低分两阶段关联和单目标锁定
  -> bbox 中心反投影为 camera LOS
  -> R_BC 与曝光时刻 R_IB 旋转到惯性 NED
  -> LOS Kalman filter: lambda_I, lambda_dot_I
  -> omega_LOS = lambda_I x lambda_dot_I
  -> a_png_I = clip_norm(N * Vm * (omega_LOS x lambda_I), a_max)
```

旧 `direct_rate_matrix` 把 `a_png` 的数值直接乘矩阵得到 `deg/s`，矩阵同时承担轴交换、符号和
量纲增益，不能表达飞行器如何由加速度形成倾角。新 `accel_tilt_rate` 保留 NED/FRD 坐标语义，
先在当前 yaw 平面求期望倾角，再由外环姿态 P 产生 Betaflight rate setpoint：

```text
a_yaw = Rz(yaw)^T * a_png_I
f_z   = max(f_min, g - a_yaw,z)
phi_d   = clamp(atan2(a_yaw,y,  f_z), +/-phi_max)
theta_d = clamp(atan2(-a_yaw,x, f_z), +/-theta_max)
p_cmd = clamp(s_roll  * Kp_roll  * (phi_d   - phi),   +/-p_max)
q_cmd = clamp(s_pitch * Kp_pitch * (theta_d - theta), +/-q_max)
```

`phi/p`、`theta/q` 使用 FRD/NED 右手系。MSP 原始 pitch 抬头为负，构造 `R_IB` 时只转换一次；
禁止在矩阵或遥控器通道上重复翻转。`s_roll`、`s_pitch` 必须通过物理符号试验确认。

## 离线权威性结果

工具 `tools/evaluate_betaflight_png_authority.py` 直接读取实测归档，逐行重算导引并扫描
`Vm/a_max/Kp/tilt/rate`。以下使用 `N=3`、`Kp=3 s^-1`、倾角上限20 deg、rate上限60 deg/s，
每行的 `a_max` 与 `Vm` 数值相同；数据是零 roll/pitch 假设下的纯导引最大轴绝对 rate：

|Vm / a_max|横移 P50 / P95 / max|纵移 P50 / P95 / max|
|---|---|---|
|1 m/s / 1 m/s2|2.57 / 7.98 / 17.38 deg/s|2.76 / 7.13 / 16.73 deg/s|
|3 m/s / 3 m/s2|7.62 / 24.07 / 51.07 deg/s|8.27 / 21.09 / 47.53 deg/s|
|5 m/s / 5 m/s2|12.54 / 39.82 / 60.00 deg/s|13.86 / 34.49 / 60.00 deg/s|
|10 m/s / 10 m/s2|25.37 / 60.00 / 60.00 deg/s|27.55 / 60.00 / 60.00 deg/s|

横移983行、纵移784行来自同一真实相机手持目标日志。结果证明新映射能把已记录 LOS 动态变成
非零、随 `N*Vm` 增长的姿态/rate 指令；它不证明闭环稳定或拦截成功。手持运动不是实际相对
运动，记录机体固定不响应命令，`Vm`也不是视觉测速结果。尤其 `Vm=10` 已频繁到达候选 rate/
tilt上限，不能据此直接选作首飞参数。

复算命令示例：

```bash
python3 tools/evaluate_betaflight_png_authority.py \
  --input logs/deployment_archives/fixed_vm_online_20260829_104631.tar.gz \
  --archive-member fixed_vm_online_20260829_104631/real_camera_handheld_repeat/fixed_vm_real_handheld_camera_20260829_repeat_20260829_144654.csv \
  --start-s 252.727 --end-s 304.026 \
  --output logs/authority_real_horizontal.json
```

## matrix15 离线闭环结果

新增 `vision_guidance/betaflight_png_sim.py` 和
`tools/simulate_betaflight_png_matrix15.py`，在真值 LOS 下把生产 `accel_tilt_rate` 接入一阶姿态和
点质量平动闭环。结果确认当前固定油门链从悬停为 `0/15`；只有预先精确建立三维 `Vm*lambda` 速度
时才达到 `15/15` 真值命中，且当前20 deg/60 deg/s包线仅 `10/15` 全程保持在120 deg上视视场。
加入上视居中、LOS三维速度保持和理想可变推力后，从悬停为 `15/15` 真值命中，但当前包线全程FOV
命中仍只有 `9/15`，平均倾角饱和47.39%。详细模型、逐包线结果和复现命令见
`doc/BETAFLIGHT_PNG_OFFLINE_MATRIX15.md`。

这证明 PNG 转 rate 的外环方向成立，也证明它不是完整飞行控制器。Betaflight 实现仍缺速度/高度
状态输入、沿 LOS 保速、垂向加速度到油门的实机标定和视场保持；不得用真值 `15/15` 代替真实
YOLO/ByteTrack命中结论。

## 上电前必须确定

- `Vm` 的物理依据：计划的拦截机速度或保守闭合速度，不得把目标手持速度当作 `Vm`。
- `a_max`、最大倾角、最大 rate：由质量、推重比、Betaflight rate/PID profile 和试飞包线确定。
- roll/pitch 两轴的 rate 符号、外环 `Kp`，以及 Betaflight rate 反函数或实测 LUT。
- 悬停 PWM、最小可控 PWM 和允许油门范围。当前1078 us只是无桨台架值，不能用于飞行。
- RC 所有权。首轮候选应由专家评审是否仅覆盖 roll/pitch（mask 3），让飞手保留 throttle/yaw；
  当前无桨 mask 15 不能自动继承。
- 进程崩溃、接收机断链、MSP断流、低电压和人工接管的实机结果。当前 armed 进程崩溃试验未完成。

## 分阶段放行

1. 使用 schema 15 `LOG_ONLY` 在真实运动数据上确认目标姿态、当前姿态、姿态误差和 rate 均有限，
   两轴符号与机体动作一致。
2. 在拆桨条件下只验证新映射符号和限幅，不复用旧 direct-matrix 结论，也不延长主动接管时间。
3. 专家确认 RC mask、手动油门/yaw、hover 标定和外环初值后，建立独立 flight approval；无该文件
   runner 必须保持 `DISABLED/snapshot_not_approved`。
4. 系留悬停先验证姿态阶跃和人工退出，再做低速、非碰撞移动目标；记录主机 CSV 和 Blackbox。
5. 只有闭环姿态、延迟、饱和率、目标连续性和退出行为全部可解释后，才评估近距通过或拦截指标。
