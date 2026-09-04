# Betaflight LOG00106 比例导引与飞控技术问答

## 0. 分析范围与证据边界

本文只解释最后一次外场主动接管架次，不把此前台架或其他飞行混入统计：

|证据|路径|用途|
|---|---|---|
|Orange Pi 原始控制日志|`logs/flight_active_supervised/FLIGHT_ACTIVE_05S_VIDEO_20260904_183721_20260904_183722.csv`|视觉、LOS、三项加速度、姿态外环、RC和MSP发布|
|拦截机原始 Blackbox|`logs/blackbox_import/LOG00106.BFL`|Betaflight setpoint、gyro、电机、电压、电流和比力|
|控制响应统计|`doc/evidence/BETAFLIGHT_LOG00106_CONTROL_RESPONSE_metrics.json`|同钟角速度跟踪和油门/比力窗口统计|
|靶机 ULog|`logs/target-log/10_35_19.ulg`|判断靶机在PNG发布窗口内是否移动|
|实际运行配置|`config/betaflight.rk3588.velocity_png.flight_supervised.json`|本架次增益、限幅、坐标系和油门策略|

原始输入哈希已记录在控制响应统计中。主机与 Blackbox 没有硬件共同时钟，因此跨设备曲线只用于
方向和幅值对照，不能用于精确测量控制延迟。本文引用的 `15 ms` 延迟来自 Blackbox 内部
setpoint 与 gyro 的同一时钟分析。

### 0.1 必须区分的物理量

|量|CSV字段|单位/坐标系|含义|
|---|---|---|---|
|LOS单位向量|`lambda_I_*`|无量纲，惯性NED|拦截机指向目标的视线方向|
|LOS导数|`lambda_dot_I_*`|`s^-1`，惯性NED|单位视线方向的变化率|
|速度参考|`intercept_velocity_reference_*`|`m/s`，惯性NED|拦截机期望建立的地速，不是靶机速度|
|速度建立、PNG、FOV加速度|`intercept_*_accel_*`|`m/s^2`，惯性NED|三种平动加速度需求，不是姿态角，也不是直接的过载g值|
|总加速度|`intercept_total_accel_*`、`g_eval_*`|`m/s^2`，惯性NED|三项经优先级和总限幅后的期望平动加速度|
|机体系诊断量|`g_eval_body_frd_*`|`m/s^2`，机体FRD|`R_IB^T g_eval`，主要用于日志诊断|
|需用比力|`command_thrust_required_specific_force_mps2`|`m/s^2`|抵消重力并实现垂直加速度、补偿当前倾角所需的推力/质量|
|需用载荷系数|`command_thrust_load_factor_raw_g`|`g`|需用比力除以 `9.80665 m/s^2`|
|姿态角|`roll_deg`、`pitch_deg`、`command_desired_*_angle_deg`|`deg`|机体方向；原始MSP抬头为负，转换到FRD后抬头为正|
|角速度|`sp_*_rate_deg_s`、Blackbox setpoint/gyro|`deg/s`|Betaflight Acro/Rate内环的输入和响应|

坐标约定为惯性系 NED，即 `N` 向北、`E` 向东、`D` 向下；机体系为 FRD，即 `x` 向机头、
`y` 向机体右侧、`z` 向下。`R_IB` 把 FRD 机体系向量旋转到 NED 惯性系，反变换为
`R_IB^T`。MSP姿态到该旋转矩阵的转换见 `vision_guidance/betaflight_msp.py:398-410`。

### 0.2 数值复算代表行

主要代表行取主机 CSV 的 `elapsed_s=39.449013`、单调时钟 `timestamp=286.262662`，约为
`2026-09-04 10:38:01.963 UTC`。该行 `safety_state=ACTIVE`、
`msp_publish_mode=algorithm`、`perception_new_result=1`，检测为 frame 1085、score 0.826158，
框为 `(315.471, 187.526, 369.216, 236.930)`。该行三项加速度、当前姿态、外环输出和油门字段
均有效，适合逐级复算。

需要注意，CSV 的控制器字段是当前主循环计算值，而 `throttle_handover_*` 和 `rc_sent_*` 是异步
MSP worker 的最后一次发布快照，两者可能相差一个约 `20 ms` 周期。因此，油门交接的精确算式
只在同一组 `throttle_handover_*` 字段之间核对。

## 1. MSP接管有效的含义

“MSP接管有效”需要分三层理解，不能只看一个开关：

1. `msp_override_active=1`：Betaflight 的永久模式ID 50处于激活状态，飞控允许
   `MSP_SET_RAW_RC` 覆盖相应通道。模式位解析见
   `vision_guidance/betaflight_runtime.py:362-371`。
2. `safety_state=ACTIVE`：授权、ARM、RC7、Acro、目标、运动学、遥测、看门狗、电压和ACK等
   软件门控同时满足。最终是否允许算法命令由
   `vision_guidance/flight_control.py:1360-1368` 决定。
3. `msp_publish_mode=algorithm`：异步worker实际选择了算法RC帧，而不是预填充或人工值直通。
   完整条件见 `vision_guidance/betaflight_runtime.py:1335-1345`。

本架次在 `elapsed_s=38.209581` 检测到RC7/MSP OVERRIDE生效，在 `38.242376` 首次记录到
`publish=algorithm`，算法实际发布 `1.670804 s`。worker发送了85个算法帧，发布率约
`49.683 Hz`，写错误为0、连续发送错误为0、ACK保持新鲜。这证明四通道MSP链路确实被飞控
接收并执行；配置中的 `override_channels_mask=15` 表示 Roll、Pitch、Throttle、Yaw 四个基本
通道均被覆盖，见 `config/betaflight.rk3588.velocity_png.flight_supervised.json:20-31,42-66`。

它不等于“导引一定正确”或“已经证明80%命中率”，也不等于切换飞行模式后已经恢复实时人工
摇杆。后者必须让RC7退出MSP OVERRIDE。

## 2. RK3588中对姿态角的PID策略

RK3588并没有实现完整的姿态角PID。它实现的是一个**姿态角比例外环**，Betaflight负责高速
**角速度Rate PID内环**：

```text
惯性NED总加速度
  -> 按当前yaw转入航向对齐坐标
  -> 计算期望Roll/Pitch角
  -> 角度误差乘Kp=4 s^-1
  -> 限幅到Roll/Pitch各60 deg/s
  -> 映射为MSP RAW RC
  -> Betaflight Acro/Rate PID跟踪角速度
```

RK3588外环公式位于 `vision_guidance/flight_control.py:1495-1556`：

```text
vertical_force = max(0.5, g - a_D)
roll_des  = atan2(a_yaw_y, vertical_force)
pitch_des = atan2(-a_yaw_x, vertical_force)
roll_rate  = clip(+1 * 4 * (roll_des-roll), +/-60)
pitch_rate = clip(-1 * 4 * (pitch_des-pitch), +/-60)
```

其中 `pitch_rate_sign=-1` 是本机Betaflight/MSP轴向实测绑定，不代表数学FRD俯仰方向反了；配置
见 `config/betaflight.rk3588.velocity_png.flight_supervised.json:293-352`。进入接管后的前
`0.8 s`，角速度还会从接管瞬间gyro平滑过渡到外环目标；之后再经过35度倾角包络，见
`vision_guidance/flight_control.py:322-450`。

代表行 `39.449013 s` 的复算如下：

```text
总加速度NED              = [-1.455241479, -0.737639406, -6.807213846] m/s^2
按yaw=333 deg旋转后       = [-0.96174837, -1.31790733, -6.807213846] m/s^2
vertical_force            = 9.80665-(-6.807213846) = 16.613863846 m/s^2
期望Roll/Pitch            = [-4.53553361, +3.31305795] deg
当前FRD Roll/Pitch        = [-1.30000000, -3.30000000] deg
姿态误差                  = [-3.23553361, +6.61305795] deg
外环Roll/Pitch rate       = [-12.94213444, -26.45223179] deg/s
CSV记录                   = [-12.942134,   -26.452232] deg/s
```

角度复算最大误差 `3.91e-7 deg`，角速度复算最大误差 `4.36e-7 deg/s`。该行已过0.8秒进入
平滑，所以 `pre_shape_sp_*` 与最终 `sp_*` 相同。这里没有RK3588的I项和D项；稳态角速度误差、
阻尼和电机混控由Betaflight Rate PID完成。

## 3. 俯仰角速度跟踪是否有问题

本架次的证据**不支持“俯仰角速度跟踪有问题”这个前提**。Blackbox同钟分析结果为：

|指标|Pitch结果|
|---|---:|
|Betaflight setpoint范围|`-34--+18 deg/s`|
|实际gyro范围|`-36--+19 deg/s`|
|最佳setpoint到gyro滞后|`15 ms`|
|相关系数|`0.994398`|
|拟合增益|`1.008540`|
|P95绝对误差|`3.508 deg/s`|

这表示方向正确、增益接近1、误差较小，没有持续反向或发散。Roll结果也相近：滞后 `15 ms`、
相关系数 `0.990996`、增益 `1.025540`、P95误差 `4.410 deg/s`。依据为
`doc/evidence/BETAFLIGHT_LOG00106_CONTROL_RESPONSE_metrics.json` 的
`same_clock_tracking`。

图上看似错位通常有四个原因：主机只有50 Hz而Blackbox约807 Hz；两个设备绝对时钟存在约
75--100 ms表观偏移；主机曲线按各自第一条算法指令归零；接管前0.8秒还有角速度进入平滑。
因此不能用两设备图线的水平距离否定Blackbox同钟的15 ms结果。

## 4. 视线角速度并不是朝零趋近，是否与目标移动有关

比例导引的目标是使惯性系LOS rate在闭合过程中趋近零，不要求每一个采样点单调下降。本实现
由卡尔曼滤波器估计 `lambda_dot_I`，并计算
`omega_los=lambda_I x lambda_dot_I`，见 `vision_guidance/los_filter.py:77-94`；PNG项使用
`a_png=N*Vm*lambda_dot`，见
`vision_guidance/betaflight_intercept_controller.py:210-213`。

本架次发布窗口内的LOS rate模长实际是总体下降、局部非单调：

|位置|`||lambda_dot||`|
|---|---:|
|算法开始 `38.242376 s`|`0.118375 s^-1`|
|最大值 `38.866200 s`|`0.127804 s^-1`|
|前半段中位数|`0.118206 s^-1`|
|后半段中位数|`0.073838 s^-1`|
|算法末行 `39.913180 s`|`0.007230 s^-1`|

所以“完全没有趋零”并不成立；更准确的说法是它先持平/上升，随后下降并发生分量反号。局部不
单调主要来自拦截机自身平移轨迹变化、约50--100 ms视觉/融合时延、LOS预测、速度建立项和总
加速度饱和共同作用。接管只有1.67秒，也不足以要求一个平滑的渐近曲线。

靶机移动不是PNG发布期LOS rate变化的主要原因。靶机ULog显示该窗口合速度P95仅
`0.056 m/s`，N/E/D端点位移为 `-0.043/+0.035/-0.038 m`，三轴人工杆量均为0。靶机飞手的
左滚输入始于主冲击前约0.27秒，即PNG停止发布之后约0.63秒；它会影响末端接触姿态，但不能
解释此前1.67秒发布窗口内的LOS rate曲线。

## 5. 参考东北下向速度是什么意思

`intercept_velocity_reference_n/e/d` 是控制器希望**拦截机**建立的惯性系地速：

```text
control_los = normalize(lambda + prediction_horizon * lambda_dot)
v_ref       = fixed_vm * control_los
v_ref_D     = clip(v_ref_D, -6, +6)
```

代码见 `vision_guidance/betaflight_intercept_controller.py:180-205`，本架次
`fixed_vm=10 m/s`、垂直参考限幅 `6 m/s`，见配置 `:227-255`。N/E/D正负含义为：

- `v_N>0` 向北，`v_N<0` 向南；
- `v_E>0` 向东，`v_E<0` 向西；
- `v_D>0` 下降，`v_D<0` 上升。

代表行的预测LOS为 `[0.01939020, 0.01704371, -0.99966671]`。乘10后原始参考为
`[0.193902, 0.170437, -9.996667] m/s`，D轴经限幅后CSV记录为
`[0.193902240, 0.170437317, -6.000000000] m/s`。它表示小幅向北、向东并以最多 `6 m/s`
向上建立速度，不是“目标机向东北下运动”。

## 6. 速度建立加速度北、东、下是什么意思

速度建立项是当前拦截机速度追踪上述速度参考所需的加速度：

```text
a_speed = clip_norm(1.2 * (v_ref-v_interceptor), 7 m/s^2)
```

公式见 `vision_guidance/betaflight_intercept_controller.py:206-209`。实际速度来自
MSP GPS地速/航向和气压垂速，经 `tau=0.25 s` 一阶滤波；转换见
`vision_guidance/betaflight_kinematics.py:145-165,236-257`。

代表行中：

```text
v_ref              = [ 0.193902240,  0.170437317, -6.000000000] m/s
v_filtered         = [-0.260261697,  0.261285330, -0.416987454] m/s
1.2*(v_ref-v)      = [ 0.544996724, -0.109017616, -6.699615056] m/s^2
CSV a_speed        = [ 0.544996724, -0.109017616, -6.699615056] m/s^2
复算向量误差       = 8.0e-10 m/s^2
```

含义是此刻应向北加速、轻微向西修正，并强烈向上加速。它是速度误差反馈，不是比例导引横向项。

## 7. PNG加速度N/E/D是否是需用过载，并代入本架次姿态复算

不是。`intercept_png_accel_n/e/d` 是PNG单项给出的惯性系平动加速度，单位 `m/s^2`：

```text
a_png = clip_norm(N * fixed_vm * lambda_dot, 7)
```

本架次 `N=3`、`fixed_vm=10 m/s`。代表行的
`lambda_dot=[-0.055377713,-0.054788638,-0.002330005] s^-1`，因此：

```text
a_png复算 = 30*lambda_dot
          = [-1.661331390, -1.643659140, -0.069900150] m/s^2
CSV       = [-1.661331393, -1.643659132, -0.069900142] m/s^2
向量误差  = 1.17e-8 m/s^2
模长      = 2.338060 m/s^2 = 0.238416 g
```

这仍不是整机需用过载。控制器先合成速度建立、PNG和FOV项得到总平动加速度，再由油门前馈加入
重力并补偿当前倾角。实现见 `vision_guidance/flight_control.py:1510-1555,1580-1616`。

代表行总加速度为 `[-1.455241479,-0.737639406,-6.807213846] m/s^2`，当前FRD姿态为
Roll `-1.3 deg`、Pitch `-3.3 deg`。代入实际实现：

```text
垂直需用比力 = g-a_D
              = 9.80665-(-6.807213846)
              = 16.613863846 m/s^2

倾角余弦      = cos(-1.3 deg)*cos(-3.3 deg)
              = 0.99808485

需用比力      = 16.613863846/0.99808485
              = 16.645742895 m/s^2

需用载荷系数  = 16.645742895/9.80665
              = 1.697393391 g
```

CSV记录为 `16.645743 m/s^2` 和 `1.697393 g`，打印精度内误差分别小于
`1.1e-7 m/s^2` 和 `3.9e-7 g`。如果直接取完整理想比力向量
`||a_cmd-[0,0,g]||`，结果为 `16.693781 m/s^2=1.702292 g`，与当前实现只差0.288%；差异来自
当前实现用实际倾角余弦补偿垂直推力，而不是直接用总需用比力向量模长。

## 8. FOV加速度按目标框位置计算是否合理

链路不是“像素偏差直接乘一个NED常数”，而是：框中心 -> 去畸变图像上的相机射线 ->
`R_BC`转FRD -> `R_IB`转NED -> LOS卡尔曼滤波和时延预测 -> 再转回当前FRD计算横向FOV项。
图像在检测前已去畸变，见 `examples/run_betaflight_log_only.py:630-659`；像素射线公式见
`vision_guidance/geometry.py:22-25`；视觉到惯性系见 `vision_guidance/fusion.py:71-79`。

本机外参为：OpenCV相机 `x` 向右、`y` 向下、`z` 向前；相机光轴映射到机体向上，即
`R_BC=[[0,1,0],[1,0,0],[0,0,-1]]`，配置见 `:133-182`。

代表行框中心为：

```text
(u,v)              = (342.3435, 212.2280) px
主点(cx,cy)         = (321.0279, 247.2573) px
像素偏差            = (+21.3156, -35.0293) px
相机归一化射线      = [+0.0400354, -0.0656134, +0.9970417]
直接映射FRD射线     = [-0.0656134, +0.0400354, -0.9970417]
```

即目标在图像中心右侧，对应机体右向分量为正；目标在中心上方，对应机体前向分量为负。考虑LOS
滤波和 `52.955 ms` 预测后，代码实际使用的当前机体系LOS为：

```text
control_los_body = [-0.04802155, +0.04663747, -0.99775692]
a_fov_body       = 16*[los_body_x, los_body_y, 0]
                 = [-0.76834472, +0.74619957, 0] m/s^2
||a_fov_body||   = 1.071059 < 7 m/s^2，未触发单项限幅
```

用该行姿态矩阵 `R_IB` 转回NED，复算得到
`[-0.34391635,+1.01249793,-0.06113022] m/s^2`；CSV记录为
`[-0.343915996,+1.012498263,-0.061130224] m/s^2`，向量误差仅
`4.87e-7 m/s^2`。所以公式、外参符号和量级在该代表行一致。单行复算能证明计算正确，不能单独
证明所有动态情况下FOV项都能把目标稳定留在画面中；这仍需用多架次图像误差闭环统计评价。

## 9. 最后的加速度是否等于速度建立项、PNG项和FOV项相加

概念上是三项合成，但严格顺序是：

1. 速度建立、PNG、FOV各自先做模长限幅；
2. 先算 `non_fov=a_speed+a_png`；
3. 目标接近FOV边缘时，按FOV优先权削弱 `non_fov` 中与FOV项反向的分量；
4. 加上 `a_fov`；
5. 对合计向量做 `7 m/s^2` 总模长限幅；
6. 最后应用可选LOS锥约束。

代码顺序见 `vision_guidance/betaflight_intercept_controller.py:206-251`。本配置启用了FOV优先，
但代表行的水平/垂直FOV占比分别约0.086和0.111，低于0.75启动阈值，因此权重为0；LOS锥半角
配置为0，也未启用约束。

代表行可直接核对：

```text
a_speed = [+0.544996724, -0.109017616, -6.699615056]
a_png   = [-1.661331393, -1.643659132, -0.069900142]
a_fov   = [-0.343915996, +1.012498263, -0.061130224]
----------------------------------------------------------------
原始和    = [-1.460250665, -0.740178485, -6.830645422] m/s^2
原始模长  = 7.024095178 m/s^2
总限幅比例= 7/7.024095178 = 0.996569640
限幅后    = [-1.455241479, -0.737639406, -6.807213846] m/s^2
CSV总项   = [-1.455241479, -0.737639406, -6.807213846] m/s^2
复算误差  = 1.53e-10 m/s^2
```

因此该行确实是三项相加后做总限幅；其他FOV优先权大于0的行不能简单用三项原值直接相加。

## 10. 速度建立项很早就饱和的含义

“速度建立项饱和”表示速度误差经 `1.2 s^-1` 增益后要求的加速度模长超过
`7 m/s^2`，控制器保留方向并把模长截到7；它不表示电机、角速度或PNG项已经饱和。

算法首行 `38.242376 s`：

```text
v_ref              = [-0.227837919, +0.650494674, -6.000000000] m/s
v_filtered         = [-0.774777852, +0.185043536, +0.143753171] m/s
未限幅a_speed      = 1.2*(v_ref-v)
                   = [+0.656327919, +0.558541365, -7.372503806] m/s^2
未限幅模长         = 7.422705 m/s^2
限幅后a_speed      = [+0.618951654, +0.526733804, -6.952657790] m/s^2
```

原因很直接：目标在上方，`fixed_vm=10 m/s` 经过D轴限幅后仍要求 `-6 m/s` 上升，而接管时拦截机
约为 `v_D=+0.144 m/s`，仅垂直速度误差就产生 `-7.37 m/s^2` 的原始需求。

速度项从 `38.242376` 到 `39.328334 s` 连续饱和，共55/84行，即 `65.48%`。到拦截机建立明显
上升速度后才退出饱和。这个现象符合当前激进参数，不是数值错误；但它说明速度建立项长期占用
绝大部分加速度权限，PNG横向修正只能在总限幅中竞争。若要降低这一问题，应优先考虑速度参考
渐进建立、按距离/框面积调度 `fixed_vm`，或给速度建立与PNG/FOV做明确的加速度权限分配，而不
是提高总上限。

## 11. 总加速度一开始就饱和的含义

总加速度饱和表示三项合成后的模长超过 `7 m/s^2`，随后整体同比缩放。算法首行：

```text
a_speed = [+0.618951654, +0.526733804, -6.952657790]
a_png   = [+3.413421819, -0.962601586, -0.182289827]
a_fov   = [-1.131220773, +1.369140310, -0.082929551]
原始和  = [+2.901152700, +0.933272528, -7.217877168] m/s^2
原始模长= 7.834885794 m/s^2
缩放比例= 0.893439954
最终总项= [+2.592005734, +0.833822964, -6.448739841] m/s^2
```

复算与CSV在 `1e-9 m/s^2` 内一致。总项从 `38.242376` 到 `39.449013 s` 连续饱和，共61/84行，
即 `72.62%`；PNG项和FOV项在整个发布窗口均未单项饱和。

这意味着最初约1.21秒内控制器主要受总加速度预算约束，需求方向仍保留，但速度建立、PNG和FOV
的绝对幅值不能同时全部实现。它能解释LOS rate为何不必单调趋零，也说明当前参数更像“快速建立
闭合速度”的命中配置，而不是留有较大横向控制余量的非碰撞跟随配置。

## 12. 设定油门和实际油门的对比

需要区分四个层次：模型载荷需求、映射后的目标RC油门、实际发送RC油门、飞控/动力响应。

|层次|本架次PNG发布窗口结果|解释|
|---|---:|---|
|模型需用载荷|`1.548--1.736 g`，P50 `1.650 g`|由总D向加速度和当前倾角计算|
|模型目标油门|`1367--1396 us`，P50 `1382 us`|未经过0.8秒交接的目标|
|实际发送油门|`1305--1396 us`，P50 `1375 us`|`rc_sent_ch3`，含0.8秒交接|
|Betaflight内部throttle|`1266--1364`|Blackbox内部控制量，不是物理PWM|
|四电机raw|`573--936`|含总推力和姿态差速，不是单一油门通道|
|实测比力模长|P50/P95/max `1.298/1.450/1.468 g`|碰撞前Blackbox IMU结果|

发送油门与Betaflight内部throttle波形相关系数约 `0.9995`，无油门下跳、无超过1500 us、无碰撞前
电机饱和，说明MSP发送和飞控接收链正确。交接完成后，实测比力/模型需用比力中位数为
`0.809`，即实际比力约低19%。因此问题在当前两点线性推力模型对本次低电压、快速爬升工况偏
乐观，而不是“设定油门没有送到飞控”。具体统计来自
`doc/evidence/BETAFLIGHT_LOG00106_CONTROL_RESPONSE_metrics.json`。

还应避免把 `rc_sent_ch3` 称为“电机实际PWM”：它是发送给Betaflight的油门杆位。真实执行证据
要联合Blackbox内部throttle、四电机raw、电流和IMU比力判断。

## 13. 油门交接原始目标、交接限幅目标的含义

相关字段按处理顺序解释如下：

|字段|含义|
|---|---|
|`throttle_handover_source_us`|拨入接管瞬间冻结的物理油门，本架次始终为 `1303 us`|
|`throttle_handover_requested_target_us`|姿态/推力模型经RC映射和绝对油门范围处理后，算法原本请求的目标|
|`throttle_handover_lower_limit_us` / `upper_limit_us`|相对切入前油门的动态上下界；只在 `throttle_relative_limit_us>0` 时存在|
|`throttle_handover_target_us`|requested target经过上述相对上下界后的交接目标|
|`throttle_handover_target_limited`|requested target是否被相对上下界改动|
|`throttle_handover_alpha`|交接进度，`0`为完全使用source，`1`为完全使用target|
|`throttle_handover_output_us`|`round((1-alpha)*source+alpha*target)`|
|`throttle_slew_output_us`|交接输出再经过运行时 `600 us/s` 变化率限制后的值|
|`rc_sent_ch3`|异步worker最后实际发送给Betaflight的油门通道|

实现见 `vision_guidance/betaflight_runtime.py:590-645,1354-1405`。本配置
`throttle_relative_limit_us=0`，所以相对限幅关闭；绝对映射范围仍为 `1200--1500 us`，见配置
`:60-66,280-290`。本架次84个算法发布记录中：

```text
target_limited       始终为0
lower/upper_limit    始终为空
requested target     与limited target始终相等
运行时slew_limited   始终为0
```

首个算法发布快照可精确核对交接公式：

```text
source              = 1303 us
requested target    = 1384 us
limited target      = 1384 us
alpha               = 0.020289
round((1-a)*1303+a*1384) = 1305 us
handover output     = 1305 us
slew output         = 1305 us
rc_sent_ch3         = 1305 us
```

到 `elapsed_s=39.047101` 时 `alpha=1`，`target=1384 us`、`output=1384 us`，交接完成。代表行
`39.449013` 的当前控制器计算对应 `rc_target_ch3=1390 us`，worker快照为上一发布周期的
`requested/target/output/rc_sent_ch3=1391 us`；这一微小差异是异步记录相位，不是限幅或传输
误差。

## 综合判断

- MSP四通道接管、角速度映射、Betaflight Rate PID跟踪和油门交接在最后架次均按代码工作。
- “Pitch跟踪有问题”和“LOS rate完全不趋零”均不符合最后架次数据；前者被同钟Blackbox结果
  直接反驳，后者应改述为短窗口内非单调但总体下降。
- `a_png`、`a_speed`、`a_fov` 和 `g_eval` 都是NED平动加速度，不能直接当作过载g或姿态角。
- 代表行的PNG、FOV、总限幅、姿态比例外环和需用载荷复算均与CSV闭合，未发现公式实现或坐标
  符号错误。
- 当前主要控制问题是速度建立项和总加速度过早、长时间饱和，以及推力模型对本次瞬态可用比力
  高估约19%。优先改进方向应是速度参考/加速度权限调度和电压相关非线性推力标定，而不是反转
  Pitch轴或提高总加速度上限。
