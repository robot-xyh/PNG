# LOS 6D + TTC 比例导引原理说明

本文解释当前 AirSim 视觉拦截程序中使用的 `LOS 6D + TTC` 改进比例导引方法，并说明它与常用经典比例导引的区别。

这里的“当前方法”主要对应云台视觉 PNG 和捷联视觉 PNG 两类程序：

- `examples/run_airsim_gimbal_vision_png.py`
- `examples/run_airsim_strapdown_vision_png.py`
- `vision_guidance/los_filter.py`
- `vision_guidance/ttc.py`
- `vision_guidance/png_eval.py`

当前视觉程序的核心约束是：算法内部不读取入侵无人机真实位置、真实速度或真实距离，只使用 AirSim `detect` 返回的检测框。真实位置只用于初始对准、离线评价、绘图和碰撞结果分析。

## 1. 经典比例导引的基本形式

常用比例导引通常假设可以获得目标和拦截器之间的相对位置、相对速度，或者至少能够从雷达/融合导航中得到距离、闭合速度和视线角速度。

设：

```text
p_t : 目标位置
p_m : 拦截器位置
v_t : 目标速度
v_m : 拦截器速度
r   = p_t - p_m              相对位置向量
R   = ||r||                  目标距离
lambda = r / R               视线单位向量
v_rel = v_t - v_m            相对速度
Vc = -dot(lambda, v_rel)     闭合速度，接近时为正
omega_LOS = r x v_rel / R^2  视线角速度向量
N : 导引比，常见取值约 3-5
```

经典三维比例导引常写作：

```text
a_cmd = N * Vc * (omega_LOS x lambda)
```

由于：

```text
lambda_dot = omega_LOS x lambda
```

所以也可以写成：

```text
a_cmd = N * Vc * lambda_dot
```

它的物理含义是：如果视线方向还在转动，就说明当前速度方向还没有建立稳定碰撞航线；拦截器应施加一个与视线垂直的法向加速度，使视线角速度趋近于零。当 `lambda_dot -> 0` 且距离仍在减小时，说明拦截器和目标大概率处于碰撞航线。

经典 PNG 的关键特点是：

- 需要相对位置 `r`。
- 需要相对速度 `v_rel` 或闭合速度 `Vc`。
- 输出通常是法向加速度指令 `a_cmd`。
- 导引比 `N` 是明确的物理比例系数。
- 适合雷达、真值仿真、双目/激光测距或成熟传感器融合系统。

### 1.1 使用拦截器速度 `V_m` 的比例导引形式

工程中还常见一种写法，不显式使用闭合速度 `Vc`，而使用拦截器自身速度大小 `V_m`：

```text
V_m = ||v_m||
a_cmd = N * V_m * lambda_dot
```

或者写成角速度形式：

```text
a_cmd = N * V_m * (omega_LOS x lambda)
```

其中：

- `V_m` 是拦截器自身速度大小。
- `lambda_dot` 是视线单位向量变化率。
- `omega_LOS` 是视线角速度。
- `N` 仍然是导引比。

这种形式的直观含义是：拦截器飞得越快，同样的视线角速度需要越大的侧向加速度才能把速度方向转过去。它把 `V_m * lambda_dot` 看作“当前速度方向应跟随视线旋转所需的法向速度变化率”。

与 `Vc` 形式相比，`V_m` 形式少用了目标径向速度信息：

```text
Vc = -dot(lambda, v_t - v_m)
V_m = ||v_m||
```

两者在某些几何下接近，但不是同一个量：

- 迎头或强闭合交会：`Vc` 可能大于 `V_m`，因为目标也在向拦截器接近。
- 尾追场景：`Vc` 可能远小于 `V_m`，因为目标也在同向逃离。
- 侧向交会：`Vc` 取决于双方速度在 LOS 方向上的投影，而 `V_m` 只看拦截器自身速度。

因此 `V_m` 型比例导引可以看作“只知道自身速度和目标角度变化时的工程近似”。它的优点是传感器要求低，缺点是不同交会角下等效导引强度会变化。

如果把经典 PNG 写成：

```text
a_cmd = N * Vc * lambda_dot
```

而 `V_m` 型写成：

```text
a_cmd = N * V_m * lambda_dot
```

则两者的等效导引比关系约为：

```text
N_eff = N * V_m / Vc
```

当 `V_m / Vc` 偏离 1 时，即使代码里 `N` 不变，实际制导强度也会改变：

- `V_m / Vc < 1`：等效导引偏弱，可能末端修正不足。
- `V_m / Vc > 1`：等效导引偏强，可能过度打舵或引起振荡。

这也是纯视觉单目方案不能只固定一个 `V_m` 或固定一个 `Vc` 就适应所有交会角的原因。

## 2. 视觉方案为什么不能直接照搬经典 PNG

当前视觉方案只有单目检测框。检测框能可靠提供的是角度信息，而不是距离。

检测框中心 `(u, v)` 可以通过相机内参转换为一条相机系视线：

```text
bbox center -> camera ray -> LOS direction
```

但单目检测框面积或宽高不能稳定反推绝对距离，原因包括：

- 目标真实尺寸未知。
- 目标姿态变化会改变投影面积。
- 检测框边缘会因光照、姿态、遮挡和算法输出而抖动。
- 末端目标会填满视场，检测框被图像边界裁切。

如果把检测框宽高强行放进 8D EKF，同时估计：

```text
lambda, lambda_dot, r, r_dot
```

就会把单目距离噪声通过协方差耦合污染 `lambda_dot`。而 `lambda_dot` 正是比例导引最关键的变量。因此当前方案故意放弃单目绝对测距，把角度通道和尺度通道解耦。

## 3. 当前 LOS 6D 角度通道

### 3.1 状态定义

6D LOS 滤波器只估计视线方向和视线方向变化率：

```text
x = [
  lambda_x,
  lambda_y,
  lambda_z,
  lambda_dot_x,
  lambda_dot_y,
  lambda_dot_z
]
```

其中：

- `lambda` 是惯性系视线单位向量。
- `lambda_dot` 是惯性系视线单位向量的变化率。
- `omega_LOS = lambda x lambda_dot` 是视线角速度向量。

注意：状态里没有距离 `R`，也没有距离变化率 `R_dot`。

### 3.2 从检测框到惯性系 LOS

每帧检测框中心先转换成相机系视线：

```text
los_C = camera_ray_from_pixel(u, v, intrinsics)
```

再经过相机外参和机体姿态转到惯性系：

```text
lambda_I_measured = R_IB(t_exposure) * R_BC * los_C
```

含义：

- `R_BC`：相机系到机体系的旋转。
- `R_IB(t_exposure)`：图像曝光时刻机体系到惯性系的旋转。
- `t_exposure`：检测框对应图像帧的时间戳。

这里必须使用曝光时刻的姿态，而不是当前时刻姿态。高速机动时，几十毫秒的姿态错位会把相机 LOS 转错方向，制造虚假的 `lambda_dot`，也就是伪视线机动。

### 3.3 滤波预测与更新

滤波器采用常速度模型：

```text
lambda(k+1)     ~= lambda(k) + lambda_dot(k) * dt
lambda_dot(k+1) ~= lambda_dot(k)
```

观测量只有 `lambda_I_measured`：

```text
z = lambda_I_measured
H = [I3, 0]
```

更新后施加两个几何约束：

```text
lambda = normalize(lambda)
lambda_dot = project_perpendicular(lambda_dot, lambda)
```

第二个约束很重要。因为 `lambda` 是单位向量，它的变化率必须垂直于自身：

```text
dot(lambda, lambda_dot) = 0
```

否则滤波器会产生非物理的径向分量。

### 3.4 LOS 质量门控

每次更新都会计算创新量：

```text
innovation = ||lambda_measured - lambda_predicted||
```

如果创新量超过阈值，当前帧被判为无效：

```text
reason = los_innovation_reject
```

这类拒绝通常来自：

- 检测框跳变。
- 目标被裁切后中心点偏离真实质心。
- 姿态时间戳错位。
- 云台接近限位或捷联机体姿态快速变化。
- 目标出视场后重新检测到错误目标。

LOS 通道输出：

```text
lambda_I
lambda_dot_I
omega_LOS = lambda_I x lambda_dot_I
los_quality
los_valid
reject_reason
```

## 4. 当前 TTC 尺度通道

### 4.1 基本几何关系

单目相机不能稳定估计绝对距离，但目标接近时，投影尺度会增大。

若目标真实外形和姿态近似不变，图像中的线尺度 `s` 与距离 `R` 近似满足：

```text
s proportional 1 / R
```

检测框面积 `A` 近似满足：

```text
A proportional 1 / R^2
```

对面积求导：

```text
A_dot / A ~= -2 * R_dot / R
```

接近时闭合速度：

```text
Vc = -R_dot
```

所以碰撞时间：

```text
TTC = R / Vc ~= 2 * A / A_dot
```

这就是 Scale Expansion / TTC 通道的核心。它不需要知道目标真实尺寸，也不直接输出距离，只估计“还剩多少接近时间”的趋势。

### 4.2 当前实现

当前代码对检测框面积做低通滤波：

```text
A_filt(k) = alpha * A(k) + (1 - alpha) * A_filt(k-1)
```

然后在一个短窗口内拟合面积变化率：

```text
A_dot_filt = slope(A_filt over time)
```

最后计算：

```text
TTC = 2 * A_filt / A_dot_filt
```

TTC 通道会拒绝以下情况：

- 面积太小：`bbox_area_too_small`
- 面积没有膨胀：`area_not_expanding`
- TTC 超出范围：`ttc_out_of_range`
- 检测框面积突变：`bbox_area_jump`
- 检测框被边界裁切：`bbox_top_clipped`、`bbox_bottom_clipped`、`bbox_left_clipped`、`bbox_right_clipped`

TTC 输出：

```text
ttc
ttc_quality
area_filtered
area_dot_filtered
ttc_valid
reject_reason
```

## 5. LOS 6D + TTC 的融合导引

当前方案不是“只用 6D LOS”，也不是“只用 TTC”，而是分两条通道：

```text
bbox center -> 6D LOS -> lambda_I, lambda_dot_I, omega_LOS
bbox area   -> TTC    -> K(TTC), ttc_quality
```

融合逻辑：

1. LOS 有效，TTC 有效：进入 `ttc_png`。
2. LOS 有效，TTC 无效但原因可接受：进入 `los_fallback`。
3. LOS 无效：不输出普通视觉 PNG，进入 coast、LossHold 或 BlindPush。
4. 检测框裁切、丢检、云台限位或末端面积过大：进入末端状态机。

### 5.1 TTC 动态增益

当前基础库中的增益调度为：

```text
if TTC <= 1s: K = max_gain
if TTC >= 6s: K = min_gain
else: K 在 min_gain 和 max_gain 之间平滑过渡
```

默认参数：

```text
min_gain = 0.5
max_gain = 5.0
ttc_fast_s = 1.0
ttc_slow_s = 6.0
```

含义：

- TTC 大：目标还远或接近较慢，降低修正，避免远距离过度打舵。
- TTC 小：目标即将接近，增大修正，压制末端残差。

这个 `K(TTC)` 不是经典 PNG 中的固定导引比 `N`，而是一个工程化动态增益。

### 5.2 当前闭环程序中的导引量

在当前 AirSim 云台/捷联程序里，实际用于速度指令修正的是：

```text
g_eval = K(TTC) * omega_LOS
correction = K(TTC) * (omega_LOS x lambda_I)
```

由于：

```text
omega_LOS x lambda_I = lambda_dot_I
```

所以速度修正方向等价于沿 `lambda_dot_I` 的法向方向修正。

完整速度指令近似为：

```text
base = speed_cap * lambda_I
v_cmd = base + clamp_lateral_vertical(correction)
```

其中：

- `base` 让拦截机沿当前视线方向飞向目标。
- `correction` 根据视线角速度修正侧向和垂直方向。
- `speed_cap` 由入侵机速度和速度系数决定。
- 侧向/垂直修正会被限幅。

也就是说，当前视觉程序输出的是速度指令，不是经典 PNG 的加速度指令。它更准确地说是：

```text
带 TTC 动态增益的 LOS-rate 速度制导
```

或者：

```text
单目视觉改进 PNG / PNG-like guidance
```

它和 `V_m` 型 PNG 的关系更近一些：二者都不直接依赖目标真实速度，也不直接使用精确 `Vc`。区别在于，`V_m` 型 PNG 仍然输出加速度：

```text
a_cmd = N * V_m * lambda_dot
```

而当前 AirSim 视觉程序输出速度修正：

```text
v_cmd = speed_cap * lambda_I + clamp(K(TTC) * lambda_dot_I)
```

其中 `speed_cap` 类似给出了“拦截器可用速度尺度”，`K(TTC)` 类似承担动态导引强度调节。但它们没有严格合成为 `N * V_m`，因此不能把当前参数直接解释成传统导引比。

### 5.3 基础库中的评估量

`vision_guidance/png_eval.py` 中的 `GuidanceEvaluator` 输出：

```text
g_eval = K(TTC) * lambda_dot_I
```

这是一个日志、回放和监督评估量。AirSim 闭环程序为了构造三维速度修正，使用了等价的角速度表达：

```text
K(TTC) * (omega_LOS x lambda_I)
```

两者方向一致，但需要注意单位和控制接口不同：它们都不是严格意义上的经典 PNG 加速度 `N * Vc * lambda_dot`。

## 6. 与常用比例导引的主要差异

| 对比项 | 经典 `Vc` 型 PNG | `V_m` 型 PNG | 当前 LOS 6D + TTC 视觉 PNG |
|---|---|---|---|
| 输入 | 相对位置、相对速度、距离、闭合速度 | 目标角度/LOS 角速度、自身速度 | 检测框中心、检测框面积、相机姿态 |
| 距离 | 显式使用 `R` | 可不显式使用 `R` | 不估计绝对距离 |
| 速度尺度 | 闭合速度 `Vc` | 拦截器速度 `V_m` | `speed_cap` 和 `K(TTC)` |
| LOS 来源 | 由相对位置直接计算，或雷达角度测量 | 角度传感器、雷达或视觉测角 | bbox center -> 相机射线 -> 姿态去旋转 |
| LOS 角速度 | `r x v_rel / R^2` 或角度微分 | 角度微分或滤波 LOS-rate | 6D KF 输出 `lambda_dot` 和 `omega_LOS` |
| 导引公式 | `a_cmd = N * Vc * lambda_dot` | `a_cmd = N * V_m * lambda_dot` | `v_cmd = speed_cap * lambda + clamp(K(TTC) * lambda_dot)` |
| 导引增益 | `N * Vc`，物理意义明确 | `N * V_m`，是缺少 `Vc` 时的近似 | `K(TTC)` 动态增益，不等价于 `N` |
| 输出 | 通常为法向加速度 `a_cmd` | 通常为法向加速度 `a_cmd` | 当前 AirSim 程序输出速度指令 `v_cmd` |
| 对交会角敏感性 | 较低，因为 `Vc` 随几何变化 | 较高，因为 `V_m` 不含目标径向运动 | 较高，靠 TTC 和门控缓解 |
| 单目尺度噪声 | 不涉及，通常有距离传感器 | 可不使用距离，但仍缺接近速度 | 面积只进入 TTC，不进入 LOS 主状态 |
| 末端处理 | 依赖持续测量或惯导/雷达 | 依赖持续角度测量和速度尺度近似 | 需要 BlindPush、coast、像面 KF 外推 |
| 工程风险 | 传感器融合、加速度饱和、目标机动 | 等效导引比随 `V_m/Vc` 漂移 | bbox 抖动、裁切、丢检、姿态时间对齐、TTC 噪声 |

最核心区别是：经典 `Vc` 型 PNG 的加速度量级由 `N * Vc` 决定；`V_m` 型 PNG 用 `N * V_m` 近似这个速度尺度；当前视觉 PNG 没有可靠的 `Vc`，也没有直接输出 `N * V_m * lambda_dot` 加速度，因此用 `TTC` 调节增益，用速度指令和限幅来实现近似的碰撞航线建立。

从工程谱系看：

```text
真值/雷达 PNG:
  a_cmd = N * Vc * lambda_dot

只测角 + 自身速度 PNG:
  a_cmd = N * V_m * lambda_dot

当前单目视觉 LOS+TTC:
  lambda_dot 来自 6D LOS
  接近趋势来自 TTC
  v_cmd = speed_cap * lambda + clamp(K(TTC) * lambda_dot)
```

因此当前方案不是严格的 `Vc` 型 PNG，也不是严格的 `V_m` 型 PNG，而是“用 TTC 补偿 `V_m` 型纯测角导引缺少接近速度信息”的视觉工程版本。

## 7. 当前方案的优点

### 7.1 避免单目测距污染 LOS

当前方案不把 bbox 面积和距离状态强耦合。面积噪声不会通过滤波协方差污染 `lambda_dot`，因此角度通道更稳定。

### 7.2 单目相机能力利用更合理

单目相机最可靠的是角度，而不是距离。6D LOS 把相机当作测角传感器使用，符合传感器本质。

### 7.3 TTC 不依赖目标真实尺寸

TTC 用面积相对膨胀率，不直接依赖目标真实宽度。只要目标外形和姿态在短时间内相对稳定，面积膨胀趋势就能反映接近程度。

### 7.4 适合纯视觉 MVP

在没有机载雷达、双目、测距仪的阶段，`LOS 6D + TTC` 能提供一条可运行、可记录、可调参的纯视觉导引链路。

## 8. 当前方案的边界和失败模式

### 8.1 TTC 对面积噪声敏感

TTC 依赖 `A_dot`，而导数天然放大噪声。检测框面积轻微跳动就可能造成 TTC 抖动。因此当前代码必须做：

- 面积低通滤波。
- 短窗口斜率估计。
- 面积突变拒绝。
- TTC 范围门控。

### 8.2 裁切后 bbox center 和 bbox area 都会失真

目标填满视野后，检测框可能被图像上边界、下边界或左右边界裁切。此时：

- bbox center 不再是目标真实投影中心。
- bbox area 不再代表目标真实尺度。
- LOS 俯仰或偏航会产生系统性偏差。
- TTC 会被低估或高估。

因此当前程序进入 `TerminalVisual` 和 `BlindPush`，而不是继续信任普通视觉更新。

### 8.3 纯横穿或弱闭合目标会削弱 TTC

若目标主要横向运动，面积膨胀弱，`A_dot` 可能很小甚至为负。此时 TTC 通道会拒绝输出，系统只能短时依赖 LOS fallback。

### 8.4 高机动目标会污染面积和角度

目标大幅滚转、俯仰或姿态剧变时，检测框面积变化不再只代表距离变化，bbox center 也可能随目标外形变化而偏移。TTC 必须降权，LOS 创新也可能升高。

### 8.5 LOS 滤波存在相位滞后

6D KF 可以抑制检测抖动，但会引入相位滞后。在近距离高速交会时，过强滤波可能导致修正晚到。当前代码提供 `--no-los-filter` 开关，用于无噪声仿真或延迟分析。

### 8.6 速度指令不是物理加速度指令

当前 AirSim 程序用 `moveByVelocityAsync` 发送速度指令。实际飞控会再把速度误差转成姿态、推力或电机输出。它与经典 PNG 直接输出 `a_cmd` 的物理链路不同。

实机或 PX4 SITL 中，速度指令可能受到：

- 最大倾角限制。
- 速度环带宽。
- 加速度限制。
- offboard 指令频率。
- EKF 延迟。
- failsafe 和模式切换。

因此仿真中的 `K(TTC)`、`los_fallback_gain`、速度上限不能直接照搬到实机。

## 9. 当前参数含义

常见参数和含义如下：

| 参数 | 含义 |
|---|---|
| `process_lambda` | LOS 方向过程噪声 |
| `process_lambda_dot` | LOS 角速度过程噪声 |
| `measurement_noise` | LOS 测量噪声 |
| `innovation_reject` | LOS 创新拒绝阈值 |
| `alpha_area` | 面积低通滤波系数 |
| `window_size` | 面积斜率估计窗口 |
| `min_area` | TTC 最小可信面积 |
| `max_area_jump_ratio` | 面积突变拒绝阈值 |
| `max_ttc_s` | 最大可信 TTC |
| `min_gain/max_gain` | TTC 动态增益范围 |
| `ttc_fast_s/ttc_slow_s` | 近距/远距增益切换时间 |
| `los_fallback_gain` | TTC 无效但 LOS 可用时的保底增益 |
| `speed_cap` | 速度指令上限 |
| `max_vision_lateral_speed` | 视觉侧向修正限幅 |
| `max_vision_vertical_speed` | 视觉垂直修正限幅 |

这些参数共同决定了导引的三个性质：

```text
平滑性：滤波越强，指令越平滑，但滞后越大。
敏捷性：增益越大，末端响应越快，但更容易过冲和饱和。
可信度：门控越严格，误导指令越少，但可能更早进入 coast/BlindPush。
```

## 10. 工程使用建议

### 10.1 当前纯视觉方案适合的场景

- 目标检测连续。
- 目标在中段没有严重遮挡。
- 目标尺寸在画面中能形成可观测的面积变化。
- 交会速度不至于让目标在数帧内穿越整个视场。
- 相机时间戳和姿态时间戳能够对齐。

### 10.2 需要谨慎的场景

- 近距离一开始 bbox 就很大。
- 纯横穿或弱闭合，面积膨胀不明显。
- 目标机动剧烈，姿态变化导致面积变化不代表距离变化。
- 目标从画面上边界或下边界饱和裁切。
- 云台达到 yaw/pitch 限位。
- 捷联相机机体转向能力不足，目标快速扫出视场。

### 10.3 与雷达/真值方案的关系

若引入毫米波雷达或真值基线，可以恢复经典 PNG 中缺失的两个关键量：

```text
R
Vc
```

此时更接近标准 PNG：

```text
a_cmd = N * Vc * (omega_LOS x lambda)
```

视觉 6D LOS 仍然有价值，因为它可以提供高精度角度；雷达可以补充距离和径向速度。更稳的融合方式是：

```text
视觉：lambda, lambda_dot
雷达：R, Vc, range_quality
融合：a_cmd = N(R, quality) * Vc * lambda_dot
```

这比用单目 bbox 强行估计距离更可靠。

如果仍不使用雷达距离，只想在纯视觉下采用 `V_m` 型导引，推荐把它作为 fallback 或受限模式：

```text
a_vis_like = N_vm * V_m * lambda_dot_I
```

工程上需要额外限制：

- `N_vm` 随 TTC 或像面误差动态调度，而不是固定不变。
- `a_vis_like` 必须限幅，并经过速度/姿态控制器可实现性检查。
- 当 TTC 无效、目标弱闭合或纯横穿时，不应把 `V_m` 型输出误判为可靠闭合导引。
- 实机中应优先发送受飞控支持的加速度设定点或姿态/角速度设定点，避免外层速度积分 wind-up。

## 11. 一句话总结

经典 `Vc` 型比例导引是“已知距离和闭合速度后，按 `N * Vc * lambda_dot` 输出法向加速度”；`V_m` 型比例导引是“缺少闭合速度时，用拦截器自身速度 `V_m` 近似速度尺度，按 `N * V_m * lambda_dot` 输出法向加速度”；当前 `LOS 6D + TTC` 是“单目视觉只可靠测角，用 6D KF 稳定 `lambda_dot`，用面积膨胀 TTC 补充接近趋势，再用动态增益生成速度修正”。它是为纯视觉单目拦截做出的工程化 PNG 变体，不是经典 PNG 或 `V_m` 型 PNG 的一比一实现。
