# Betaflight/PNG 转动台架测试方案

## 1. 目的与边界

转动台架位于拆桨手持检查和带桨自由飞行之间，用已知角运动替代无法稳定复现的手持动作。主要
验证Betaflight姿态/角速度、相机外参`R_BC`、视觉时间戳、姿态插值、惯性LOS去旋转、YOLO+
ByteTrack连续性和出框重捕获。默认全程拆桨、DISARM、RC7人工侧和`LOG_ONLY`，不得传入
`--allow-control`。

台架不能产生可信平移、升力、悬停油门、闭合速度或碰撞几何，因此不能证明自由飞行稳定性、TTC
精度、比例导引命中率或主动控制安全。刚性固定机体无法按飞控指令产生预期姿态响应，禁止用长时
RC7接管测试；此前固定机体已出现PID/I-term累积和电机联锁触发。

## 2. 需要先确认的台架参数

|项目|记录值|
|---|---|
|厂家、型号、序列号|__________|
|可动轴及正方向|__________|
|机械角度范围/限位|__________|
|连续/峰值角速度|__________|
|额定整机质量、力矩和推力|__________|
|编码器类型、分辨率、采样率|__________|
|编码器时间戳来源|__________|
|急停、断电、机械锁止方式|__________|
|锚固、防护罩和人员隔离距离|__________|
|是否由制造方明确允许带桨动力测试|□是 □否|

无编码器时可用高帧率外部相机和角度标尺作为降级真值，但只能验证符号和大致重复性，不能给出
可信的毫秒级延迟、角速度比例误差或动态残差结论。

## 3. 坐标与计算合同

- Betaflight机体系使用FRD：`X`向机头、`Y`向右、`Z`向下。
- OpenCV相机系：`x`向图像右、`y`向图像下、`z`沿光轴向前；上视光轴应映射到机体`-Z`。
- `R_BC`把相机向量变换到机体系；`R_IB`把机体向量变换到惯性系。
- 去畸变像素形成相机LOS：

```text
lambda_C = normalize([(u-cx)/fx, (v-cy)/fy, 1])
lambda_I = normalize(R_IB * R_BC * lambda_C)
```

世界固定目标且台架只绕相机附近转动时，正确外参和姿态对齐应使`lambda_I`近似恒定。以稳定段
平均LOS `lambda_ref`为基准：

```text
angle_residual = acos(clamp(dot(lambda_I, lambda_ref), -1, 1))
omega_LOS = cross(lambda_I, d(lambda_I)/dt)
```

另将编码器角速度变换到机体系，拟合：

```text
omega_msp(t + delay) = scale * omega_rig(t) + bias
```

报告每轴的符号、`delay`、`scale`、`bias`、主轴相关性和交叉轴响应。对固定目标，还可用
`omega_C = R_CB * omega_B`和`d(lambda_C)/dt ~= -cross(omega_C, lambda_C)`检查图像运动方向。

`R_BC`必须满足`R_BC.T * R_BC ~= I`、`det(R_BC)=+1`，并满足
`R_BC*[0,0,1] ~= [0,0,-1]`。矩阵代数正确不等于实物安装已验证；只有单轴正反运动、重复性和
惯性LOS残差均通过后，才允许评审是否把`extrinsic_validation.verified`改为`true`。

## 4. 安装、同步与启动

1. 台架空载检查锚固、轴锁、软/硬限位和急停；在全运动范围内确认不会碰撞人员、地面或线缆。
2. 拆下全部桨，断电安装飞机，使重心和相机尽量靠近旋转中心；固定电池、LQ、Orange Pi和线缆。
3. 在机体和台架上标注FRD方向、台架轴正方向与零位；每次改变安装后重新执行R00-R03。
4. RC5保持DISARM、RC7保持人工侧，关闭Configurator和其他串口/相机进程。
5. 台架编码器优先记录单调时钟。若它与Orange Pi不共享时钟，在每段开始/结束执行可同时出现在
   编码器、CSV/Blackbox或外部视频中的清晰事件边沿；不能只依赖NTP墙钟。
6. 使用与飞场相同的50 Hz只读配置，每段使用独立前缀：

```bash
cd /home/orangepi/png_betaflight_python

python3 -u examples/run_betaflight_log_only.py \
  --config config/betaflight.rk3588.kinematics_log_only.example.json \
  --duration-s 300 --rate-hz 50 \
  --control-mode log_only \
  --detector-source rknn_bytetrack \
  --isolate-rknn-process \
  --main-cpu-affinity 6,7 \
  --rknn-cpu-affinity 4,5 \
  --log-prefix R01_roll_20260831
```

启动后确认`LOG_ONLY`、`publish=disabled`、`armed=0`、`override_active=0`，并确认所有
`MSP_SET_RAW_RC`计数为0。目标使用模型类别对应的真实无人机，固定在安全位置；测试期间目标自身
不得移动。

## 5. 测试矩阵

### R00 零位和静态基线

- 每个机械安装状态记录至少30秒静止数据。
- 记录台架零位、MSP attitude、`gyro_msp_raw_*`、Blackbox gyro、目标bbox中心和惯性LOS。
- 检查目标、台架和机体无相对松动；静态track ID稳定，时间戳单调递增。

### R01 单轴正反运动

- 每次只解锁一个轴，其余轴机械锁定。
- 对每个可用轴做正向、回零、反向、回零，各重复3次；先低速，角度不超过台架批准范围。
- 比较编码器、MSP attitude、RAW IMU和Blackbox的正负号、主轴、幅值和回零偏差。
- 若主响应出现在错误轴、同一方向重复时符号变化或出现明显线缆回弹，本轴失败。

### R02 外参和惯性LOS

- 固定目标先位于画面中部，再放到四个非中心位置分别重复。
- 对roll/pitch/yaw可用轴执行低速正反运动，计算`lambda_C`和`lambda_I`。
- 比较候选`R_BC`及所有物理上可能的轴交换/符号组合；不得只凭单次画面方向选择矩阵。
- 输出每轴每方向的角残差P50/P95/max、LOS角速率残差和交叉轴残差。

### R03 动态时延和跟踪

- 在台架额定能力内设置低/中两档角速度，各执行至少3个完整三角波或正弦周期。
- 用编码器与MSP/Blackbox角速度互相关估计飞控链路延迟；用目标图像运动与姿态预测估计图像
  时间戳偏差和抖动。
- 记录`detection_attitude_offset_ms`、`perception_result_age_ms`、`timestamp_after_buffer`、队列
  丢弃、MSP RTT和解析错误。
- 保持目标在视场内的段落统计confirmed比例、ID switch和fragment；随后受控出框1至2秒再重入，
  检查旧测量是否撤销及重捕获事件。

### R04 可选带桨怠速

仅在台架制造方明确批准整机质量、推力和振动载荷，且锚固、防护、机械限位、遥控DISARM和物理
急停均独立有效时执行。操作员撤离旋转/桨平面后，由飞手ARM，只观察稳定怠速，不做油门阶跃，
RC7保持人工侧，程序仍为`LOG_ONLY`。任何振动、异响、台架位移、电机输出明显分化或温升立即
DISARM并断电。R04数据不得用于PID调参或推断自由飞行响应。

## 6. 验收与待标定门限

以下为硬门禁：

- 全部记录均为`LOG_ONLY/disabled`，所有RC写入计数为0。
- MSP、checksum、parser、worker和相机错误为0；日志时间戳单调。
- 每个测试轴正反方向符号一致，主轴映射正确，无轴交换。
- `R_BC`正交、det=+1，上视光轴误差不超过配置的5度。
- 视场内ByteTrack confirmed比例至少95%，单段ID switch为0。
- `perception_result_age_ms` P95小于100 ms，最大值不持续超过120 ms。
- 出框后不继续输出旧目标，重入有明确reacquire/track事件。

以下门限依赖编码器和台架精度，必须在运行前填写，不得测试后迁就数据：

|指标|预先门限|结果|
|---|---:|---:|
|MSP/编码器角度误差P95|__________ deg|__________|
|主轴角速度比例误差|__________ %|__________|
|交叉轴响应/主轴响应|__________ %|__________|
|MSP/编码器延迟P95|__________ ms|__________|
|视觉姿态对齐残差P95|__________ ms|__________|
|惯性LOS角残差P95|__________ deg|__________|
|惯性LOS角速率残差P95|__________ rad/s|__________|

若无足够真值确定这些门限，结果标记为“方向检查”，不能标记为“动态标定通过”。

## 7. 数据格式与归档

台架真值至少包含：

```text
sample_monotonic_s, axis, angle_deg, rate_deg_s, command, limit_active, estop_active
```

每段必须保存Orange Pi CSV/meta/events/audit、原始Blackbox `.BFL`、台架原始编码器文件、同步事件
说明、外部视频、台架参数照片和测试矩阵。建议前缀：

```text
R00_zero_<date>
R01_<axis>_<speed>_<date>
R02_extrinsic_<target_position>_<date>
R03_dynamic_<axis>_<speed>_<date>
R04_idle_<date>
```

分析报告必须同时给出通过项、失败项、真值质量、时间对齐方法和未确定门限。台架通过后仍需执行
带桨测试卡中的P00/F01；只有自由飞行数据才能验证真实振动、气动力、平移速度和飞控闭环响应。
