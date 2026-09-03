# Betaflight 无桨四航向验证（2026-09-02）

## 1. 目的与边界

本次测试用于核对物理机头方位与 `MSP_ATTITUDE.yaw_deg` 的关系，并复核当天
`LOG00065` 人工飞行中“左、右、前、后”动作对应的 NED 方向。测试全程拆桨、
DISARM、RC7 人工侧，Orange Pi 仅运行 `LOG_ONLY`；不测试 GPS 定位精度、主动控制或
比例导引拦截能力。

四个物理方位由操作员使用手机指南针确定。本轮 GPS 为 `fix=0/satellites=0`，所以方位
真值精度受手机指南针、附近金属和磁场影响，但四个独立方向足以判断航向增减方向是否正确。

## 2. 运行与安全审计

运行配置为 `config/betaflight.rk3588.kinematics_log_only.example.json`，采集命令使用
`--duration-s 300 --rate-hz 20 --control-mode log_only --detector-source camera_only`。

- 程序自然运行 `300.042 s`，写入 `5985` 行。
- `armed_seen=false`，停止原因为 `duration_complete`。
- 审计结果：`passed=true`、`violations=0`、`warnings=0`。
- 全程 `LOG_ONLY`、`publish=disabled`、`MSP_SET_RAW_RC=0`。

完整证据包位于
`logs/yaw_cardinal_remote/YAW_CARDINAL_NOPROP_20260902_162057.tar.gz`，SHA256 为：

```text
5f1f2ee92a727cdbab29efaa66f4d633a6e0f219366c9da61858b2a1cd80483a
```

## 3. 四航向结果

|物理机头方向|真值方位标称|采样区间（elapsed_s）|样本数|MSP yaw|
|---|---:|---:|---:|---:|
|北|0 deg|95.185--99.746|92|281 deg|
|东|90 deg|141.824--146.485|93|184 deg|
|南|180 deg|186.812--191.524|94|95 deg|
|西|270 deg|222.738--227.350|93|19--20 deg|

四方向可由以下关系拟合：

```text
yaw_MSP = wrap_360(279.744 deg - heading_cardinal)
RMSE = 5.974 deg
max_abs_error = 9.256 deg
```

物理机头从北顺时针转向东时，MSP yaw 从 `281 deg` 降至 `184 deg`。因此问题不只是
固定航向零偏：航向增减方向也与项目采用的 FRD/NED 约定相反。仅设置一个
`mag_align_yaw` 常数不能同时修正符号和零偏。

当前已归档 Betaflight 配置显示：

```text
align_mag = DEFAULT
mag_align_roll = 0
mag_align_pitch = 0
mag_align_yaw = 0
align_board_roll = 0
align_board_pitch = 0
align_board_yaw = 0
```

应优先检查 QMC5883 的实际安装面、连接板坐标、`align_mag` 枚举和磁罗盘校准；不能先在
Python 中添加经验偏置掩盖飞控航向轴向问题。

### 3.1 CLI 导出与厂商目标配置比对

后续取得的完整 CLI 导出为
`logs/BTFL_cli_20260902_013722_MICOAIR743V2.txt`，其 SHA256 为：

```text
4e0c84c56f5f5a9d6a375b4aa59acde85bf5a868024bc0b22a1544a2b42e90f1
```

除上述对齐值外，CLI 还给出：

```text
MAG: QMC5883
mag_bustype = I2C
mag_i2c_device = 1
mag_hardware = AUTO
mag_calibration = 0,0,0
mag_declination = 0
gps_rescue_use_mag = ON
```

操作员确认 QMC5883 位于飞控板上，且飞控板箭头朝向机头。该确认支持继续保持
`align_board_roll/pitch/yaw=0`，但不能单独证明磁罗盘芯片的 X/Y/Z 轴与机体系一致。

MicoAir 官方 `MICOAIR743V2_INTMAG` 目标配置定义为：

```text
MAG_I2C_INSTANCE = I2CDEV_2
MAG_ALIGN = CW180_DEG
MAG_ALIGN_YAW = 1800
```

而 `MICOAIR743V2_EXTMAG` 目标使用 `I2CDEV_1`，没有定义固定 `MAG_ALIGN`。当前 CLI 的
`I2C1 + DEFAULT + custom yaw 0` 与 EXTMAG 目标默认特征一致，与厂商 INTMAG 定义不一致。
这可能表示烧录了 EXTMAG 固件、保存配置覆盖了目标默认值，或实机实际读取的是外置/GPS
模块上的 QMC5883。仅凭 `status` 中的器件名称无法区分来源。

随后在断电后拔除 GPS/外置模块，仅用 USB 重新启动飞控，`status` 变为：

```text
DEVICES DETECTED: SPI=1, I2C=1 (10 errors)
GYRO: (1) BMI270
ACC: BMI270
BARO: DPS310
GPS: NOT CONNECTED
MAG: 不再出现
```

该隔离试验证明此前活动的 QMC5883 来自已拔除的 GPS/外置模块。即使飞控板上物理存在
另一颗 QMC5883，它也不是当前固件读取和产生 MSP 航向的数据源。当前应继续使用 EXTMAG
路径和 `mag_i2c_device=1`，不得刷 INTMAG 固件或改为 I2C2 来替代外置磁罗盘校正。

### 3.2 校正顺序

1. 拆桨、DISARM、仅 USB 供电，停止 Orange Pi 程序并关闭所有 RC 写入。
2. 断开外置 GPS/磁罗盘模块后重启，再执行 `status`。本机已完成该项，QMC5883 随模块
   消失，确认活动数据源为外置磁罗盘。
3. 不修改 `align_board_*`、`mag_i2c_device`，也不套用 INTMAG 的 `CW180` 默认值。重新接好
   外置模块后，使用 Alignment Detect 按外置模块的实际芯片安装方向确定对齐枚举。
4. 使用 Betaflight Configurator 的 Magnetometer Guided/Free Calibration，完整覆盖六个
   姿态并保存。CLI 的 `mag_calibration` 必须由 `0,0,0` 变为非零结果。
5. 重新检查北/东/南/西，应接近 `0/90/180/270 deg`，并确认从北顺时针转到东时航向
   增大。校准只能消除偏置，不能把反向航向判为通过。
6. 若仍反向，使用 2025.12 Configurator 的 Alignment Detect 采样并记录建议枚举和置信度；
   不猜测 `FLIP` 枚举。应用建议后重新校准并重复四航向验证。
7. 只在 USB 无桨验证通过后，才接动力电池保持无桨、DISARM 重测磁干扰。若航向仍未
   通过，带桨飞行前必须令 `gps_rescue_use_mag=OFF`，并继续禁止依赖 `R_IB` 的主动控制。

### 3.3 最终校正结果

外置磁罗盘重新接入后，Configurator Alignment Detect 以 `12.2x high` 置信度建议
`CW270FLIP`。应用该对齐并重新校准后，最终 CLI 配置为：

```text
mag_i2c_device = 1
align_mag = CW270FLIP
mag_calibration = -38,127,-132
mag_declination = -33
```

`mag_declination=-33` 是 Betaflight 的 0.1 deg 单位，对应 `-3.3 deg`。该值按
`22.7991667 deg N, 113.8600000 deg E` 和 2026-09-02 的 NOAA/NCEI WMM-2025 结果
`-3.33462 deg` 四舍五入得到。

操作员随后确认真北基准下北、东、南、西四个方向均正确，且俯仰显示符合 Betaflight
约定。由于未记录四个方向的精确航向数值，本项只能关闭“方向反转/象限错误”问题，不能
给出绝对航向误差、RMSE 或通过 `+/-15 deg` 定量门限的结论。USB 无桨方向验证可记为
定性通过；功率电无桨磁干扰复测仍未完成。

校正后又由 Orange Pi 经 `/dev/ttyS1` 运行 300 s `LOG_ONLY` USB 静态复测。飞机保持固定，
本轮共记录 `5979` 行：

```text
duration                 299.990 s
yaw_deg                  181.0 ... 181.0 deg
roll_deg                 1.2 ... 1.3 deg
pitch_deg                0.3 ... 0.4 deg
vbat_v                   4.7 ... 4.7 V
attitude_age_s max       0.139870 s
i2c_error_count          6 ... 6
msp_worker_poll_errors   0
armed_seen               false
MSP_SET_RAW_RC attempts  0
```

该结果证明校正后的航向在 USB 静态环境中没有可见漂移或量化跳变，且采集过程没有控制写入。
`msp_request_error_count=1` 从首行起保持不变，未形成持续通信错误。证据包为
`logs/yaw_power_retest_remote/YAW_USB_STATIC_20260902_203319.tar.gz`，SHA256：

```text
c8f4cac6a182aa745f4508119a25ef8537135198b6adea64a0e958188242cba8
```

由于本轮 `vbat_v=4.7 V`，它不是功率电复测，不能评价 6S 供电、ESC、动力线或电机电流
造成的磁干扰。

## 4. 对当天飞行数据的解释

当天 `LOG00065` 飞行的人工动作顺序为左、右、前、后。Blackbox GPS 主运动段给出：

|动作|实测 NED 方向|主位移|主速度中位|
|---|---|---:|---:|
|左|东 `+E`|`+5.29 m E`|`v_e=+2.56 m/s`|
|右|西 `-E`|`-6.56 m E`|`v_e=-2.20 m/s`|
|前|南 `-N`|`-9.66 m N`|`v_n=-3.48 m/s`|
|后|北 `+N`|`+5.80 m N`|`v_n=+2.83 m/s`|

四段在主方向速度绝对值不小于 `0.5 m/s` 的样本中均为 `100%` 同号。这一映射与
“物理机头朝南”完全一致；飞行期间主机记录的 `yaw=89--95 deg` 也与本次无桨南向
`yaw=95 deg` 一致。

因此 `LOG00065` 可以作为带桨状态下 GPS 的北/东速度符号和正反运动证据，但该架次记录的
MSP yaw 发生在磁罗盘校正之前，不能作为正确的机体到 NED 航向。项目中的
`attitude_degrees_to_R_IB()` 假设航向已经符合 FRD/NED；校正后的 USB 静态数据已关闭方向
反转问题，但在功率电磁干扰复测和定量四航向误差完成前，仍不批准依赖 `R_IB` 的惯性 LOS、
NED 加速度到机体系转换和速度闭环主动输出。

## 5. 后续关闭条件

1. 已定性复核真北四方向；仍需记录四个精确读数以计算绝对误差。
2. 已将外置 QMC5883 修正为 `CW270FLIP` 并重新校准。
3. 已确认顺时针转动时四个象限顺序正确；`+/-15 deg` 定量门限仍待精确读数。
4. 接动力电池但保持无桨、DISARM 再重复一次，检查实际供电环境下航向误差和抖动。
5. 上述两轮均通过后再复核 `R_IB` 与 GPS 速度；本报告本身不改变
   `release_passed=false`，也不批准 RC7/PNG 主动接管。
