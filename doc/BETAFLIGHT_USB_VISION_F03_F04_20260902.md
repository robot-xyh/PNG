# Betaflight USB 无桨 F03/F04 感知预检（2026-09-02）

## 1. 目的与边界

本轮按以下约束执行：拦截机拆桨、DISARM、RC7人工侧，飞控由USB供电；Orange Pi运行
Python `LOG_ONLY`，目标无人机由操作员手持。测试用于复核真实YOLO+ByteTrack检测、静止
跟踪、横穿LOS符号、出框失效和端到端软件链路年龄，不评价GPS/NED、动力振动、双机实飞、
闭环稳定性或拦截成功率。

两轮均使用：

```text
config/betaflight.rk3588.kinematics_log_only.example.json
detector_source = rknn_bytetrack
main loop = 20 Hz
perception = 30 Hz
attitude poll = 20 Hz
web preview = enabled
```

全程未使用`--allow-control`，也未切入`msp_raw_rc`。

## 2. 飞控配置归档

完成磁罗盘校正后，从当前`orangepi5max`的`/dev/ttyS1`只读导出CLI并采集MSP快照：

```text
diff all   config/betaflight_diff_all_20260902_204657.txt
           SHA256 4fb86d60c213f7fbc26fa4c9bb96e03f190f699d27da156715685bcf2ad3062d
dump all   config/betaflight_dump_all_20260902_204657.txt
           SHA256 eb8de617eec88ee88305a09ca05a5ea2343c1244356d00ae96ab437b0d224e09
snapshot   logs/betaflight_snapshots/betaflight_snapshot_20260902_204718/manifest.json
           SHA256 946d21748490aef9c6a4762e60594ca85b9b309f56fa24a820f1094533e88107
approval   logs/betaflight_noprop_approval.json
           SHA256 777317949d9c9cb9ed2a6d969fe038b772d5cff7d4869d1ac4777c3db2901d37
```

快照包含`diff all`和`dump all`，`configuration_evidence_complete=true`，25个MSP样本均成功，
并确认`MSP OVERRIDE permanent_id=50`。关键配置为：

```text
aux 2 50 2 1700 2100 0 0
msp_override_channels_mask = 15
align_mag = CW270FLIP
mag_calibration = -38,127,-132
mag_declination = -33
```

旧批准文件已另存为`betaflight_noprop_approval.pre_mag_20260902_204718.json`，避免与新校准
配置混用。本地副本位于`logs/betaflight_config_20260902/`。

## 3. 历史飞行数据复用结论

### 3.1 F02与R_IB

`LOG00065`可复用为带桨状态下GPS原始N/E速度方向证据：左移为`+E 5.29 m`、右移为
`-E 6.56 m`、前移为`-N 9.66 m`、后移为`+N 5.80 m`，四段主方向速度样本均100%同号。
但该架次发生在磁罗盘由`DEFAULT`修正为`CW270FLIP`之前，因此不能证明校正后航向与GPS速度
组合得到的`R_IB`动态转换正确。F02只能登记为“GPS N/E符号通过、校正后R_IB动态验证未关闭”。

### 3.2 人工LOG_ONLY飞行

人工飞行基线无需重复：`LOG00063`已给出41.20 s稳定悬停窗口，物理油门中位1275 us；
`LOG00065`已确认Betaflight Rates `1.0/0/0.7`，Roll/Pitch设定值到gyro延迟10/11 ms、
相关系数0.9913/0.9934。主机与Blackbox Roll/Pitch RC合并RMSE为1.44 us，两架次均为零
`MSP_SET_RAW_RC`。这些证据通过人工飞行、Rate映射和日志链路，不替代校正后的F02/R_IB复测。

## 4. F03 USB手持静止目标

日志为`F03_USB_HANDHELD_STATIC_20260902_205100.csv`，持续179.97 s，共3589行；安全审计
`passed=true`、零违规。

静止有效区间取`0.8--107.25 s`：

|指标|结果|
|---|---:|
|新感知结果|2050|
|有效目标测量比例|99.32%|
|ByteTrack confirmed比例|99.90%|
|LOS有效比例|99.27%|
|有效track ID|仅`1`|
|结果年龄P50/P95/max|95.49/145.79/189.27 ms|

该区间满足静止目标确认率和无ID切换要求；操作员按测试序列移开并重新放回目标后出现新ID，
属于重捕获，不计为静止连续段内ID switch。全日志包含移除阶段时，有效测量比例为91.38%，
结果年龄P95为143.58 ms。因此F03静止跟踪通过，但100 ms时延门槛未通过。

## 5. F04 USB手持横穿

有效重测日志为`F04_USB_HANDHELD_CROSSING_RETRY_20260902_205751.csv`，持续123.82 s，共
2471行；安全审计`passed=true`、零违规。首轮未形成横穿的日志不作为结果输入。

规定动作集中在`20.9--70.75 s`：

|指标|结果|
|---|---:|
|左右跨区次数|6|
|横向归一化中心范围|-0.843至+0.680|
|有效目标测量比例|95.82%|
|confirmed比例|96.76%|
|LOS有效比例|95.61%|
|结果年龄P50/P95/max|101.22/156.69/188.55 ms|

第一趟快速横穿发生短暂漏检，ID由`21`重建为`32`。此后的`ID=32`在`25.86--70.70 s`
覆盖连续5次反向横穿；该段有效目标测量比例98.26%、confirmed比例99.19%，没有再次换ID。

图像横坐标与惯性LOS的`lambda_I_y`相关系数为`-0.998868`。在本轮约181 deg机头航向和已验证
`R_BC`下，目标由左到右时该NED分量反向连续变化，六次横穿均完成符号反转。该结论只适用于
当前姿态和坐标变换，不把图像右移泛化为固定NED方向。

出框事件最后一次有效测量为`70.699 s`，`70.749 s`即变为无目标；`74.761 s`重新检测，
`74.811 s`恢复有效LOS。当前日志分辨率下，失效和重捕获确认均不超过约50 ms。测试后段目标
位于极左边缘时出现低分、间歇候选，说明边缘/截断目标仍容易碎片化。

因此F04的运动覆盖、LOS符号和出框失效通过；第一趟ID重建及P95时延不通过，不能把本轮登记为
完整F04或主动控制放行。

## 6. 时延增大原因

两轮的主要耗时分解如下：

|阶段|F03 P95|F04 P95|
|---|---:|---:|
|相机`read()`|9.42 ms|11.53 ms|
|RKNN总耗时|6.33 ms|6.39 ms|
|ByteTrack更新|0.92 ms|1.35 ms|
|姿态融合等待|100.12 ms|100.16 ms|
|最终结果年龄|143.58 ms|148.96 ms|

相机失败帧、RKNN worker错误、MSP请求错误和串口解析错误均为0；内核日志也没有欠压、USB reset、
disconnect或over-current记录。RKNN单帧推理保持约6 ms，所以没有证据支持“飞控USB功率不足导致
P95时延上升”。

直接原因是当前主循环20 Hz、感知30 Hz、姿态轮询20 Hz。检测结果需要等待时间戳不早于曝光时刻
的姿态样本，典型等待一个50 ms周期，P95达到两个周期；主循环消费频率又低于感知生产频率，增加
排队和融合丢弃。网页MJPEG预览可能贡献个别尖峰，但不是P95的主体。后续应离线验证提高主循环/
姿态轮询频率或把融合改为按最新可用姿态批量消费，不能通过增加USB电源电压来修复调度问题。

## 7. 安全与结论

两轮共同满足：

```text
safety_state = LOG_ONLY
armed = 0
msp_publish_mode = disabled
MSP_SET_RAW_RC attempt/write/success = 0
MSP request/parser/checksum errors = 0
```

证据包：

```text
logs/deployment_archives/USB_VISION_F03_F04_20260902_205100.tar.gz
SHA256 18b9dc51714b44c18435117a23dccc54200834cf4b5b2e534905640d86122c82
```

机器可读摘要见`doc/evidence/BETAFLIGHT_USB_VISION_F03_F04_20260902.json`。本轮不改变
`release_passed=false`，也不批准RC7/PNG主动接管。
