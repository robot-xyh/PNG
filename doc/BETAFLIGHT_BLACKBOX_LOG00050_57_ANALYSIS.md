# Betaflight LOG00050-LOG00057 Blackbox 分析

## 证据与解码方法

新导入文件为`logs/blackbox_import/LOG00050.BFL`至`LOG00057.BFL`。SD卡文件时间仍为2015年，
不能用于选择架次。分析使用Betaflight `blackbox-tools`提交
`f832acf9cd9dbe5ad8220de1a5f4eb4021523d72`，以`--unit-rotation raw --merge-gps`解码；8个文件
均为单航段、0帧解码失败且有`Log clean end`。解码器显示约74.5%的loop iteration未记录，是
`P interval=4`按四分之一PID循环记录的预期结果，不是文件损坏。

固件头中的`gyro_scale=0x3f800000`不能由标准解码器可靠换算为deg/s，因此本文只使用gyro raw
数值和频率，不把它写成物理角速度。原始证据图见
[`LOG00057_F01_INDOOR_summary.png`](evidence/LOG00057_F01_INDOOR_summary.png)，结构化指标见
[`LOG00057_F01_INDOOR_analysis.json`](evidence/LOG00057_F01_INDOOR_analysis.json)。该JSON由
`tools/analyze_betaflight_blackbox_flight.py`生成；工具不要求MSP OVERRIDE或algorithm区间，先按
ARM时长选择候选航段，再以物理油门和`rcCommand[3]`网格搜索完成时间对齐。

## 文件筛查

|文件|有效时长|油门高于idle|油门峰值|最低电压|最大电流|最大RPY setpoint|判读|
|---|---:|---:|---:|---:|---:|---:|---|
|LOG00050|0.032 s|0|1000|21.95 V|0.27 A|0/0/0|瞬时ARM/同步片段|
|LOG00051|116.547 s|1.237 s|1029|21.77 V|1.84 A|17/3/0|低电量长怠速，未飞行|
|LOG00052|0.457 s|0|1000|25.15 V|1.61 A|4/1/0|瞬时ARM片段|
|LOG00053|12.503 s|0|1000|25.01 V|6.01 A|121/103/0|低油门方向输入，未形成悬停|
|LOG00054|0.255 s|0|1000|25.15 V|1.44 A|7/1/0|瞬时ARM片段|
|LOG00055|35.064 s|29.768 s|1563|21.07 V|60.09 A|151/110/25|较高动态飞行，缺主机日志配对|
|LOG00056|113.963 s|0|1000|24.91 V|3.30 A|6/3/0|长怠速，未飞行|
|LOG00057|69.134 s|32.542 s|1298|22.55 V|76.59 A|13/14/0|与本次F01室内悬停匹配|

所有航段`failsafePhase=IDLE`、GPS卫星数为0。`LOG00051`的6S电压只有21.77-21.98 V，不应据此
批准再次起飞。`LOG00055`确有飞行动力和较大姿态指令，但没有对应Orange Pi CSV、现场动作和
架次标识，只能保留为未配对证据，不能与本次F01结论混用。

## LOG00057 与主机日志对齐

`LOG00057`的69.134 s与主机正式ARM窗口76.577-146.043 s唯一匹配。将主机RC油门按
`min_check=1050`转换到Blackbox `rcCommand[3]`后，最佳关系为：

```text
host_elapsed_s = blackbox_elapsed_s + 76.672
throttle correlation = 0.999398317
throttle RMSE = 4.46 command units
```

68个独立MSP ANALOG样本与Blackbox的电压/电流相关系数分别为0.9787/0.8932，进一步确认配对。
主机1 Hz电源轮询只能看到24.3 V和6.71 A，不能捕捉Blackbox中的毫秒级尖峰。

## 悬停、控制器与电机

按可复现规则取Blackbox 38.970-67.772 s为稳定悬停段，共28.803 s。油门中位/95%/最大为
1264/1272/1298；对应主机物理油门约1301/1308/1333 us。四电机raw中位为
`[668,656,630,668]`，95%为`[702,692,662,702]`，最大仅`[763,796,740,765]`，没有饱和。
复用LOG00042同一固件下的MSP标定，四电机中位约为`[1311,1305,1292,1311] us`，电机极差95%约
54 us；该换算是继承标定，不替代本架次RPM遥测。

稳定段roll/pitch范围为-1.7至2.7度、-1.7至4.4度。飞手setpoint的95%绝对值为
`[6,8,0]`，gyro raw为`[7,8,1]`；最大值分别为`[13,14,0]`和`[18,17,5]`。这说明本架次是小幅
人工悬停，未见持续失控或稳态电机饱和。四轴命令全部来自接收机：Orange Pi日志中
`MSP_SET_RAW_RC=0`、`override_active=0`，因此本结果不验证PNG接管或比例导引拦截能力。

## 振动与落地瞬态

稳定段未滤gyro主峰约在99-103 Hz；100-350 Hz能量经飞控gyro滤波明显降低，roll/pitch高频RMS
约下降17-18 dB。该结果说明当前滤波在本次低油门悬停中有效，但`dshot_bidir=0`，没有RPM遥测，
单架次也不足以批准PID或滤波调参。

Blackbox 67.772-68.772 s的结束窗口与稳定段不同：68.226 s后出现多轴快速运动，电机达到2047
累计10.7 ms，电流高于60 A累计20.2 ms，电压低于23 V累计59.3 ms；最大电流76.59 A、最低
22.55 V。事件发生在油门切回idle前不足0.55 s，结合现场“人工落地后DISARM”，最可能是触地或
落地姿态扰动，但没有视频不能断言。后续要求触地立即收油并DISARM，Blackbox电流和电机饱和仍作为
独立终止项检查，不能只看网页的低频电源数据。

## 结论

本次F01可升级为“人工室内短悬停与Blackbox采集链路通过，落地瞬态待飞手复核”。它证明接收机、
飞控PID、电机和日志在约28.8 s稳定悬停段内可工作，也证明Blackbox补足了主机低频遥测漏检。
它不证明GPS/NED、相机外参、无目标假检、主动PNG控制或拦截成功率；下一次仍应先做无目标天空
LOG_ONLY和室外GPS/NED科目，再讨论RC7接管。

复现解码与分析命令：

```bash
/tmp/betaflight-blackbox-tools/obj/blackbox_decode \
  --unit-rotation raw --merge-gps --save-headers \
  --output-dir /tmp/blackbox_recent_raw \
  logs/blackbox_import/LOG00050.BFL logs/blackbox_import/LOG00051.BFL \
  logs/blackbox_import/LOG00052.BFL logs/blackbox_import/LOG00053.BFL \
  logs/blackbox_import/LOG00054.BFL logs/blackbox_import/LOG00055.BFL \
  logs/blackbox_import/LOG00056.BFL logs/blackbox_import/LOG00057.BFL

mkdir -p /tmp/f01_indoor
tar -xzf logs/deployment_archives/F01_INDOOR_20260831_143812.tar.gz \
  -C /tmp/f01_indoor

python3 tools/analyze_betaflight_blackbox_flight.py \
  --host-csv /tmp/f01_indoor/logs/flight_20260831/F01_INDOOR_20260831_143812_20260831_143814.csv \
  --blackbox-csv /tmp/blackbox_recent_raw/LOG00057.01.csv \
  --blackbox-bfl logs/blackbox_import/LOG00057.BFL \
  --decoder-commit f832acf9cd9dbe5ad8220de1a5f4eb4021523d72 \
  --current-high-a 60 \
  --motor-scale-us-per-raw 0.499231573 \
  --motor-offset-us 977.151031 \
  --output doc/evidence/LOG00057_F01_INDOOR_analysis.json
```
