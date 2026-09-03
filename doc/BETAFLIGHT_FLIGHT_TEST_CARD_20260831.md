# Betaflight/PNG 飞手人工飞行 LOG_ONLY 测试卡（2026-08-31，更新至 2026-09-02）

> 当前离线评估 `release_passed=false`。本卡只允许飞手人工控制，Orange Pi 运行 Python
> `LOG_ONLY` 采集。全程禁止 RC7/MSP OVERRIDE 接管、`--allow-control`、自主逼近、碰撞航线和
> 比例导引闭环。本卡通过只能证明实飞数据链路和算法观测质量，不能证明 PNG 拦截已放行。

当前进度：2026-08-31已完成一次室内`F01_INDOOR`短悬停，人工飞行和日志链路有条件通过，但
发现启动时RC5实际处于ARM区、室内无目标假检及触地前电机/电流瞬态。2026-09-01已完成拆桨
`G00`和高度复测：GPS fix、原点及水平N/E数据链路可用，现场明确记录了东移，但未用动作标记完整
覆盖北/南/东/西；DISARM期间气压D位置有明显慢漂，两次局部抬升响应约1.795/1.780 m，
但缺同步高度真值。DISARM日志中MSP vario/`v_d`为0；后续ARM推力实验的825个高度样本中
479个vario非零，证明动态垂直速度链路存在，但精度和延迟尚未标定。当天还完成
三次`F00_SKY_GROUND`共约654.3 s；静态天空/建筑背景子项通过，瞬时小目标被ByteTrack拒绝，
但操作员确认的真实飞机被单类UAV模型持续confirmed并产生制导候选，且四航向覆盖未补齐。室外
F01、`F00_SKY_HOVER`和F03-F06尚未按正式科目执行；F02后来由带桨动作和两次高度复测的历史
数据组合关闭功能项。当天另完成一次`THRUST_ENVELOPE`人工油门脉冲，
`LOG00062`确认1500 us附近形成约0.4 s、2.37 g平台且脉冲内电机未饱和。落地末端的40.92 A、
23.28 V和单电机1985 raw已确认为正常触地姿态修正瞬态，仍从推力拟合中排除。该结果只作为已测短时上限，
不是最大推力或主动控制批准；
`release_passed=false`及主动控制禁令不变。2026-09-02的`LOG00063`确认稳定悬停物理油门中位
`1275 us`，`LOG00065`确认Betaflight Rate profile 0为`RC Rate=1.0 / Super=0.7 / Expo=0`，
Roll/Pitch命令到gyro响应约10/11 ms。实测`MSP_RAW_IMU / 16`转换已进入runner，但仍绑定
`BTFL 25.12.2 / API 1.47`。2026-09-02两轮固定目标动作试验确认本机显式
`R_BC=[[0,1,0],[1,0,0],[0,0,-1]]`，各姿态惯性LOS残差均小于5 deg；同时确认RAW_IMU转FRD
符号为`[+1,-1,+1]`，独立pitch复测相关系数为`+0.969`；20 Hz Yaw复测9/9个事件同号。
硬件曝光时间戳仍未关闭。磁罗盘修正后的CLI、MSP快照和无桨批准文件已于2026-09-02重新归档。
历史`LOG00065`严格复算后确认带桨GPS N/E正反符号和自然起降D符号；旧航向离线还原后代入
当前`R_IB.T @ v_NED`，四个机体动作主轴符号全部正确。校正后的四方位确认和300 s静态航向又
关闭了当前航向象限问题，因此F02水平、垂直及`R_IB`功能符号均按跨日志证据通过，无需专门
重飞或无桨手持复测。仍未量化6S动力磁干扰和计量级高度精度，但前者改为正常人工LOG_ONLY
架次的监测项，不再列为独立F02门槛。
同日USB无桨手持目标预检中，F03静止区间confirmed为99.90%且无ID变化；F04完成6次左右跨区，
图像横坐标与`lambda_I_y`相关系数为-0.9989。两轮结果年龄P95为143.58/148.96 ms，主要来自
20 Hz主循环/姿态轮询的50--100 ms融合等待，因此100 ms门槛未通过。随后使用50 Hz主循环、
20 Hz姿态轮询、30 Hz隔离RKNN进程执行120 s VM最终链路
LOG_ONLY复测：静止20 s为单一ID且confirmed/LOS均100%，连续水平段完成3次反转且ID不变；有效
结果年龄P95降至84.56 ms，融合等待P95降至40.39 ms。`fixed_vm_png -> accel_tilt_rate`两轴均
产生正负有限候选，失效时非零Rate为0，全程RC写入为0。本轮关闭室内实现链路门槛，但不改变
主动控制禁令。

同日硬件约束MC100又使用真实约`62x50 deg`视场、84.556 ms软件P95、154.139 ms保守物理时延
预算和实测中央漏检突发模型评估3000条轨迹。两个场景的初始可见总体命中率为96.92%/97.31%，
FOV可行命中率为89.38%/86.54%，均超过新定义的初期80%聚合目标，故
`initial_performance_target_passed=true`。但这只是离线模型结果，不是实飞命中率；完整门槛仍因
FOV连续性、陈旧失败率和最差距离失败，`release_passed=false`。速度建立型控制器仍为offline-only，
尚未接入生产runner或MSP输出，所以本测试卡继续只允许人工飞行和`LOG_ONLY`。

## 1. 本次启动哪个程序

唯一允许启动的飞行采集程序是：

```text
/home/orangepi/png_betaflight_python/examples/run_betaflight_log_only.py
```

使用配置：

```text
config/betaflight.rk3588.kinematics_log_only.example.json
```

该配置读取 Betaflight MSP、GPS、气压高度、姿态、相机和 RKNN/ByteTrack，同时计算候选
LOS/TTC/PNG 诊断，但不发送 `MSP_SET_RAW_RC`。不要启动以下程序：

- `src` 中的 C++ `bf_flight` 或 `bf_flight_png`
- 带 `--control-mode msp_raw_rc` 或 `--allow-control` 的命令
- `png-betaflight-log-only.service`（本次使用前台命令，便于立即发现错误和正常停止）

## 2. 人员、场地与硬边界

日期/场地：__________  风速/天气：__________  空域许可：__________

拦截机/电池/桨编号：________________  目标机/电池编号：________________

- [ ] 拦截机飞手：只看飞机和现场，始终拥有人工控制权
- [ ] 目标机飞手：独立控制目标机，不进入非计划闭合航线
- [ ] 安全观察员：监视人员、空域和两机间距，拥有立即终止权
- [ ] Orange Pi 操作员：启动、监看、停止程序，不碰遥控器
- [ ] 已约定口令：`准备`、`开始`、`动作结束`、`立即终止`
- [ ] 起降区、人员区、两机工作区已分离
- [ ] 已填写最小水平间距：______ m；最小垂直间距：______ m
- [ ] 风、电池、场地或人员条件超出飞手批准范围时取消测试

LQ、Wi-Fi、网页和 Orange Pi 均不是飞行安全链路。任何计算机或数传故障都由飞手人工返航和
降落，不允许依赖程序自动处置。

## 3. 飞场前配置确认

### 3.1 Betaflight 与遥控器

在装桨前使用 Configurator 确认，完成后必须关闭 Configurator：

- [ ] 固件/目标：MICOAIR743V2，Betaflight 2025.12.2，MSP API 1.47；如不同则停止
- [ ] Receiver 页四个主通道方向、AETR 顺序、中位和端点正常
- [ ] RC5/AUX1 是 ARM，当前有效区间为低位 `900-1300`
- [ ] RC7/AUX3 是 MSP OVERRIDE，当前配置应为 `aux 2 50 2 1700 2100 0 0`
- [ ] ARM 和 MSP OVERRIDE 使用两个独立物理开关
- [ ] 当前实测 RC5约`1000`进入ARM区，RC5约`2000`才是DISARM；RC7人工侧、油门最低
- [ ] 不依赖遥控器开关标签或操作者口述判断ARM状态，启动前必须在8080页面确认`armed=false`
- [ ] `msp_override_channels_mask=15` 已记录，但本次 RC7 始终不得拨入接管侧
- [ ] 遥控 failsafe、DISARM 和人工飞行模式由飞手独立验证
- [ ] `diff all`、`dump all`、Rates/PID/Mode/Receiver 页面截图已归档
- [ ] Blackbox SD 卡 Ready、剩余空间足够，已记下起始日志编号

### 3.2 飞机、动力与计算平台

- [ ] 机架、机臂、电机、螺钉、天线、GPS、相机和 Orange Pi 固定可靠
- [ ] 相机镜头清洁、上视无遮挡，安装方向未在标定后改变
- [ ] 四副桨型号、旋向、正反面和锁紧方式正确，无裂纹或缺口
- [ ] 电池健康、固定和重心正常；所有线缆远离桨盘和活动部件
- [x] 推力科目实机基本参数：起飞质量`2.412 kg`、`6S 10000 mAh`、`3115-900KV`电机、共四副10x5.0三叶桨、手制ESC
- [x] 桨叶实物标记`1050R`，确认标称直径10英寸、桨距5.0英寸；`R`为反向旋转版本
- [ ] 补充电池C数；ESC额定电流未知，暂记为非当前限制项，不等于无电流上限
- [ ] LQ 天线必须先安装再上电；AP=`192.168.1.200`、STA=`192.168.1.201`
- [ ] 地面电脑 LQ 网口=`192.168.1.123/24`，Orange Pi=`192.168.1.10/24`
- [ ] Configurator、C++ src、debugd、旧 Python 和其他串口/相机进程均已关闭

转动台架 R00-R03 对本次“飞手人工控制 + LOG_ONLY”是推荐的数据质量验证，不是 P00 的强制
前置。进入任何主动 RC7/PNG 控制之前，R00-R03 或等效客观外参/时延/轴向验证必须完成。R04
带桨台架怠速始终为可选项，详见 `doc/BETAFLIGHT_ROTARY_RIG_TEST_PLAN.md`。

## 4. 现场上电顺序

1. [ ] 先选择科目分支：室内预备悬停走`P00 -> F01_INDOOR`；正式室外运动学验证先拆桨执行G00。
2. [ ] 开启发射机并选择正确模型，将RC5拨到约`2000`的DISARM侧，RC7保持人工侧，油门最低。
3. [ ] 给地面 LQ AP 上电，确认两端天线已安装，再给机载 STA、Orange Pi 和飞控上电。
4. [ ] 等待接收机、飞控、相机和数传稳定；只有G00/F02等运动学科目需要等待GPS。
5. [ ] 连接功率电前再次清场；网页可用后先确认`armed=false`，不一致时不得继续。
6. [ ] 地面电脑执行 `ping 192.168.1.10`，应连续响应且无持续丢包。
7. [ ] SSH 只连接主机名为 `orangepi5max` 的当前飞机：

```bash
ssh orangepi@192.168.1.10
hostname
```

期望 `hostname` 输出：

```text
orangepi5max
```

不是该主机名时立即退出，禁止在另一台 Orange Pi 上操作。

## 5. 每个科目启动前检查

每个科目使用一个新进程和独立日志。先在 Orange Pi SSH 终端执行：

```bash
cd /home/orangepi/png_betaflight_python

pwd
test -d .git && git rev-parse --short HEAD || printf '%s\n' 'repository_commit=UNAVAILABLE'
sha256sum config/betaflight.rk3588.kinematics_log_only.example.json
ls -l /dev/ttyS1
fuser -v /dev/ttyS1
pgrep -af 'run_betaflight_log_only|bf_flight|bf_flight_png|debugd'
df -h .
```

检查结果：

- [ ] `pwd` 是 `/home/orangepi/png_betaflight_python`
- [ ] 当前板端是无`.git`导出目录，出现`repository_commit=UNAVAILABLE`属已知状态；必须依靠配置
  SHA256、meta内的源引用和PC端归档追溯版本
- [ ] 配置 SHA256 是 `b9118cc9ad0ebfb82fe0ecf2ace74f26602d6ca15bb7f0a5fd1546c5632a3c81`
- [ ] `/dev/ttyS1` 存在；`fuser` 没有显示占用进程
- [ ] `pgrep` 没有显示旧 runner、C++ 或 debugd 进程
- [ ] 磁盘可用空间大于 10 GB，Orange Pi 温度起始低于 70 C

如果配置哈希不同，不得临时修改配置继续飞；先保存差异并重新评审。

## 6. 每个科目的完整启动命令

把 `TEST_ID` 改成当前科目，例如 `G00`、`P00`、`F01`。以下命令在 Orange Pi 上执行，默认最多
运行15分钟；runner检测到有效ARM后再DISARM，会自动保留10秒尾段并正常退出：

```bash
cd /home/orangepi/png_betaflight_python

TEST_ID=G00
STAMP=$(date +%Y%m%d_%H%M%S)
RUN_ID="${TEST_ID}_${STAMP}"
LOG_DIR="logs/flight_20260831"
mkdir -p "$LOG_DIR"

python3 -u examples/run_betaflight_log_only.py \
  --config config/betaflight.rk3588.kinematics_log_only.example.json \
  --duration-s 900 --stop-after-disarm-s 10 --rate-hz 50 \
  --log-dir "$LOG_DIR" --log-prefix "$RUN_ID" \
  --control-mode log_only \
  --detector-source rknn_bytetrack \
  --isolate-rknn-process \
  --main-cpu-affinity 6,7 \
  --rknn-cpu-affinity 4,5 \
  2>&1 | tee "$LOG_DIR/${RUN_ID}_console.log"
```

终端必须看到：

```text
Control mode: log_only; allow_control=0
BF state=LOG_ONLY reason=log_only ... publish=disabled ...
```

还会打印 CSV 路径和 `Browser telemetry`。`Authorization ... approved=0`、批准文件缺失或
`Camera extrinsic ... verified=0` 在当前 LOG_ONLY 阶段可以记录后继续；这些状态禁止主动控制。
启动异常、串口打不开、相机失败或进程直接退出时，本科目取消。

终端还必须显示：

```text
MSP RAW_IMU gyro: available=1 reason=firmware_binding_match scale=0.0625 ...
```

若为`available=0`，说明固件身份或配置不一致；本轮只能保存原始gyro，不能把转换值用于交接平滑。

## 7. 启动后 30 秒验收

地面电脑打开：

```text
http://192.168.1.10:8080/
```

起飞前由操作员逐项口头报告：

- [ ] 页面持续刷新，`LIVE`，画面与时间戳持续变化且不是黑屏
- [ ] `state=LOG_ONLY`、`reason=log_only`、`publish=disabled`
- [ ] ARM 前 `armed=0`且RC5约`2000`；RC7 人工侧时`override_active=0`
- [ ] `MSP_SET_RAW_RC` attempt/write/success/ACK 全部为 0
- [ ] `sent_us` 为空，不存在 885 us 或任何算法发送帧
- [ ] camera/isolated worker/MSP/parser/Web 均无错误
- [ ] `Gyro conversion=READY / firmware_binding_match`；静止时Roll/Pitch rate接近0，转动时连续变化
- [ ] RKNN 一般约5-8 ms，tracker持续更新；无目标时应记录任何假轨迹或假`guidance_valid`
- [ ] `perception_result_age_ms` 不持续超过 120 ms
- [ ] 温度低于 75 C，电池电压与实际电池节数一致

GPS采用按科目门禁，不是所有LOG_ONLY飞行的共同前置：

- G00、F02和任何NED/速度结论：至少6星、有效fix、`Origin=LOCKED`、`State=VALID`，GPS和高度
  年龄均不超过0.5秒。
- P00、F01_INDOOR及只评价图像链路的F03-F06：允许室内`gps_fix_invalid`和
  `kinematics_valid=false`，但日志必须标记为“无运动学结论”，不得据此评价NED、速度、TTC真值
  或拦截能力。
- 卫星数本身不是充分条件；室内偶发卫星和多路径定位不得当作G00/F02通过证据。

可在地面电脑第二个终端直接查看关键 JSON：

```bash
watch -n 0.5 'curl -s http://192.168.1.10:8080/api/v1/telemetry | jq "{
  state: .safety.state,
  reason: .safety.reason,
  armed: .safety.armed,
  override: .safety.override_active,
  publish: .safety.publish_mode,
  set_attempt: .msp.commands.set_raw_rc.attempt_count,
  set_write: .msp.set_raw_rc.write_success_count,
  set_ack: .msp.set_raw_rc.ack_count,
  fix: .kinematics.gps.fix,
  sats: .kinematics.gps.satellites,
  origin: .kinematics.origin.locked,
  valid: .kinematics.valid,
  reason_kin: .kinematics.reason,
  position_ned: .kinematics.position_ned_m,
  velocity_ned: .kinematics.velocity_ned_filtered_m_s,
  gyro_deg_s: .flight_controller.gyro_deg_s,
  gyro_conversion: .flight_controller.gyro_conversion,
  log_tail_s: .run.post_disarm_tail_remaining_s,
  track: .vision.track_id,
  confirmed: .vision.tracker_confirmed,
  result_age_ms: .vision.result_age_ms,
  temperature_c: .host.thermal_max_c
}"'
```

字段为 `null` 而不是 0 时，视为版本或遥测合同不一致，不能假定为安全值。LOG_ONLY 下
`watchdog_ok=false` 或 `snapshot_approved=false` 可以出现，因为本次未申请控制；但状态仍必须是
`LOG_ONLY/disabled`。

### 7.1 室内 R_BC 与gyro动作采集方法

科目命名为`R_BC_GYRO_INDOOR`，全部拆桨、保持DISARM和RC7人工侧；无需GPS和功率电池。使用第6节
命令，但改为`--duration-s 180 --stop-after-disarm-s 0`。把真实无人机目标固定在上视相机可见位置，
不得手持目标和机体同时运动。

1. 机体保持基线10秒，记录图像中心、姿态和转换后gyro。
2. 仅抬头约10--15度，保持2秒后回基线，重复3次；再仅低头重复3次。
3. 仅向右滚转约10--15度并恢复，重复3次；再向左滚转重复3次。
4. 每次动作之间静止3秒，不改变目标、相机安装或机体航向；动作角度不要求完全一致。
5. 全部动作结束后静止10秒，让180秒定时自然退出。提前`Ctrl+C`会在meta中记录
   `keyboard_interrupt`，不能作为日志完整性通过证据。
6. 离线联合检查`gyro_*_deg_s`、`roll/pitch_deg`、`los_C`和`lambda_I`。先由动作符号确定相机三轴，
   再写显式正交`R_BC`；禁止只把`verified`改为true或用控制增益符号掩盖轴交换。

通过条件：gyro转换全程可用、抬头/低头和左右滚转各自符号稳定且回零、固定世界目标的惯性LOS
残差明显小于当前旧外参结果。未通过时继续LOG_ONLY，不生成主动控制批准文件。

### 7.2 2026-09-02 R_BC 与 Pitch 复测结果

`R_BC_GYRO_120S_OFFSET`和`PITCH_AXIS_RETEST_120S`均自然完成120秒，分别记录5975/5970行；
审计通过且所有RC写入计数为0。显式`R_BC`在抬头、低头、右滚、左滚和最终基线阶段的惯性LOS
残差为`0.874/2.294/3.419/2.986/1.884 deg`，旧`pitch_up_deg=90`相同窗口最大为
`24.515 deg`。`q_frd=d(-MSP pitch)/dt`与取反后的gyro Y在正式轮和独立复测中的相关系数分别
为`+0.964/+0.969`，确认`axis_sign=[+1,-1,+1]`中的Roll/Pitch部分。

首轮3159个新感知结果中3124个有效框，左滚时因宽高比门限出现最长1.288秒缺口；独立pitch轮
3116个新结果中3112个有效框，最长缺口0.120秒。完整指标和SHA256见
`doc/BETAFLIGHT_RBC_GYRO_VALIDATION_20260902.md`。随后20 Hz复测的9/9个有效Yaw事件积分与航向
变化符号一致，确认当前固件和安装方向使用`+Z`。硬件曝光时间戳和飞行振动仍需单独验证；此项
通过不改变`release_passed=false`，也不允许RC7主动接管。

## 8. G00 室外拆桨定位与 NED 符号

必须先拆下全部桨，DISARM，RC7 人工侧；执行第 6 节并令 `TEST_ID=G00`。

1. 将飞机静止放在无遮挡天空下，等待至少 6 星和有效 fix。
2. 继续静止，直到 `Origin=LOCKED`、`State=VALID`，然后保持 60 秒。
3. 用手机指南针确定真北附近方向，手持飞机向北走 5-10 m：
   - `Velocity N/E/D` 第一项 `v_n>0`
   - `Position N/E/D` 第一项总体增加
4. 向南返回：第一项速度变负，位置回到接近起点。
5. 向东走 5-10 m：第二项 `v_e>0`，第二项位置总体增加；向西返回时符号反转。
6. 平稳抬高约 1 m：第三项 `v_d<0`、第三项位置减小；放低时 `v_d>0`。
7. 每个方向至少重复 2 次；依据连续多帧和往返反转判断，不依据单个 GPS 抖动样本。
8. 完成后保持静止 10 秒，按第 16 节停止并归档。

通过条件：至少 6 星、运动学连续有效 60 秒、六个运动方向符号正确、MSP/相机错误和所有 RC
写入均为 0。每次重启 runner 都会重新建立 NED 原点，因此不同日志的位置不能直接拼接。

### 8.1 2026-09-01 G00 实测结果

证据包及外层 SHA256：

- `logs/outdoor_g00/G00_20260901_140100_bundle.tar.gz`：
  `6b36827d08d50e6c7c0b869b90508b2f4002dc0b66e431527937ecfe03ec70fc`
- `logs/outdoor_g00/ALTITUDE_RETEST_20260901_142540_bundle.tar.gz`：
  `66afd65e08692dd6e8711e81222c837b14f20e0b11da050ba675a6681e4d8349`

|项目|G00 主日志|高度复测|判定|
|---|---:|---:|---|
|时长/行数|846.147 s / 42033|299.987 s / 14863|完整|
|运动学有效率|99.955%|99.818%|通过|
|GPS fix/卫星数|fix 2；16-21|fix 2；20-25|通过|
|N位置范围|-10.01至7.77 m|-1.05至1.12 m|水平往返可见|
|E位置范围|-11.82至9.78 m|-1.33至0.14 m|水平往返可见|
|N/E滤波速度范围|-1.45至1.36 / -1.54至1.73 m/s|-0.24至0.39 / -0.35至0.33 m/s|双向值存在；动作标签不足|
|D位置范围|-20.55至0 m|-4.72至0.18 m|全局范围含慢漂，不能作为单次抬升量|
|滤波`v_d`/MSP vario|始终0|始终0|未通过垂直速度门禁|
|ARM/override/RC写入|0 / 0 / 0|0 / 0 / 0|通过|

高度复测包含两次约1.8 m抬升并放回，局部D/气压响应分别为1.795和1.780 m，重复差0.015 m；
全局约4.9 m范围来自DISARM静置期间慢漂，不能当成单次动作幅值。故两次动作的方向、工程量级和
重复性通过，但操作员报告的1.8 m不是同步尺/激光真值，不能宣称计量级绝对精度。该架次
Betaflight MSP altitude中的vario一直为0；后续`LOG00065`已在ARM带桨起降中补充非零`v_d`及
上下符号。两份历史证据组合后无需重做垂直F02。

## 9. 安装桨与 P00 地面怠速

正式室外分支在G00通过后停止程序、断开飞行电池并确认完全断电，再由飞手安装并复核四副桨。
室内预备分支不要求先完成GPS/G00，但只允许在既有无桨、Receiver/failsafe和P00条件均已确认后
进行，且不能记作正式运动学科目通过。重新按第4-7节上电和启动，令`TEST_ID=P00`。

1. 无目标机，人员退出桨盘和飞机前方，观察员宣布清场。
2. 操作员报告 `LOG_ONLY/disabled`、RC 写入全 0、RC7 人工侧。
3. 飞手短时 ARM，只观察正常怠速 5-10 秒，不增加油门。
4. 四电机应启动一致，无异常振动、异响、机体移动、松动或明显温升。
5. 飞手 DISARM，等待桨完全停止后断电检查桨、电机、螺钉和线缆。

ARM 后页面 `armed=1` 是正常的，但必须仍为 `state=LOG_ONLY`、`override_active=0`、
`publish=disabled`、所有 SET_RAW_RC 计数为 0。任何异常立即 DISARM；P00 不通过则不得起飞。

## 10. 每个飞行科目的通用操作

1. 每个科目都重新执行第 5-7 节，并使用对应 `TEST_ID` 启动新日志。
2. 观察员确认飞行区和计划间距；操作员报告日志状态；飞手最后决定是否 ARM。
3. 飞手人工起飞，RC7 全程保持人工侧，不允许为了“看算法效果”拨入接管。
4. 飞机进入稳定等待点后，飞手做一次轻柔、清晰的“向右 yaw 后回正”作为 CSV/Blackbox/视频
   对齐标记。观察员喊 `开始` 后才执行科目。
5. 观察员按顺序喊动作，飞手或目标机飞手完成后回答 `动作结束`；每段之间稳定 5 秒。
6. 科目结束再做一次同样的 yaw 标记，随后人工返航和落地。
7. 落地先 DISARM，程序继续记录至少 10 秒，再按第 16 节停止。
8. 首日每个核心科目后都要断电检查和离线复核，不连续飞多个科目共用一份日志。

## 11. F01 单机人工悬停基线

令 `TEST_ID=F01`，不启动目标机。

1. 人工起飞至飞手批准的安全低高度，稳定悬停 60-90 秒。
2. 做开始 yaw 标记并保持 20 秒稳定悬停。
3. 分别做小幅前/后、左/右和 yaw 正/反动作，每个方向 2 次，动作间稳定 5 秒。
4. 再稳定悬停 20 秒，做结束 yaw 标记，人工降落。

通过：人工操纵无异常，GPS/姿态/气压/Blackbox 连续，NED 静止段速度合理，CSV 与 Blackbox 可
通过 yaw 标记对齐；全程零 RC 写入。F01 只证明带桨振动和负载下采集链路工作。

### 11.1 F01_INDOOR 室内预备悬停

该科目只验证飞手人工悬停和带桨负载下的采集链路，不要求GPS。场地、飞手、观察员、遥控
failsafe、P00及人员隔离条件仍全部适用。

1. 使用独立`TEST_ID=F01_INDOOR`，无目标，RC7始终人工侧。
2. 启动后必须先确认`armed=false`、`LOG_ONLY/disabled`和全部SET_RAW_RC计数为0。
3. 飞手ARM后先保持怠速5秒；正常后飞到飞手认可的最低稳定高度，悬停10-20秒。
4. 第一轮不做方向动作，立即人工落地并DISARM；继续记录至少10秒再停止。
5. 室内`gps_fix_invalid`可以接受，但报告必须写明没有NED、速度或拦截结论。

#### 2026-08-31 实测结果

证据包：`logs/deployment_archives/F01_INDOOR_20260831_143812.tar.gz`；外层SHA256为
`5e3de1c52af182e00b72b1d123d27a089bf51bcad16620011fd6a7676371a058`，包内六个文件的
`SHA256SUMS`全部通过。

|项目|实测结果|判定|
|---|---:|---|
|总时长/行数|164.46 s / 8186|完整|
|起飞前错误ARM窗口|0.009-30.140 s，共30.13 s|启动门禁失败，起飞前已纠正|
|正式ARM窗口|76.577-146.063 s，共69.49 s|含等待、怠速和悬停|
|油门高于1050 us|112.922-145.378 s，共32.46 s|以CSV为准，不采用口头10 s|
|油门/电流/电压|max 1334 us / 6.71 A / min 24.3 V|低油门人工段|
|roll范围|-1.7至2.7 deg|未见大姿态异常|
|pitch范围|-1.7至4.4 deg|未见大姿态异常|
|结果年龄P50/P95/max|41.2/74.7/116.9 ms|满足本卡100/120 ms门限|
|RKNN P50/P95/max|6.27/6.46/18.81 ms|正常|
|姿态年龄P50/P95/max|34/72/109 ms|LOG_ONLY可用|
|状态遥测年龄P50/P95/max|264/497/572 ms|max超过0.5 s，主动控制阻断|
|主循环P50/P95/max|20/20/27 ms|50 Hz稳定|
|publish deadline miss|9|需继续统计，未发生RC发布|
|最高温度|49.9 C|正常|
|RC输出/override|全部写入0 / 0行active|通过|
|MSP/parser/camera/Web错误|全部0|通过|

包内原始审计`passed=1`、0项违规；当时的`unsupported_log_schema_version:16`来自旧审计工具。
当前`tools/analyze_betaflight_noprop_log.py`已正式支持schema 16，相同CSV复跑仍为`passed=1`、
0项违规，且不再产生该unsupported警告。该配置`motor_poll_hz=0`，因此CSV没有电机输出；后续导入的
`LOG00057.BFL`已补齐四电机、PID、gyro和高频电源证据，完整分析见
`doc/BETAFLIGHT_BLACKBOX_LOG00050_57_ANALYSIS.md`。状态遥测最大年龄572 ms超过0.5秒，进一步确认
当前单UART调度只能用于LOG_ONLY，不能据此批准实时控制。

`LOG00057`与正式ARM窗口的油门相关系数为0.999398，确认属于本架次。其28.80 s稳定悬停段四电机
raw中位为`[668,656,630,668]`、95%极差109 raw，未饱和；复用LOG00042标定约为54 us极差。
落地前最后约0.55 s则出现10.7 ms满量程电机输出、76.59 A和22.55 V瞬态，疑似触地扰动。该瞬态
不否定稳定悬停，但要求后续触地立即收油/DISARM，并始终联合检查Blackbox而非只看1 Hz网页电源值。

本轮无真实目标，但4530个新感知结果中有3202个被tracker标为confirmed，出现5个非空track ID、
4次switch和4次fragment；2987个结果因顶部裁切被拒绝。仍有3行顶部小框通过TTC/制导有效性：
其中1行score 0.50、TTC 18.59 s，另2行score 0.314、TTC 1.25/1.05 s。三行候选rate均为0，是
因为本LOG_ONLY配置的`rate_gain_matrix`为零；同时全程`LOG_ONLY/disabled`、零RC发送，因此没有
作用于飞控。这不能证明换用非零控制映射后仍无影响，反而证明“无目标时不会形成有效候选”尚未
成立。结论为：**室内人工短悬停和采集链路有条件预备通过；启动门禁需按实测RC5极性执行；正式
F01、天空负样本、目标跟踪和主动控制均未通过。**

### 11.2 THRUST_ENVELOPE 自然动作与日志分箱

该科目只测量飞手人工油门下的悬停和短时推力包线。它不测试自动油门，不要求命中精确PWM，
也不用于追求最大推力。原先按`H+50/H+100/H+150 us`定义P1/P2/P3的做法从本卡废止；这些名称
只保留为历史口令，不能再作为递增飞行步骤。

当前默认直接复用`THRUST_ENVELOPE_20260901_164023`和`LOG00062`，不要求为补齐1327、1377、
1427 us重新飞行。如专家要求重复性证据，只允许使用新`TEST_ID=THRUST_ENVELOPE_REPEAT`并执行：

1. 无目标机，RC7始终人工侧；启动后确认`LOG_ONLY/disabled`及全部SET_RAW_RC计数为0。
2. 飞手人工起飞并稳定悬停至少10秒，操作员不得要求飞手在空中查看网页或RadioMaster数值。
3. 飞手只做一次自然、平滑、小幅加油门再回到悬停，不设置油门拨动开关，不追求任何精确PWM，
   也不以达到上次1500 us为任务目标。
4. 恢复稳定至少10秒后人工落地；触地立即最低油门并DISARM，继续记录10秒。
5. 实际峰值、持续时间和响应全部以后处理日志为准。实际峰值低于计划仍是有效样本；如日志显示
   超过既有1500 us上限，本轮只归档为越界证据，不得继续下一轮。
6. 起飞前没有飞手、观察员和Blackbox，或电机、ESC、桨、机架与电池状态未复核时，不执行重复科目。

离线处理应把物理`rc_in_ch4`按实际值分箱，而不是按飞手口述赋值。当前建议区间为
`[1260,1300)`、`[1300,1350)`、`[1350,1400)`、`[1400,1450)`和`[1450,1501)` us。每箱分别
统计上升段和回落段的样本数、连续时长、比力中位/P10/P90、电流、电压、功率、四电机raw及极差；
同时估计油门到`accSmooth`的响应延迟。起飞、落地、明显横向动作和Roll/Pitch绝对值超过5度的
样本不得混入垂向推力拟合。动态分箱只能形成保守飞行包线，不能替代推力台静推力曲线。

#### 2026-09-01 LOG00062实测结果

完整分析见`doc/BETAFLIGHT_LOG00062_THRUST_ENVELOPE_ANALYSIS.md`，结构化指标见
`doc/evidence/LOG00062_THRUST_ENVELOPE_analysis.json`。主机与Blackbox油门相关系数为0.999279，
ARM时长只差0.213 s；全程`override_active=0`、SET_RAW_RC写入0。

|项目|实测|判定|
|---|---:|---|
|脉冲前悬停油门中位|1277 us|悬停候选，未写入批准配置|
|实际脉冲峰值|1500 us|相对悬停约+223 us，已超过旧P3计划|
|1500 us平台|0.407 s；中位2.371 g|短时平台成立|
|100/200 ms滑动峰值|2.448/2.427 g|不是单样本尖峰|
|超过2.0/2.25 g时长|0.531/0.330 s|短时能力成立|
|脉冲电流/电压/功率|30.52 A / 24.13 V / 739.50 W|须与硬件限值核对|
|脉冲电机最大/饱和|1119 raw / 0 s|未进入1800高值门限|
|落地末端|40.92 A、23.28 V、1985 raw|已确认正常触地瞬态；排除于推力拟合|

当前判定为“1500 us附近的短时2.4 g级比力已测得，最大推力未知”。默认不做更高油门测试；
按`2.412 kg`起飞质量换算，1500 us平台中位动态总推力估计约`56.09 N（5.72 kgf）`，
峰值约`58.57 N（5.97 kgf）`；该结果不等同于推力台静推力。本架次为`ANGLE_MODE|HORIZON_MODE`，也不能用于校准
Acro模式的`RC us -> body rate`。该结果不改变`release_passed=false`。

## 12. F02 实飞 NED 与气压速度复核

令 `TEST_ID=F02`。地面提前标记北、东和返回路线，不启动目标机。

1. 稳定悬停后向北平移 5-10 m并停稳，向南返回；重复 2 次。
2. 向东平移 5-10 m并停稳，向西返回；重复 2 次。
3. 在飞手批准高度范围内爬升约 2 m、停稳、下降返回；重复 2 次。
4. 不同时组合多个方向，不追求速度；每段动作之间稳定 5 秒。

页面和日志应满足：北移 `v_n>0`、东移 `v_e>0`、爬升 `v_d<0`，反向时符号反转；对应 NED
位置分量总体同向变化。以持续趋势判断，不以单帧判断。F02 通过才说明 NED 符号在动力振动下仍
成立。

### 12.1 历史数据复用判定（2026-09-02）

`LOG00065` 已在6S带桨人工Acro飞行中覆盖操作员确认的左、右、前、后动作。按
`|主轴速度| >= 0.5 m/s` 连续段重算，左/右/前/后分别为 `+5.11 m E`、`-6.26 m E`、
`-9.47 m N`、`+5.68 m N`，四段主方向速度样本均100%同号。自然起飞段GPS高度增加3.5 m、
`v_d`中位`-0.80 m/s`；降落段GPS高度减少4.4 m、`v_d`中位`+0.76 m/s`，上下符号正确。
主机重叠段运动学有效率100%，GPS/高度状态年龄最大242.3/250.9 ms，MSP请求错误和RC写入均为0。

2026-09-01的`ALTITUDE_RETEST`还包含两次约1.8 m抬升/放下，局部高度响应为1.795/1.780 m。
因此水平GPS/NED、近2 m垂直动作的量级与重复性、带桨上下速度符号均可跨架次复用，F02运动学
功能项记为**历史复用通过**。旧航向恢复后使用当前`R_IB.T @ v_NED`重算，左、右、前、后主轴
符号均正确，且在`184.7--190.7 deg`重建区间内不变；结合校正后四方位顺序正确和300 s
`yaw=181 deg`静态记录，`R_IB`功能符号也按组合证据关闭。不再要求水平、垂直或校正后无桨手持
移动复测。限制是当前没有校正后6S动态日志，故动力磁干扰和绝对航向误差未定量；下次正常人工
LOG_ONLY架次只需自然监测，不设置额外动作。1.8 m也不是同步外部测距真值，不能宣称计量级
绝对高度精度。本判定本身不批准PNG主动控制。

完整方法、分段指标、SHA256和机器可读结果见
`doc/BETAFLIGHT_F02_HISTORICAL_DATA_ANALYSIS_20260902.md`及
`doc/evidence/BETAFLIGHT_F02_HISTORICAL_DATA_ANALYSIS_20260902.json`。

## 13. F00天空负样本与F03-F06双机感知科目

### F00_SKY_NEGATIVE 室外无目标天空基线

真实目标入场前必须先采集天空负样本，以验证室内顶部小框是否由环境杂波造成。先做拆桨、DISARM
的地面段；F01通过后再做飞手人工悬停段。两个阶段分别使用独立日志，RC7始终人工侧：

1. `TEST_ID=F00_SKY_GROUND`：上视相机对准实际测试空域，无目标，静止记录60秒；缓慢改变机头
   朝向四次，每次稳定10秒，覆盖不同天空、光照和地面边缘进入画面的情况。
2. `TEST_ID=F00_SKY_HOVER`：人工稳定悬停60秒，无目标机；不根据网页修正姿态。
3. 记录太阳相对方位、云量、曝光、相机画面边缘遮挡和所有进入画面的非目标物。
4. 只统计`perception_new_result=1`的新结果，不能把50 Hz主循环重复行当成独立检测。

LOG_ONLY数据有效的共同门禁仍是零RC写入和零链路错误。进一步进入主动控制评审的负样本门禁为：

- 无目标段`guidance_valid=0`且`sp_valid=0`，不得出现假TTC/假制导候选；
- 不得出现持续超过0.5秒的confirmed假轨迹；所有候选必须能由边缘、面积、类别或时序门控拒绝；
- 报告confirmed占比、非空track ID数、switch、fragment、顶部/四边裁切和拒绝原因分布；
- 地面天空和悬停天空均通过，才能说明室内杂波问题未在室外复现。

#### 2026-09-01 F00_SKY_GROUND 实测结果

完整分析见`doc/BETAFLIGHT_F00_SKY_GROUND_ANALYSIS_20260901.md`，机器可读指标见
`doc/evidence/F00_SKY_GROUND_20260901_analysis.json`。三次运行使用同一配置和模型哈希，共
记录32,465行主循环数据、17,754个新感知结果和约654.3 s；三份日志审计均通过，ARM、override
和所有RC写入均为0。

|RUN_ID|新感知结果|tracked/confirmed|guidance/SP valid|结果年龄P95|现场解释|
|---|---:|---:|---:|---:|---|
|`F00_SKY_GROUND_20260901_141859`|6708|272/256|81/81|46.55 ms|持续移动小目标；未录视频，类别未确认|
|`F00_SKY_REVIEW_20260901_143149`|6447|0/0|0/0|47.12 ms|瞬时小黑点被ByteTrack拒绝|
|`F00_SKY_CONTINUE_20260901_144608`|4599|315/300|73/73|50.54 ms|操作员确认是天空中的真实飞机|

飞机事件的同一`track_id=1`跨越约13.128 s，分数0.50-0.763，未发生ID switch，并进入了
guidance/SP评估；因`LOG_ONLY/disabled`没有作用于飞控。该事件排除了“建筑边缘静态假纹理”的
解释，但暴露出单类UAV模型不能抑制非目标航空器。当前判定拆分为：

- **F00-A静态天空/建筑背景：当前样本通过**；
- **瞬时小目标时序拒绝：通过**；
- **非目标航空器类别抑制：未通过**；
- **四航向覆盖：未完成**，有录像的复测仅明确完成一次约72 deg转向；
- **完整F00和主动控制放行：未通过**，还缺`F00_SKY_HOVER`、非目标航空器抑制、状态源和闭环
  安全放行；相机外参已在后续2026-09-02专项试验中关闭，不再是当前阻塞项。

飞机样本必须作为困难负样本保留，不能仅用最小框面积阈值删除；远距离真实无人机可能具有相同
面积。应在取得F03/F04真实无人机数据后再比较类别、轨迹和时序特征，不得现场改阈值继续飞。

F00失败不妨碍飞手人工执行F03-F06以继续收集LOG_ONLY数据，但必须保持
`release_passed=false`，不得启动RC7接管。

双机不得形成碰撞航线。目标机应高于上视相机，但必须同时保留预先批准的水平和垂直间距；不得
从拦截机正上方近距通过。两机在分离的起降区起飞，先分别进入等待点，再由观察员允许进入测试
几何。

### 13.1 无 GPS 厂房内分支

F03/F04的检测、ByteTrack、图像中心、LOS符号、结果年龄和出框失效指标可以在无GPS厂房内先行
采集。必须使用独立`TEST_ID=F00_FACTORY_NEGATIVE`、`F03_INDOOR`和`F04_INDOOR`，不能把结果
直接登记为室外正式科目。无GPS时`origin_locked=0`、`kinematics_valid=0`属于预期，只允许得出
纯视觉结论，不评价NED、真实闭合速度、地理边界或TTC物理精度。

1. 先做`F00_FACTORY_NEGATIVE`：无目标，地面和人工悬停各记录60 s，确认屋顶、钢梁、吊车、
   灯具及LED闪烁不会形成持续confirmed轨迹。
2. 可先做降风险预检：拦截机拆桨固定，真实目标机或等比例目标安全悬挂/移动；该结果不计正式
   F03/F04，因为没有拦截机动力振动和姿态扰动。
3. 正式`F03_INDOOR`需要拦截机和目标机分别由两名飞手人工悬停，另设安全观察员；目标机保持
   更高但有明确侧向偏置的位置，不得位于拦截机正上方。
4. 正式`F04_INDOOR`由拦截机人工定点、目标机沿错开的平行通道低速横穿；路线不得迎面、追尾
   或形成碰撞线。
5. 厂房高度、宽度、照明、人员隔离和两机间距任一不足时，只做地面预检，不同时飞两架飞机。

厂房结果可作为进入室外F03/F04前的感知预检，但室外天空、太阳、风、远距离尺度和目标外观仍需
单独覆盖。全程仍为`LOG_ONLY`，GPS缺失不是纯视觉终止条件；MSP姿态、相机、时间戳、人工遥控或
安全间距异常仍须立即终止。

### F03 静止目标跟踪

令 `TEST_ID=F03`。

1. 拦截机由飞手人工定点；目标机进入已批准的上方偏置等待点。
2. 目标机稳定悬停 3 段，每段 20 秒，段间改变少量位置但不缩小批准间距。
3. 拦截机飞手只做保持，不追踪目标，不根据网页修正飞行。

通过：连续可见段confirmed比例至少95%，同一连续段ID switch为0，结果年龄P95小于100 ms，
最大值不持续超过120 ms；并且相对F00负样本能明确区分“真实目标有效候选”和“无目标拒绝”。

### F04 目标横穿

令 `TEST_ID=F04`。

1. 拦截机人工定点，目标机按与碰撞线错开的平行路线低速左到右横穿 3 次。
2. 再按相同路线右到左横穿 3 次，每次之间回到等待点并稳定 5 秒。
3. 观察员持续确认两机水平和垂直间距。

通过：图像横向中心和 LOS 符号随横穿方向反转；连续可见段无持续 ID 跳变；出框时数据明确
失效而不是继续使用旧目标。

### F05 距离和尺度变化诊断

令 `TEST_ID=F05`。

1. 两机保持足够侧向偏置，目标机沿非碰撞路线缓慢接近，再远离，共 3 组。
2. 不进入近距、不追求高速闭合；拦截机仍由飞手人工保持。
3. 只观察框面积、LOS、尺度变化率和 TTC 拒绝原因，不执行拦截。

通过：接近时 bbox 面积总体增大，远离时总体减小；无明显身份误切；TTC 有效/无效原因和尺度
变化一致。该结果不能证明真实碰撞航线下的 TTC 精度。

### F06 视场边缘、短时丢失与重捕获

令 `TEST_ID=F06`。

1. 目标机缓慢移动到画面左、右、前、后边缘后返回，每个方向 2 次。
2. 在安全分离航线中完全离开视场 1-2 秒后重入，共 3 次。
3. 每次重入后稳定 10 秒，记录是否重新 confirmed 和 track ID 是否变化。

通过：出框期间不输出旧目标测量；重入有明确 reacquire/track 事件；连续可见段稳定。出框后
重新分配 track ID 可以接受，但必须在 events 和报告中明确，不能记为同一连续跟踪段。

## 14. 实时监看与立即终止条件

操作员只监看网页和终端，飞手只看飞机。以下任一项出现，操作员喊 `立即终止`，目标机先退出
冲突区域，拦截机由飞手人工降落；不得在空中 SSH 重启或改配置：

- 页面离开 `LOG_ONLY`，`publish` 不再是 `disabled`，或任一 SET_RAW_RC 计数不为 0
- RC7/override 意外生效、飞行模式异常、遥控丢帧或飞手感觉操纵异常
- runner 退出、终端报错、页面冻结、时间戳不再增加或 LQ 链路中断
- G00/F02等运动学必选科目中GPS fix丢失、运动学无效或GPS/高度数据陈旧持续超过0.5秒；
  F01_INDOOR、F00_FACTORY_NEGATIVE、F03_INDOOR和F04_INDOOR等纯视觉科目只记录该状态，
  不把它误报为飞控失效
- 相机黑屏、worker/MSP/parser/checksum 持续报错或感知年龄持续超过 120 ms
- 目标身份明显误切、两机低于批准间距或进入非计划闭合航线
- 人员进入飞行区、观察员或任一飞手要求终止、天气/空域条件变化
- 低电压、异常电流、动力下降、异常振动/声音、硬件松动或温度达到 75 C
- 推力复验中飞手需要低头找精确油门数值、出现非计划大幅爬升，或落地后未能立即最低油门/DISARM

程序在飞行中退出不会自动接管飞机；飞手继续人工控制并立即安全降落。

## 15. 当日建议顺序

室内预备分支按下列顺序执行：

1. `P00`：带桨地面怠速
2. `F01_INDOOR`：首次10-20秒短悬停；通过后才可另架次扩大到60秒
3. `F00_FACTORY_NEGATIVE`：厂房屋顶和灯具无目标负样本
4. `F03_INDOOR`：双机安全偏置静止目标；两名飞手加一名观察员
5. `F04_INDOOR`：双机错开通道低速横穿；场地不足时停止在地面预检

室内F03/F04只形成纯视觉证据，不替代室外正式科目，也不要求或证明GPS/NED。只有一名飞手、
无法划定两条隔离飞行通道或无法保证目标机不在拦截机正上方时，不得执行双机室内飞行。

正式室外分支按下列顺序执行，前一项未通过不得扩大安全包线：

1. `G00`：拆桨GPS、原点和NED手持符号
2. `F00_SKY_GROUND`：拆桨无目标天空负样本
3. `P00`：现场重新检查带桨地面怠速
4. `F01`：无目标单机人工悬停，同时形成带桨天空基线
5. `THRUST_ENVELOPE_REVIEW`：默认只离线复核既有LOG00062，不新增飞行；专家要求时才执行自然动作复验
6. `F02`：单机NED方向与气压速度
7. `F00_SKY_HOVER`：若F01没有完整60秒稳定负样本则单独补采
8. `F03`：双机静止目标
9. `F04`：双机非碰撞横穿
10. `F05`：可选尺度变化
11. `F06`：可选边缘、丢失和重捕获

如果当日时间或电池不足，优先完成G00、F00_SKY_GROUND、P00、F01、F02和F03；不要通过缩短
复核或减小双机间距赶进度。2026-09-01的F00-A静态背景结果可以支持继续人工LOG_ONLY采集，但
飞机非目标事件、四航向覆盖和F00_SKY_HOVER仍未关闭；后续F03-F06不得宣称主动控制就绪。
推力包线不得为了“完成P2/P3”临时增加架次；现有LOG00062已经越过旧P3计划值，后续优先分析
既有平滑过渡数据和横向人工动作，而不是继续增加油门。

## 16. 落地、停止程序与日志归档

1. [ ] 目标机先退出工作区；拦截机人工落地并 DISARM。
2. [ ] 确认桨完全停止，RC7保持人工侧；不要按`Ctrl+C`，等待10秒尾段完成并自动返回shell。
3. [ ] 若DISARM后12秒仍未退出，按一次`Ctrl+C`并把本架次标记为“尾段自动退出失败”；保存现有文件，
   后续定位runner阻塞，不得删除或重跑覆盖证据。
4. [ ] 确认runner已退出、串口已释放；以下两个命令均不应显示占用进程：

```bash
pgrep -af 'run_betaflight_log_only|bf_flight|bf_flight_png|debugd'
fuser -v /dev/ttyS1
```

5. [ ] 不要直接断 Orange Pi 电源；先确认 CSV、meta 和 events 已写完。
6. [ ] 执行以下命令定位本架次文件：

```bash
cd /home/orangepi/png_betaflight_python

CSV=$(ls -1t "$LOG_DIR"/${RUN_ID}_*.csv | head -1)
META="${CSV%.csv}_meta.json"
EVENTS="${CSV%.csv}_events.jsonl"

ls -lh "$CSV" "$META" "$EVENTS" "$LOG_DIR/${RUN_ID}_console.log"
jq '.completion' "$META"
tail -n 3 "$EVENTS"
```

正常飞行架次的meta必须显示`stop_reason=post_disarm_tail_complete`、`complete=true`、
`post_disarm_tail_completed=true`，CSV末段必须持续记录`armed=0`。`keyboard_interrupt`或`unknown`
只能归档为不完整日志，不能关闭LOG00065暴露的25.6秒缺口问题。

7. [ ] 运行日志合同审计：

```bash
python3 tools/analyze_betaflight_noprop_log.py --csv "$CSV"
AUDIT="${CSV%.csv}_audit.json"
jq '{passed, violations, warnings, metrics}' "$AUDIT"
```

`passed=1` 是最低要求；它主要审计日志、MSP 和 RC 输出合同，不能代替 GPS、跟踪和飞行科目人工
复核。任何 warning 都必须记录，不能静默忽略。

8. [ ] 计算本架次文件哈希：

```bash
sha256sum "$CSV" "$META" "$EVENTS" "$AUDIT" "$LOG_DIR/${RUN_ID}_console.log" \
  > "$LOG_DIR/${RUN_ID}_SHA256SUMS"
```

9. [ ] 断开飞行电池后检查电机/电调温度、桨、紧固件、电池和线缆；异常时封存该架次。
10. [ ] 从 Betaflight 导出原始 `.BFL`，不得只保留 Configurator 转换的 CSV。
11. [ ] 保存目标机 GPS/Blackbox、地面视频、科目口令时间、风、间距、速度和异常说明。
12. [ ] 将 Orange Pi 的 CSV/meta/events/audit/console/SHA256SUMS 与对应 `.BFL` 放入同一架次目录。

在PC完成归档后，用相同提交的`blackbox_decode --unit-rotation raw`生成Blackbox CSV，再执行通用
人工飞行对齐分析。该工具不要求RC7接管或algorithm区间；`HOST_CSV`、`BLACKBOX_CSV`和`BFL`必须
属于待配对候选，工具会在全部ARM窗口中按时长筛选并用油门波形确认：

```bash
python3 tools/analyze_betaflight_blackbox_flight.py \
  --host-csv "$HOST_CSV" \
  --blackbox-csv "$BLACKBOX_CSV" \
  --blackbox-bfl "$BFL" \
  --decoder-commit "$BLACKBOX_DECODER_COMMIT" \
  --output "${HOST_CSV%.csv}_blackbox_flight.json"
```

必须检查`selection.throttle_alignment.correlation`、`duration_match_error_s`、稳定段电机分布和
`endpoint_transients`。没有油门变化的纯怠速航段只能退化为ARM起点对齐，不能宣称完成精确配对；
跨架次复用LOG00042电机raw到us标定时，必须显式提供`--motor-scale-us-per-raw`和
`--motor-offset-us`并在报告中注明是继承标定。

`THRUST_ENVELOPE`使用相同工具，但必须显式声明日志切分阈值和Blackbox固件头中的
`acc_1G`。以下命令只识别实际发生的脉冲，不要求飞手事先命中阈值：

```bash
python3 tools/analyze_betaflight_blackbox_flight.py \
  --host-csv "$HOST_CSV" \
  --blackbox-csv "$BLACKBOX_CSV" \
  --blackbox-bfl "$BFL" \
  --decoder-commit "$BLACKBOX_DECODER_COMMIT" \
  --motor-scale-us-per-raw 0.499231573 \
  --motor-offset-us 977.151031 \
  --thrust-pulse-threshold-us 1350 \
  --thrust-plateau-threshold-us 1495 \
  --acc-1g-raw 2048 \
  --thrust-hover-window-s 8 \
  --thrust-hover-gap-s 2 \
  --thrust-post-delay-s 1 \
  --output "${HOST_CSV%.csv}_thrust_envelope.json"
```

阈值用于离线选段，不是下一架次的飞行目标。必须联合检查`thrust_envelope.windows`、
`endpoint_transients`和原始曲线；再按第11.2节的实际油门区间分箱，并把上升/回落和落地段分开。

## 17. 数据有效性判定

所有科目的共同硬门禁：

- 全程 `control_mode=log_only`、`allow_control=0`、`publish=disabled`
- 全部 SET_RAW_RC attempt/write/success/ACK 为 0，无 885 us 发送帧
- RC7 全程人工侧；ARM 状态与飞手实际操作一致
- CSV、meta、events、audit、console、SHA256SUMS 和原始 `.BFL` 齐全
- MSP/parser/checksum/camera/worker 错误为 0，时间戳单调
- G00/F02等运动学科目的GPS/气压数据未持续陈旧；纯视觉科目明确标注无运动学结论
- 无目标段任何confirmed、`guidance_valid`或`sp_valid`假候选都已统计，不得因最终零发送而忽略
- 飞手、观察员和操作员均确认无未记录异常
- 推力包线报告使用Blackbox固件头的`acc_1G`，区分脉冲与落地，并报告实际油门而非计划油门

F01/F02 可验证带桨状态链路和 NED 符号；室内F03/F04只验证无GPS条件下的视觉检测、跟踪和图像
LOS，室外F03-F06才覆盖真实天空、风和远距离尺度下的YOLO+ByteTrack、LOS、TTC/VM候选诊断。
它们都不验证RC映射、飞控响应、PNG主动控制稳定性或拦截命中率。飞机等非目标航空器必须与真实
无人机目标分开统计，不能因画面中确有物体就自动记为F00通过。只有后续完成外参、时延、控制
边界、推力/悬停包线和独立安全批准后，才能制定主动控制测试卡。

## 18. 架次记录表

|科目|RUN_ID / CSV|电池|开始/结束时间|GPS/跟踪摘要|审计/Blackbox|结果或终止原因|
|---|---|---|---|---|---|---|
|G00|`G00_20260901_140100`；`ALTITUDE_RETEST_20260901_142540`|拆桨；电源见CSV|846.15 s；299.99 s|fix 2，16-25星；两次约1.8 m抬升响应1.795/1.780 m；DISARM时`v_d=0`|两份审计通过；证据包已归档|高度方向、工程量级和重复性通过；无同步外部高度真值|
|F00_SKY_GROUND|`F00_SKY_GROUND_20260901_141859`；`F00_SKY_REVIEW_20260901_143149`；`F00_SKY_CONTINUE_20260901_144608`|拆桨；电源见CSV|239.86 / 239.97 / 174.46 s|静态背景通过；瞬时目标被拒绝；飞机13.13 s confirmed并产生73个制导候选|三份审计通过；报告及录像已归档|F00-A背景子项通过；非目标航空器和四航向未关闭|
|P00|||||□通过 □失败|怠速|
|F01_INDOOR|F01_INDOOR_20260831_143812|主机25.0至24.3 V；BFL瞬态22.55 V|164.46 s主机/69.13 s BFL|GPS无效；室内假检严重|审计通过；LOG00057已对齐|□人工悬停通过；落地瞬态待复核|
|F01|||||□通过 □失败||
|THRUST_ENVELOPE|`THRUST_ENVELOPE_20260901_164023`|2.412 kg；6S 10000 mAh；3115-900KV；四副10x5.0三叶桨|178.46 s ARM；1.031 s脉冲|悬停1277 us；1500 us平台2.371 g；估计56.09 N|LOG00062相关系数0.999279；脉冲无电机饱和|短时上限已测；落地峰值已确认正常且排除于拟合；不做旧P2/P3递增|
|R_BC_GYRO_INDOOR|`R_BC_GYRO_120S_OFFSET`；`PITCH_AXIS_RETEST_120S`；`YAW_AXIS_20HZ_RETEST2_90S`|拆桨；DISARM|120.26 / 120.27 / 89.98 s|R_BC各姿态残差<5 deg；Pitch相关+0.969；Yaw事件9/9同号、相关0.994|三份审计通过；RC写入0|外参和Roll/Pitch/Yaw轴向通过；曝光时刻未关闭|
|VISION_GUIDANCE_LATENCY|`VISION_GUIDANCE_LATENCY_HANDHELD_120S_20260902_145714`|拆桨；DISARM|119.99 s|有效框链路年龄P50/P95/P99/max=56.8/85.3/99.8/116.8 ms；Track 14/31有效率97.80%/99.58%|审计通过；RC写入0；MSP错误0|软件链路P95门槛通过；硬件曝光和命令响应未测|
|F02历史复用|`LOG00065`；`ALTITUDE_RETEST_20260901_142540`；四航向及校正后静态日志|6S带桨飞行+拆桨高度/航向复测|55.81 s BFL；299.998 s高度；300 s航向|左/右/前/后=`+E/-E/-N/+N`且主方向100%同号；`R_IB`回放主轴均正确；两次1.8 m响应1.795/1.780 m；带桨`v_d`符号正确|Blackbox/主机已对齐；相关LOG_ONLY日志零RC写入|功能通过；水平/垂直/无桨`R_IB`均不重测；动力磁干扰与计量级高度精度未量化|
|F00_SKY_HOVER|||||□通过 □失败|无目标天空|
|F00_FACTORY_NEGATIVE|||||□通过 □失败|无GPS厂房负样本|
|F03_USB_BENCH|`F03_USB_HANDHELD_STATIC_20260902_205100`|USB；拆桨；DISARM|179.97 s|静止区间confirmed 99.90%；单一ID；P95 145.79 ms|审计通过；RC写入0；MSP错误0|△跟踪通过；100 ms时延门槛失败；不替代双机F03|
|F04_USB_BENCH|`F04_USB_HANDHELD_CROSSING_RETRY_20260902_205751`|USB；拆桨；DISARM|123.82 s|6次跨区；`corr(x,lambda_I_y)=-0.9989`；首趟ID重建；P95 156.69 ms|审计通过；RC写入0；MSP错误0|△运动/LOS/出框失效通过；连续性及时延失败；不替代双机F04|
|VM_FINAL_CHAIN_INDOOR|`VM_FINAL_CHAIN_INDOOR_120S_20260902_214423`|USB；拆桨；DISARM|119.99 s|静止20 s全有效；水平连续段3次反转且ID=2；一次纵向往返；P95 84.56 ms|审计通过；RC写入0；MSP/RKNN错误0|室内实现链路通过；不证明动力闭环或拦截命中|
|INTERCEPT_HW_MC100|离线3000条；2场景x15工况x100 seeds|不适用|40 s/条仿真|总体命中96.92%/97.31%；FOV可行89.38%/86.54%|归档及逐文件SHA256已生成|初期80%聚合性能目标通过；完整release失败；未测实机命中率|
|F03_INDOOR|||||□通过 □失败|纯视觉，不替代室外F03|
|F04_INDOOR|||||□通过 □失败|纯视觉，不替代室外F04|
|F03|||||□通过 □失败||
|F04|||||□通过 □失败||
|F05|||||□通过 □失败|可选|
|F06|||||□通过 □失败|可选|

现场异常与处理：

```text
时间：________________
科目/动作：________________
页面/终端现象：________________
飞手处置：________________
对应 CSV/BFL/视频：________________
是否允许继续：□否  □经全体复核后继续
```
