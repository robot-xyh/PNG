# 2026-09-04 Betaflight 外场主动接管实验归档与审计

## 1. 归档结论

2026-09-04 外场数据包含 13 组 Orange Pi 主 CSV、1 次仅有控制台的启动失败、13 个拦截机
Blackbox 文件，以及两段覆盖主要科目的靶机 PX4 ULog。依据绝对时间、ARM 上升沿和 ARM 段长度，
可以把 12 个 Blackbox ARM 段唯一配对到主机日志；`LOG00097.BFL` 位于两个主机进程之间，不能
强行配对。9 组主机日志真正进入过 `publish=algorithm`，其中一组有两次脉冲，共 10 次算法发布
脉冲；按 CSV 首末算法样本计算，累计跨度约 `7.977 s`。

最后一架次形成目前证据最完整的闭环链：Orange Pi 主日志、拦截机 `LOG00106.BFL`、靶机
`10_35_19.ulg` 和双方冲击特征均可对齐。PNG 实际发布 `1.670804 s`，停止发布后
`0.902322 s` 发生物理接触。接管窗口内靶机速度 P95 为 `0.056 m/s`，可称为“主要闭合阶段
平移基本静止”，但主冲击前约 `0.27 s` 靶机飞手已经给出左滚输入，因此不能描述为末端始终完全
无操作。

最后一架次证明了单次静止目标场景中的识别、导引、MSP 速率/油门执行、闭合和物理接触。它不
证明真实命中率达到 `80%`，也不证明对横移、规避目标有同等能力。当天全部算法发布行中，总加速
度限幅占 `351/406=86.5%`，速度建立项限幅占 `312/406=76.8%`；当前控制器大部分时间处于
限幅区，近距静止目标配合 `fixed_vm=10 m/s` 的响应明显偏激进。

## 2. 数据范围、时间基准与场景

### 2.1 时间范围

|数据源|记录范围|时间基准|来源|
|---|---|---|---|
|Orange Pi 主机|`2026-09-04 17:55:37 +0800` 至约 `18:38:15.894 +0800`|meta 的 `created_unix_s` 加 CSV `elapsed_s`|`logs/flight_active_supervised/*_meta.json` 与同名前缀 CSV|
|拦截机 Blackbox|`09:58:53.984--10:38:05.641 UTC`，即 `17:58:53.984--18:38:05.641 +0800`|BFL 头 `Log start datetime` 加 `time (us)`|`logs/blackbox_import/LOG00094.BFL` 至 `LOG00106.BFL`|
|靶机连续段|`10:03:35.027--10:22:33.888 UTC`|PX4 GPS UTC 与 ULog 单调时钟中位偏移|`logs/target-log/10_03_35.ulg`|
|靶机最后段|`10:35:19.327--10:38:40.479 UTC`|同上|`logs/target-log/10_35_19.ulg`|

本工作站时区是 `America/Los_Angeles`，所以 `find`/`stat` 显示的文件 mtime 不是现场时钟。配对
不得使用当前工作站显示的 mtime，应使用上述日志内部时间。报告中的现场时刻默认采用北京时间
`UTC+8`；需要跨设备计算时保留 UTC。

### 2.2 统一场景和运行参数

|项目|外场实际配置|来源|
|---|---|---|
|拦截机|Betaflight `25.12.2`，MICOAIR743V2，6S，四通道 MSP OVERRIDE|各 `_meta.json` 的 `fc_identity`、`fc_configuration`、`msp_runtime`|
|靶机|PX4 `v1.16.2`，Position Control；最后一架次为静止目标科目|`10_35_19.ulg` 和 `logs/analysis/LOG00106_target_joint/metrics.json`|
|相机|上视固定相机，输出 `640 x 512`；`R_BC=[[0,1,0],[1,0,0],[0,0,-1]]`，标记为 verified|各 `_meta.json` 的 `camera_calibration`|
|检测器|`drone_v8n_v21_kd_relu_lambda008_640_640-rk3588.rknn`，SHA256 `ad905c19...10c60f59`，30 Hz，ByteTrack|各 `_meta.json` 的 `detector`|
|导引律|`velocity_establishing_png`，`N=3`，`fixed_vm=10 m/s`，`max_guidance_accel=7 m/s2`|各 `_meta.json` 的 `guidance`|
|执行映射|`accel_tilt_rate`；Roll/Pitch 上限 `60 deg/s`，倾角硬限 `35 deg`|各 `_events.jsonl` 的 `run_start`|
|控制权限|mask `15`：Roll/Pitch/Throttle/Yaw；Yaw 固定 `1500 us`|各 `_meta.json` 的 `flight_profile`、`msp_runtime`|
|油门|允许参考 `1200--1400 us`，发送绝对上限 `1500 us`，交接 `0.8 s`|各 `_meta.json` 的 `msp_runtime.config`|
|接管时限|软件无时长上限，完全依赖现场飞手退出 RC7|各 `_meta.json` 的 `flight_profile.max_takeover_duration_s=null`|
|MSP|单 UART 异步流水线，目标发布 50 Hz|各 `_meta.json` 的 `msp_runtime.config`|
|配置绑定|配置 SHA256 `4032482c47496c89d52f3cb09bfcef04b6c028de9012d10433d0aed4751865ff`|全部 `_meta.json`|

文件名前缀中的 `05S` 不是有效的软件限时；配置明确为无时长上限。`VIDEO` 也只表示当时启用了
网页预览，目录内没有 MP4/AVI/MKV 等录像文件，不能把该前缀当作场景视频证据。

## 3. 原始文件索引

### 3.1 Orange Pi 主机记录

每个正常前缀包含 `.csv`、`_meta.json`、`_events.jsonl` 和对应 `_console.log`。CSV 是逐行审计的
主记录，meta 固化配置和设备身份，events 是状态变化摘要，console 仅用于启动/异常诊断。

|现场启动前缀|CSV 字节/行数/跨度|原始结果|
|---|---:|---|
|`FLIGHT_ACTIVE_05S_20260904_175535_20260904_175537`|`14,186,659 B / 5108 / 103.115 s`|未 ARM；RC7 高位时 prefill 未就绪；异常终止|
|`FLIGHT_ACTIVE_05S_20260904_175723`|无 CSV|相机设备打开失败，仅 console|
|`FLIGHT_ACTIVE_05S_20260904_175800_20260904_175802`|`25,336,552 B / 8697 / 175.394 s`|人工飞行，无目标、无接管|
|`FLIGHT_ACTIVE_05S_20260904_180433_20260904_180434`|`10,394,698 B / 3592 / 72.730 s`|未 ARM、无目标、无接管；异常终止|
|`FLIGHT_ACTIVE_05S_VIDEO_20260904_180552_20260904_180553`|`27,329,301 B / 8609 / 173.906 s`|两次 ARM；多次 RC7，但始终 `aux_disabled`，无算法发布|
|`FLIGHT_ACTIVE_05S_VIDEO_20260904_180918_20260904_180919`|`15,704,705 B / 5030 / 101.764 s`|一次有效算法接管|
|`FLIGHT_ACTIVE_05S_VIDEO_20260904_181142_20260904_181144`|`19,861,175 B / 6299 / 127.417 s`|两次很短算法脉冲，均由 `aux_disabled` 结束|
|`FLIGHT_ACTIVE_05S_VIDEO_20260904_181433_20260904_181434`|`10,292,199 B / 3295 / 66.396 s`|一次算法接管，目标机在移动|
|`FLIGHT_ACTIVE_05S_VIDEO_20260904_181616_20260904_181617`|`10,008,489 B / 3218 / 65.165 s`|一次算法接管，目标机在移动|
|`FLIGHT_ACTIVE_05S_VIDEO_20260904_181801_20260904_181802`|`8,112,682 B / 2613 / 52.746 s`|一次算法接管，由目标失效结束|
|`FLIGHT_ACTIVE_05S_VIDEO_20260904_181911_20260904_181913`|`11,006,142 B / 3592 / 72.593 s`|约 0.10 s 算法脉冲|
|`FLIGHT_ACTIVE_05S_VIDEO_20260904_182055_20260904_182057`|`11,835,382 B / 3719 / 75.107 s`|约 0.08 s 算法脉冲，感知年龄偏高|
|`FLIGHT_ACTIVE_05S_VIDEO_20260904_183459_20260904_183502`|`13,059,239 B / 4291 / 86.730 s`|约 0.34 s 算法脉冲|
|`FLIGHT_ACTIVE_05S_VIDEO_20260904_183721_20260904_183722`|`8,060,890 B / 2645 / 53.342 s`|最后一次算法接管并发生物理接触|

主 CSV 的 SHA256 可由第 10 节命令重算。最后一组主 CSV 的固定 SHA256 为
`2f4cba58655be4d142237fc5e05c7ad16b0c3c9131a838d1413e9676cdc557a6`。

### 3.2 拦截机 Blackbox

|BFL|BFL 起始 UTC / 时长|字节|SHA256|
|---|---|---:|---|
|`LOG00094.BFL`|`09:58:53.984 / 113.587562 s`|`2,959,721`|`f21df6ed39208f3a9f8068906f85b1c9804c84f35d6eecd93479961503403161`|
|`LOG00095.BFL`|`10:06:31.737 / 83.407057 s`|`2,244,985`|`7277c52813fc2b15107e1daa9d852dc2a5b74b80cb7fb8b4eb2d5d9de04c7cd8`|
|`LOG00096.BFL`|`10:07:58.789 / 38.832666 s`|`1,057,188`|`1c5e425d4340aa3cc1c43d2ccaaf00d57ce10ab2db936366c8afeca5db8428a8`|
|`LOG00097.BFL`|`10:08:59.651 / 15.624079 s`|`430,524`|`cfa07c229a00aa7e3bfcf19ee7c9472a7b6684a296992bc734ef8a1ca6bb5ddc`|
|`LOG00098.BFL`|`10:09:41.636 / 69.972244 s`|`1,891,293`|`4a3eecdd56cef345ff6c601d2fb11de9016e94c3c4339ee936a243d0fd852337`|
|`LOG00099.BFL`|`10:12:18.341 / 83.173680 s`|`2,238,990`|`9720a8887757e06995d72a3bd3c658dcfdb8fb8397e60a5b6c810d2b8db77928`|
|`LOG00100.BFL`|`10:14:46.505 / 44.610516 s`|`1,207,653`|`e07b90622400bacb74f1f23b225eee9c34e8b986b474970315434ec4ea5ce0f4`|
|`LOG00101.BFL`|`10:16:29.082 / 43.904583 s`|`1,185,083`|`634f725f6bfddf84f46c5d358f0571416c53ef80055e301458555e94bfcc8699`|
|`LOG00102.BFL`|`10:18:11.398 / 33.943706 s`|`919,835`|`134884fa48fe8aee369122ccb6224b8180d74e8478c5858a1117789b2559d97a`|
|`LOG00103.BFL`|`10:19:39.876 / 35.779354 s`|`965,647`|`f127e301f980e11968ff557a022c0d5783e72559ab4e1171d348d8a6370cbdc1`|
|`LOG00104.BFL`|`10:21:08.947 / 53.270275 s`|`1,433,911`|`eefc4f84c56c2732d9ac269bb6ad8137d287f9575b2ade3d7dda43ef3f4b2918`|
|`LOG00105.BFL`|`10:35:34.983 / 44.091620 s`|`1,185,212`|`29e3dd7d7f29de4ff538a9b8dde6546314852aa5b965a5fb004482cea40d5345`|
|`LOG00106.BFL`|`10:37:42.679 / 22.962053 s`|`651,894`|`fc58d049df776a6a771f312ec5ad71bbd96763b5db7ed60f1b00116f6ed748ec`|

### 3.3 靶机与分析产物

- `logs/target-log/10_03_35.ulg`：`46,185,506 B`，SHA256
  `ad539c391a97982ea3428f859dcf0c7f37a24981a310309c53a7ceae08fbbed5`；连续覆盖
  `10:03:35.027--10:22:33.888 UTC`，19 次 dropout、合计 `3.74 s`。它覆盖
  `LOG00095--LOG00104` 时段，但不是逐架次独立文件。
- `logs/target-log/10_35_19.ulg`：`8,563,174 B`，SHA256
  `366fbea3ab9d322efe7e597161a32ad48e1caaf3a627b9bed990f715ced7ec96`；连续覆盖
  `10:35:19.327--10:38:40.479 UTC`，无 dropout，覆盖 `LOG00105/106`。
- 最后一架次联合指标：`logs/analysis/LOG00106_target_joint/metrics.json`。
- 最后一架次联合事件：`logs/analysis/LOG00106_target_joint/joint_event_timeline.csv`。
- 最后一架次 50 Hz 联合序列：`logs/analysis/LOG00106_target_joint/joint_timeseries_50hz.csv`。
- 已有专题报告：`doc/BETAFLIGHT_FLIGHT_ACTIVE_LOG00106_COLLISION_ANALYSIS.md`、
  `doc/BETAFLIGHT_LOG00106_EXPECTED_ACTUAL_RESPONSE_REPORT.md`。
- 已有图：`doc/figures/log00106_control_response/` 与
  `logs/analysis/LOG00106_target_joint/joint_*_timeline.png`。

## 4. 配对审计

### 4.1 唯一配对

主机 ARM 预测 UTC 使用 `created_unix_s + elapsed_s`。下表的差值是 BFL 起始时刻减主机 ARM
上升沿；所有差值均小于 `0.226 s`，且 BFL 时长和对应 ARM 段长度闭合，因此不是仅凭文件名猜测。

|主机运行/ARM 段|配对 BFL|BFL 相对主机 ARM|配对判定|
|---|---|---:|---|
|`175800`|`LOG00094`|`+70.0 ms`|唯一；单 ARM 段且时长闭合|
|`180552` 第 1 段|`LOG00095`|`+128.6 ms`|唯一；第一 ARM 段|
|`180552` 第 2 段|`LOG00096`|`+142.0 ms`|唯一；第二 ARM 段|
|`180918`|`LOG00098`|`+124.6 ms`|唯一|
|`181142`|`LOG00099`|`+170.8 ms`|唯一；一个 ARM 段内两次算法脉冲|
|`181433`|`LOG00100`|`+126.9 ms`|唯一|
|`181616`|`LOG00101`|`+225.9 ms`|唯一|
|`181801`|`LOG00102`|`+162.7 ms`|唯一|
|`181911`|`LOG00103`|`+194.2 ms`|唯一|
|`182055`|`LOG00104`|`+143.1 ms`|唯一|
|`183459`|`LOG00105`|`-106.2 ms`|唯一|
|`183721`|`LOG00106`|`-155.6 ms`|唯一；另有双机冲击锚点|

### 4.2 仅时间覆盖和不可配对项

- 没有把任何拦截机 BFL 标成“仅时间近似配对”。已接受的 12 项都有 ARM 边沿和时长双重证据。
- `LOG00097.BFL` 从 `10:08:59.651 UTC` 开始，处于 `180552` 主机日志结束后、`180918`
  主机日志开始前。它可能是一次人工 ARM/短飞，但无同期主机 CSV，归档为**不可评价 PNG**。
- `10_03_35.ulg` 是靶机连续长日志，与多架次按 UTC 重叠，可切片做趋势对照；由于现场没有逐科目
  事件标记或独立目标机文件，它不与某一个拦截机架次建立“一对一文件配对”。
- `175535`、`180433` 无 ARM，故没有 BFL 是合理结果；`175723` 在相机初始化阶段失败。

Blackbox 解码器对定制固件的飞行模式文字存在枚举错位，不能以解码 CSV 的 `ANGLE_MODE` 文本判
定实际模式。Acro/模式门控以主机 CSV 的 `mode_flags`、`aux_enabled` 和 `safety_reason` 为准。

## 5. 逐架次时间线和结果

算法时长均为主 CSV 中首末 `msp_publish_mode=algorithm` 样本之差，不是操作员口述时长。短至
`0.08--0.10 s` 的脉冲只有 5--6 个 50 Hz 行，不能评价命中性能。

|现场运行|ARM/接管时间线|目标与算法结果|结束原因/证据|
|---|---|---|---|
|`17:55:37`|未 ARM；`elapsed=0.127--42.622 s` RC7 已高|无 confirmed 目标；算法 0 行|`msp_prefill_not_ready`，先后 `physical_rc_invalid/manual_rc_unavailable`；无 `run_stop`，console 有资源泄漏告警。来源：`175535` CSV/events/console|
|`17:57:23`|未进入运行循环|无 CSV、无 BFL|相机 `/dev/v4l/by-id/...` 打开失败。来源：`175723_console.log`|
|`17:58:02`|`elapsed=51.605--165.393 s` ARM；无 RC7|无目标、无算法|人工飞行/动力检查；配对 `LOG00094`，尾段正常完成。来源：`175800` CSV/events、`LOG00094.BFL`|
|`18:04:34`|未 ARM、未 RC7|无目标、无算法|无 `run_stop`，console 有资源泄漏告警。来源：`180433` CSV/events/console|
|`18:05:53`|两段 ARM；共 5 次 RC7，其中 1 次在 ARM 前|有间歇目标，但算法 0 行|整段 `aux_enabled=0`；ARM 内 RC7 均为 `aux_disabled`，部分随后 `watchdog_expired`。配对 `LOG00095/96`。来源：`180552` CSV/events、两个 BFL|
|`18:09:19`|ARM 后 RC7 `67.826--69.941 s`；算法 `68.087--69.961 s`，`1.873646 s`|track 39；结果年龄 P95 `75.917 ms`；目标机速度 P95 `0.144 m/s`|RC7 正常退出，`msp_override_inactive`；配对 `LOG00098`。来源：`180918` CSV/events、`LOG00098`、`10_03_35.ulg`|
|`18:11:44`|一个 ARM 段内两次算法脉冲：`0.325069 s`、`0.301820 s`|track 16/30；两段目标机速度 P95 `0.054/0.062 m/s`|两次均由 `aux_disabled` 结束，RC7 仍分别保持约 4 s；第二次后进入 watchdog。配对 `LOG00099`。来源：`181142` CSV/events、`LOG00099`、`10_03_35.ulg`|
|`18:14:34`|算法 `34.740--35.969 s`，`1.229113 s`|track 12；目标机速度 P95 `1.898 m/s`，该段是明确运动目标|`aux_disabled`，随后 watchdog，RC7 到 `39.178 s` 才退出。配对 `LOG00100`。来源：`181433` CSV/events、`LOG00100`、`10_03_35.ulg`|
|`18:16:17`|算法 `35.945--36.729 s`，`0.784200 s`|track 3；目标机速度 P95 `0.796 m/s`|`aux_disabled`，随后 watchdog，RC7 到 `40.415 s` 才退出。配对 `LOG00101`。来源：`181616` CSV/events、`LOG00101`、`10_03_35.ulg`|
|`18:18:02`|算法 `24.555--25.823 s`，`1.268536 s`|track 12；目标机速度 P95 `0.070 m/s`；末端目标失效|`target_invalid` 后约 20 ms 切 passthrough，再进入 watchdog；RC7 到 `31.277 s` 才退出。配对 `LOG00102`。来源：`181801` CSV/events、`LOG00102`、`10_03_35.ulg`|
|`18:19:13`|算法 `48.240--48.340 s`，`0.100646 s`|仅 6 个算法行；目标机 ULog 仅 1 个 10 Hz 速度样本|`aux_disabled`；样本不足以评价控制响应。配对 `LOG00103`。来源：`181911` CSV/events、`LOG00103`、`10_03_35.ulg`|
|`18:20:57`|算法 `51.107--51.188 s`，`0.080616 s`|仅 5 个算法行；结果年龄 P95 `137.377 ms`，超过 100 ms 目标|`aux_disabled`；样本不足。配对 `LOG00104`。来源：`182055` CSV/events、`LOG00104`、`10_03_35.ulg`|
|`18:35:02`|算法 `57.946--58.288 s`，`0.342554 s`|track 2；目标机速度 P95 `0.072 m/s`|`aux_disabled`，随后 watchdog；RC7 到 `61.575 s` 才退出。配对 `LOG00105`。来源：`183459` CSV/events、`LOG00105`、`10_35_19.ulg`|
|`18:37:22`|算法 `38.242--39.913 s`，`1.670804 s`|track 5；接管窗口 confirmed 100%，目标机速度 P95 `0.056 m/s`；形成闭合|`aux_disabled` 后约 20 ms 切 passthrough；`0.902322 s` 后物理接触；RC7 再过 `0.771 s` 才退出。配对 `LOG00106` 与 `10_35_19.ulg`|

### 5.1 跨架次控制与通信统计

9 个进入过算法发布的主机运行共有 406 个算法 CSV 行：总加速度限幅 351 行（`86.5%`），速度
建立项限幅 312 行（`76.8%`）。各运行的写错误、MSP RX checksum/parser 错误和连续发送错误
均为 0；算法行中 `set_raw_rc_ack_fresh` 为 `406/406`。算法窗口的观测写间隔最大值在
`20.106--27.772 ms` 范围，说明单 UART 的 50 Hz 发送不是当天的主要失控因素。来源：上述 9 个
主 CSV 的 `intercept_*_saturated`、`msp_set_raw_rc_*` 和 `msp_rx_*` 字段。

这不等于所有主机运行都没有长间隔：包含启动/退出阶段时，个别日志累计最大间隔达到
`144.306 ms`。主动窗口和全运行窗口必须分开评价，不能用最后一个累计计数反推某次控制脉冲。

## 6. 最后一架次重点证据

### 6.1 关键时序

|事件|UTC|说明/来源|
|---|---|---|
|RC7/MSP OVERRIDE 生效|`10:38:00.723373`|主 CSV `msp_override_active`|
|PNG 首次算法发布|`10:38:00.756168`|主 CSV `msp_publish_mode=algorithm`|
|`aux_disabled`|`10:38:02.426972`|主 CSV；实际发布跨度 `1.670804 s`|
|切到 passthrough|`10:38:02.448019`|停止 PNG 指令，约一个 20 ms 周期|
|靶机接收机开始左滚偏离|`10:38:03.079877`|靶机 ULog，距主冲击约 `0.270 s`|
|最后有效检测曝光|`10:38:03.295687`|主 CSV 的曝光时间换算，距冲击 `54.7 ms`|
|靶机主冲击|`10:38:03.350341`|靶机 IMU 首次超过 `20 m/s2`|
|拦截机首次电机边界响应|`10:38:03.352876`|`LOG00106`，与靶机冲击名义差 `2.535 ms`|
|RC7 回人工侧|`10:38:04.121307`|主 CSV；冲击后 `0.771 s`|
|拦截机 DISARM|`10:38:05.890330`|主 CSV/Blackbox|

以上数字来源于 `logs/analysis/LOG00106_target_joint/metrics.json`。PX4 GNSS 报文映射残差 P5/P95
为约 `-92.8/+70.8 ms`，所以 `2.535 ms` 不能解释为硬件级同步精度；双方冲击落在同一保守
`+/-0.1 s` 窗口并具有一致动力响应，足以确认同一次物理接触。

### 6.2 视觉、导引和目标状态

- 目标有效段新感知结果 354 个，confirmed `98.31%`；接管窗口 40 个新结果 confirmed
  `100%`，track ID 始终为 5，结果年龄 P50/P95/max 为 `49.18/78.47/84.58 ms`。
- 接管窗口总加速度达到 `7 m/s2` 上限的控制行占 `72.62%`，速度建立项饱和占 `65.48%`；
  PNG 项和 FOV 项自身均未饱和。
- 最后正常框面积比约 `0.274`；目标转 lost 后候选导引直到约 `0.304 s` 才因
  `detection_stale` 失效。由于此时已是 passthrough，这些候选没有发给飞控；若仍 ACTIVE，
  该近距盲推迟滞会带来超过 1 m 的潜在盲飞距离。
- PNG 发布窗口内靶机速度 P50/P95/max 为 `0.042/0.056/0.058 m/s`，N/E/D 端点位移为
  `-0.043/+0.035/-0.038 m`。但最后 `0.27 s` 已有左滚输入，主冲击时 Roll 约 `-2.5 deg`。

来源：最后一组主 CSV、`10_35_19.ulg`、`logs/analysis/LOG00106_target_joint/metrics.json` 和
`doc/BETAFLIGHT_FLIGHT_ACTIVE_LOG00106_COLLISION_ANALYSIS.md`。

### 6.3 期望与实际执行

|项目|PNG/主机期望|Blackbox 实际|判定|
|---|---:|---:|---|
|Roll rate|`-31.74--+22.87 deg/s`|setpoint `-32--+25`，gyro `-39--+32 deg/s`|方向和幅值合理|
|Pitch rate|`-34.14--+15.64 deg/s`|setpoint `-34--+18`，gyro `-36--+19 deg/s`|方向和幅值合理|
|setpoint 到 gyro|不适用|Roll/Pitch 同钟延迟均约 `15 ms`，相关系数 `0.991/0.994`|速率闭环通过|
|模型载荷|`1.548--1.736 g`，P50 `1.650 g`|比力 P50/P95/max `1.298/1.450/1.468 g`|模型偏乐观|
|油门|目标 `1367--1396 us`；切入前 `1303 us`|发送 `1305--1396 us`，0.8 s 平滑交接|传输/交接正确|
|电机|不得饱和|PNG 窗口 raw `581--936`，最大极差 `242`|碰撞前无饱和|
|电源|不超过现有限制|最低 `22.65 V`，电流峰值 `21.01 A`|PNG 窗口无异常峰值|

交接完成后实测比力/模型中位约 `0.809`，即当前两点线性推力模型对本次瞬态高估约 `19%`。
碰撞后的 `963 deg/s`、电机 `158/2047`、`4.84 g` 和 `82.06 A` 是外部接触后的 Betaflight
恢复动作，必须从 PNG 指令统计中排除。来源：
`doc/evidence/BETAFLIGHT_LOG00106_CONTROL_RESPONSE_metrics.json`、`LOG00106.BFL`。

### 6.4 退出接管与接触关系

PNG 因 `aux_disabled` 停止后约 `0.902322 s` 才发生接触。当时 RC7/MSP OVERRIDE 仍为高位，
运行时只能发送接管前冻结的 `AETR=1500/1500/1303/1500`；`publish=passthrough` 不代表飞手已经
取得实时四通道控制。接管期间建立的向上速度在停止时仍约为 `2.5--4.4 m/s`（估计器不同），
接触前约为 `2.1--3.5 m/s`。接触锚定估计显示停止算法时仍有约 `3.27--3.39 m` 待闭合，随后
靠既有动量继续接近。

因此可以同时成立：PNG 在碰撞前已经停止发送算法指令；这条命中轨迹和闭合动量又确实由此前
PNG 增推/导引建立。不能把接触说成“停止后 PNG 继续输出”，也不能把它与 PNG 轨迹割裂。

## 7. 跨架次共性问题

1. **退出语义不一致。** 多次运行在 `aux_disabled` 或 `target_invalid` 后停止算法，但 RC7 继续
   保持 MSP OVERRIDE 数秒。此时 passthrough 是冻结值，不是实时遥控输入。最后一架次在该窗口
   内发生接触。
2. **人工时长不可重复。** 软件配置没有接管时限；实际脉冲从 `0.0806 s` 到 `1.8736 s`，与口述
   的约 `0.5 s` 不一致，不能用口令代替数据测量。
3. **速度建立/总加速度长期限幅。** 跨算法行总限幅 `86.5%`、速度项限幅 `76.8%`；限幅后输出
   对 `fixed_vm` 和增益不再线性，且可能压缩 PNG/FOV 修正余量。
4. **近距终端状态缺失。** 大框快速增长、目标消失和接触前没有专用 terminal/impact 状态；固定
   `0.35 s` 检测超时对 `2--4 m/s` 闭合速度过长。
5. **推力模型幅值偏差。** 最后一架次油门链执行正确，但交接后可用比力仅约模型的 81%。电压、
   非线性油门曲线、快速爬升入流和动力动态未被当前两点模型充分描述。
6. **模式门控/开关操作反复失败。** `180552` 的全部 RC7 尝试均 `aux_disabled`；此后 7 个运行
   也由 `aux_disabled` 结束。模式开关与 RC7 的操作关系没有形成硬联锁。
7. **视觉连续性依赖场景。** 全运行阶段有多次 track switch/fragment；虽然最后一架次接管窗口
   track 5 连续且结果年龄达标，但此前户外背景中的关联并不稳定。没有同步原始视频，无法人工复核
   所有 ID 变化是出框、真实重捕获还是假目标。
8. **同步精度不足以测硬件端到端时延。** 主机与 BFL 的 ARM 边沿相差可达约 `0.226 s`；靶机
   GNSS 发布时间也有数十毫秒抖动。`15 ms` 仅是 Blackbox 同钟 setpoint-to-gyro 延迟。
9. **绝对 GNSS 不能作脱靶距离。** 已确认物理接触时，两机独立 GNSS 仍有约 `5.34 m` 水平残差，
   只能使用各自增量和接触锚定结果，不能直接相减经纬度。
10. **归档链不完整。** 全部 meta 的 `repository_commit` 为空；最后一组 console 有 872 个 NUL，
    events 有 1636 个 NUL 且缺失 ACTIVE/接触/DISARM 事件；`VIDEO` 运行没有录像文件。
11. **启动资源管理不稳定。** `175723` 因相机占用启动失败；两次无正常 stop 的运行出现 14 个
    leaked semaphore 告警。

## 8. 问题分级

### P0：再次带桨主动接管前必须处理

|问题|风险|最小改进|
|---|---|---|
|RC7 高位下的冻结 passthrough|算法门控退出后飞手并未恢复实时四通道，容易误判“已人工接管”|把“RC7 回人工侧”设为唯一第一退出动作；在遥控器做互斥混控，使任何 ANGLE/救援模式选择自动先拉低 RC7；地面验证模式切换后 `override_active=0`|
|无近距 terminal/失效策略|大框、lost 后仍可能按旧轨迹盲飞约 0.3 s|依据框面积、面积增长率和 TTC 进入 terminal；近距缩短预测/失效时间；明确“命中科目”和“非碰撞科目”的不同结束策略|
|无软件接管时限且人工脉冲失真|计划 0.5 s，实测可达 1.87 s；风险暴露不可控|恢复可审计的监督时限或硬件定时/三态开关；每次只允许一个脉冲，落地后由日志决定升级|
|当前参数对近距目标过激|高速闭合后即使停止也无法及时消除动量|非碰撞验证降低 `fixed_vm`/加速度并增加错开航线；实际命中科目需预先定义碰撞后处置，不得混用同一测试卡|

### P1：影响性能结论或可复现性

|问题|改进方案|
|---|---|
|速度建立项过早饱和|用距离/TTC 调度期望速度；给 PNG/FOV 横向加速度预留预算，再对速度建立项限幅；记录限幅前后向量和优先级|
|推力模型高估约 19%|用多电压、多油门点和上升工况拟合非线性 `throttle x voltage -> specific force`；加入动力一阶响应，使用 Blackbox 实测比力回归验证|
|目标关联和户外假目标缺少人工证据|同步保存低码率原始/叠框视频；每架次保存目标进入、出框和重捕获事件；区分 ID switch 与正常重新捕获|
|目标动作没有逐科目标记|目标机每个动作单独 ULog，或在双方日志写同一事件脉冲；记录目标是静止、横移还是规避，不靠事后回忆|
|时间同步误差|至少记录双方共同 LED/蜂鸣/GPIO 事件；需要硬件延迟结论时使用同一 PPS/触发源，不能只靠 GNSS 消息发布时间|
|日志提交和 flush 缺失|启动时写入真实 commit、dirty 状态和配置/批准哈希；events/console 逐行 flush，退出时 fsync；异常退出也写 final summary|
|碰撞/末端缺少状态机|增加 `TERMINAL/CONTACT/ABORT` 可观测状态以及触发原因；将碰撞后数据与控制窗口自动分段，避免恢复动作污染统计|

### P2：操作和分析质量

- 启动前检测相机占用并打印占用 PID；正常关闭隔离 RKNN 子进程和信号量。
- 重命名运行前缀，使时长和是否录像反映真实配置；当前 `05S`、`VIDEO` 均有误导性。
- 固化定制固件 Blackbox mode flag 映射，分析脚本不得直接采用上游枚举文字。
- 自动生成每架次 manifest：主 CSV、meta、events、console、BFL、目标 ULog、视频、哈希和配对置信度。
- Web 遥测增加“RC7 仍在 override，passthrough 为冻结值”的醒目状态，避免把 passthrough 理解为
  飞手已恢复实时控制。

## 9. 后续测试与验收方案

### 9.1 控制逻辑离线回放

1. 用当天 10 个算法脉冲回放新的速度调度和加速度分配，不改变原始日志。
2. 对比限幅前后：总加速度占限比例、PNG/FOV 保留量、峰值 Roll/Pitch rate、预测盲飞距离。
3. 使用 `LOG00106` 的电压和实测比力拟合首版电压相关推力模型；要求交接后载荷中位误差从约
   `19%` 降到预先定义的工程范围。
4. 对最后大框和 lost 段做终端门控回放，证明在接触前足够早进入 terminal/abort，而不是只在
   `detection_stale` 后归零。

### 9.2 退出接管和人机操作验证

1. 无桨固定，复现 RC7 高位后切换飞行模式；验证新的遥控器互锁能使 `override_active` 同步变 0。
2. 验证 RC7 退出后下一周期读取的是实时摇杆，而不是接管前冻结 AETR。
3. 操作口令固定为：异常时先退出 RC7，再切模式/稳定/降落；网页状态只供观察，不作为等待条件。
4. 每一架次只做一次预设时长脉冲，并由日志自动判定实际时长，禁止在同一飞行连续升级。

### 9.3 推力、终端和测试设计

1. 推力回归至少覆盖 6S 高/中/低电压和 `1275--1500 us`，分悬停、平缓爬升、快速爬升三类。
2. 非碰撞科目采用错开航线和较低速度建立目标；命中科目另设安全目标、碰撞后策略和场地边界。
3. 静止、横移、规避目标分别建样本集，不能把最后一次静止目标接触外推成动态目标命中率。
4. 若项目指标为命中率至少 `80%`，必须在预定义场景、独立重复样本和明确命中判据下统计置信区间；
   当前单次接触只有 `1/1` 的样本事实，不构成 80% 证明。
5. 每次保存同步视频、双方飞控日志和主机日志；碰撞与非碰撞结果都要保留，不得只保留成功样本。

## 10. 实测、推断与不能证明

### 10.1 已实测

- 12 个 BFL ARM 段与主机日志可唯一配对；`LOG00097` 无主机日志。
- 10 次算法脉冲累计 CSV 样本跨度约 `7.977 s`；算法发布时 MSP ACK 新鲜，未见写入、checksum、
  parser 或连续发送错误。
- 最后一架次发布 `1.670804 s`，停止后 `0.902322 s` 发生同一次双机物理接触。
- 最后一架次 Roll/Pitch setpoint-to-gyro 同钟跟随约 `15 ms`，方向正确，碰撞前电机未饱和。
- 最后一架次油门交接和 1500 us 上限有效，但实测比力低于模型需求。
- 靶机在最后一架次 PNG 发布窗口内平移基本静止；末端有左滚操作。

### 10.2 有证据支持的工程推断

- 最后接触轨迹的闭合速度和动量主要由此前 PNG 增推/导引建立；停止发布后依惯性继续闭合。
- 当前推力偏差主要来自简化线性模型、电压差、入流和动力动态，而不是 MSP 丢油门；发送油门、
  飞控 throttle、电机和电流波形方向一致。
- 跨架次高饱和占比表明速度建立参数对当前近距场景过强，但不能仅凭饱和占比确定最佳新参数。
- 多次 `aux_disabled` 与飞行模式/开关操作相关；具体每次飞手动作仍缺同步视频确认。

### 10.3 当前不能证明

- 命中率达到或超过 `80%`。
- 对横移、规避目标的命中率与静止目标相同。
- 用两机绝对 GPS 直接得到脱靶距离；接触时仍有约 `5.34 m` 水平残差。
- 相机曝光到电机响应的硬件级端到端延迟；现有 `15 ms` 仅是飞控同钟速率环延迟。
- `LOG00097` 是否运行过 PNG、当时目标状态和控制命令。
- 除最后一架次外，各 BFL 全日志中的峰值加速度/电流是否由 PNG、人工动作、着陆或接触造成；缺少
  同步视频时不得按峰值强行归因。

## 11. 归档复现命令

以下命令只读取原始文件。Blackbox 的 gyro raw 在当前定制固件中与 deg/s 数值一致；不要使用
`--unit-rotation deg/s`，否则 `gyro_scale=1` 会造成错误换算。

```bash
cd /home/linux/Documents/PNG-betaflight-upward-camera

# 文件完整性
sha256sum logs/blackbox_import/LOG{00094..00106}.BFL
sha256sum logs/flight_active_supervised/FLIGHT_ACTIVE*.csv
sha256sum logs/target-log/10_03_35.ulg logs/target-log/10_35_19.ulg

# 主机运行配置、内部时间和绑定哈希
for f in logs/flight_active_supervised/FLIGHT_ACTIVE*_meta.json; do
  jq '{created_local,created_unix_s,config_path,config_sha256,
       allow_control,control_mode,repository_commit}' "$f"
done

# 关键离散事件；最后一组有 NUL，先过滤再解析
for f in logs/flight_active_supervised/FLIGHT_ACTIVE*_events.jsonl; do
  tr -d '\000' < "$f" | jq -c \
    'select(.event=="armed" or .event=="msp_override_active" or
            .event=="safety_state" or .event=="publish_mode" or
            .event=="target_valid" or .event=="run_stop")'
done

# 查看 Blackbox 头中的固件和绝对时间
for f in logs/blackbox_import/LOG{00094..00106}.BFL; do
  strings -n 4 "$f" | rg '^H (Firmware revision|Board information|Log start datetime|motorOutput|acc_1G):'
done

# 解码某一架次；输出目录应放在 /tmp，避免污染归档目录
/tmp/betaflight-blackbox-tools/obj/blackbox_decode \
  --unit-rotation raw --merge-gps --save-headers \
  --output-dir /tmp/log00106_decode_raw \
  logs/blackbox_import/LOG00106.BFL

# 靶机覆盖范围、dropout 和消息清单
ulog_info logs/target-log/10_03_35.ulg
ulog_info logs/target-log/10_35_19.ulg

# 重建最后一架次控制响应图和机器可读指标
python3 tools/plot_log00106_control_response.py \
  --blackbox-csv /tmp/log00106_decode_raw/LOG00106.01.csv \
  --blackbox-event /tmp/log00106_decode_raw/LOG00106.01.event
```

复现最后一架次时优先读取：

- `doc/BETAFLIGHT_FLIGHT_ACTIVE_LOG00106_COLLISION_ANALYSIS.md`
- `doc/BETAFLIGHT_LOG00106_EXPECTED_ACTUAL_RESPONSE_REPORT.md`
- `doc/evidence/BETAFLIGHT_LOG00106_CONTROL_RESPONSE_metrics.json`
- `logs/analysis/LOG00106_target_joint/metrics.json`

本报告只归档和审计现有数据，没有修改控制代码，也没有把任何无法唯一配对的数据写成确定事实。
