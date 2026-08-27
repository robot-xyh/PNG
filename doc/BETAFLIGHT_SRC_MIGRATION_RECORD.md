# Betaflight src 参考与迁移记录

## 目的与安全边界

本文记录从 Orange Pi `/home/orangepi/src/circle_pilot` 参考或移植到当前 Python 工程的
非视觉能力。原目录不修改；历史 Python 实机验证保持无桨、`LOG_ONLY`。2026-07-15 新增的
Python `noprop_bench` 输出路径仍须在 Orange Pi 实测，不能把代码测试当作实机验收。机器可读来源、哈希和候选参数保存在
`config/betaflight.src-reference.json`，该文件明确禁止作为运行配置加载。

## 来源基线

2026-07-11 对 12 个 Betaflight 配置、MSP、RC、安全门控和 systemd 文件计算 SHA256。
板端 `src/circle_pilot` 没有 Git 元数据，因此不能用 commit 标识版本；后续来源文件任一哈希
变化时必须重新审计，不能静默沿用旧结论。

已确认的来源能力：

|能力|src 实现|迁移决定|
|---|---|---|
|MSP STATUS/ATTITUDE/RC/BOXIDS|`adapters/bf/common`|参考 BOXIDS 与单串口所有者设计|
|MSP OVERRIDE mode 定位|permanent ID 50，缺失时回退固定 bit|Python 改为缺失即硬阻断|
|RC rate/throttle 映射|线性 PWM 映射和中心斜率近似|保留限幅/斜率限制，后续实现曲线反算|
|物理杆保持与 AUX 保留|`bf_control_host`|移植 fail-closed 策略，禁止伴随机主动 ARM|
|watchdog/新鲜检测/armed 门控|`bf_gating`|扩展 Python safety inputs 和日志|
|油门交接|0.4 s 线性混合|实现但在 hover throttle 标定前阻断控制|
|systemd 打包|`adapters/bf/systemd`|改写为 Python LOG_ONLY 服务，安装但不启用|

## 已发现冲突

- 默认 PNG 文件使用 `hover=0.078`、`strike=0.082`，速度档使用 `0.283/0.50`。
- `bf_flight_common.yaml` 使用最大速率 `3.491 rad/s`，现有 `bf_runtime_test` 仍期望
  `5.236 rad/s`，板端测试有两项失败。
- 来源在 MSP_RC 逻辑层使用 R/P/Y/T 索引 0/1/2/3，发送前显式重排为 A/E/T/R；该转换与
  实际 `AETR1234` 一致，但方向仍须通过无桨测试。
- 所有飞行 profile 均设置 `dry_run=false`，不能作为当前工程默认值。
- 未发现 Betaflight CLI `dump/diff all`、Blackbox、动力标定或 Rate/PID 来源记录。

处理决定：全部冲突值只进入来源清单；RK3588 运行配置继续保持零 `rate_gain_matrix`、
`control_authorization.enabled=false`，直到 CLI 快照、无桨方向验证和动力标定完成。

## 阶段记录

|阶段|状态|验收证据|
|---|---|---|
|S0 来源清单与冲突审计|完成|来源 SHA256、候选参数、冲突和禁止生效标记|
|S1 只读飞控快照|完成|MSP identity/BOXIDS/BOXNAMES/telemetry 与真实 CLI artifact|
|S2 安全 MSP 运行架构|完成|单元测试证明所有授权条件 fail-closed|
|S3 Python systemd|完成|服务已安装且 disabled/inactive，命令固定 LOG_ONLY|
|S4 Orange Pi 验证|完成|60 s 日志、RAW RC 发送计数为 0、文件哈希一致|
|S5 配置审计工具|完成|schema v2、diff/dump 解析、一致性审计、时钟与 artifact 哈希|
|S6 真实 CLI 审计|完成|八类配置完整，解析错误/重复/跨导出冲突均为 0|
|S7 Python PNG 无桨 RC 验证|代码完成、实机待执行|受限profile、预填状态机和测试已完成；缺新快照与板端CSV/Blackbox证据|

## 留档规则

- 代码、示例配置、来源清单、决策和脱敏测试摘要进入 Git。
- 每次实机运行在 `logs/` 下生成独立目录，保存原始 CSV、meta、CLI 导出和 manifest；
  `logs/` 不提交，但 manifest 中必须包含每个文件 SHA256。
- 不记录 SSH 密码、无线凭据或其他秘密。
- 每阶段独立提交；板端验证完成后追加最终验证记录，不改写历史结论。

## 只读快照命令

先在 Betaflight Configurator CLI 中人工执行 `diff all` 和 `dump all` 并分别保存文本，再运行：

```bash
python3 tools/capture_betaflight_snapshot.py \
  --config config/betaflight.rk3588.example.json \
  --duration-s 5 --rate-hz 5 \
  --cli-diff-all /path/to/betaflight_diff_all.txt \
  --cli-dump-all /path/to/betaflight_dump_all.txt
```

`--cli-export` 仅为旧命令兼容入口，不能与两个新参数组合。schema v2 分别保存原始导出，
结构化解析 Ports、Receiver、Modes、Failsafe、PID、Rate、Blackbox 和 Battery，并生成
`configuration_review.json`。缺文件、缺类别/结构、重复或畸形命令、diff/dump 交叉冲突均
形成 blocker。
快照工具没有 RC 发送接口，且始终输出 `control_ready=false`，不得修改 manifest 绕过授权。

## 安全运行架构

新增的 MSP worker 是唯一允许承载未来 RC 发布的路径，示例配置默认
`msp_runtime.io_worker_enabled=false`。旧同步路径已改为只读，即使命令行误传
`msp_raw_rc --allow-control`，worker 未启用时也会直接拒绝，退出阶段不再发送中性 RC。

未来控制授权要求独立 approval manifest，同时校验快照 SHA256、FC identity、参数哈希、
来源冲突已关闭和 MSP OVERRIDE permanent ID 50。运行时还必须满足 armed、OVERRIDE active、
物理 RC/姿态/遥测新鲜、AUX、电压、目标和 watchdog gate。物理 RC 合并只覆盖明确 mask
内的 A/E/T/R，ARM 与其他 AUX 始终来自接收机；物理 RC 过期后停止发送，由 Betaflight
failsafe 接管。

## systemd 安装

```bash
./tools/install_betaflight_log_only_service.sh \
  --project-root /home/orangepi/png_betaflight_python
```

服务使用 `duration-s 0` 持续记录，固定 `rknn_bytetrack` 和 `control-mode log_only`。安装器
拒绝含 `--allow-control` 的单元，并在 daemon-reload 后执行 `disable --now`。渲染单元的
SHA256、Python/config 路径和 enabled/active 状态写入忽略的 `logs/deployment/`。

## 2026-07-11 Orange Pi 验证

只读快照识别到 `BTFL 25.12.2`、API `1.47`，BOXIDS 为
`0,1,2,5,3,6,27,11,46,7,13,15,19,20,26,30,32,33,34,35,36,37,39,45,40,41,48,49,51,52,53`。
实际列表不含 `src` 假设的 permanent ID 50，因此该快照中
`msp_override_available=false`，控制继续阻断。后续源码核对确认 Betaflight 仅在
`msp_override_channels_mask != 0` 时公布该 BOX；当前 mask=0 会隐藏 ID 50，不能单凭这份
BOXIDS 判定固件未编译该功能。快照完成 25/25 样本、错误 0。

60 s 相机+MSP+RKNN+ByteTrack 联合测试完成 300 行，全程 `LOG_ONLY`、`rc_active=0`；
`msp_set_raw_rc_attempt_count` 和 success 最大值均为 0，MSP、发送、相机和 perception worker
错误均为 0。感知均值 27.311 Hz，结果帧龄均值/最大 1.403/13.936 ms，RKNN 总耗时
5.550/6.523 ms，ByteTrack 0.257/0.377 ms。

systemd 单元已安装到用户目录，状态为 disabled/inactive，没有运行进程，渲染命令不含
`--allow-control`。关键部署文件与本地 SHA256 全部一致。机器可读的脱敏结果和原始 artifact
哈希见 `config/betaflight.rk3588.validation.json`；该轮板端时钟仍不正确，随后处理见下节。

## 2026-07-11 配置审计与板端复测

- 新增只读 `MSP_BOXNAMES`。该飞控返回 350 字节名称载荷，实际使用 MSP v1 jumbo frame；
  已按 `0xff + command + uint16 payload length` 实机帧格式补充解码和回归测试。
- 31 个 BOX 名称与 BOXIDS 数量、顺序一致。末三个 ID 51/52/53 分别对应
  `STICK COMMANDS DISABLE`、`BEEPER MUTE`、`READY`，均不是 ID 50 的替代项。Betaflight
  `msp_box.c` 仅在 override mask 非零时加入 ID 50；因此当前结论是“mask=0、mode未公布”，
  不是“固件确定不支持”。设置 mask=15、保存重启并重新查询前，控制阻断结论不变。
- Configurator 导出的 `diff all`/`dump all` SHA256 分别为
  `18b6997c082aad1f22764439c97117c2e212eeb8292c4610a1aaad9ff087dd43` 和
  `7641dd6e158a0fc6ab0cb00bc8710be14cb1d7eeeb2eeba3465e400ff853c73f`。解析覆盖八类配置，
  缺失类别/结构、畸形命令、重复赋值和交叉冲突均为 0。
- 最终 schema v2 快照完成 25/25 样本、错误 0，原始目录为
  `logs/betaflight_snapshots/betaflight_snapshot_20260711_222720`，manifest SHA256 为
  `1e714858710fea7ed1923a1f4d1d7de285b32da14400a26b3089aec97eaa3a8e`。
- 飞控配置确认 CRSF、`AETR1234`；ARM 为 RC5 的 900--1300 us，ANGLE 为 RC8 的
  1700--2100 us，BEEPER 为 RC6 的 1300--1700 us。Failsafe 为 delay 15（0.1 s 单位）后
  `DROP`；Blackbox 为 SDCARD/NORMAL/1/4。
- PID profile 0 的 roll/pitch/yaw PIDF 为 `51/64/22/84`、`54/67/25/87`、
  `51/64/0/84`。Rate profile 0 为 Betaflight rates，RPY 均为 RC rate 100、expo 0、
  super rate 70；Python 线性 RC 映射尚未反算该曲线。
- 飞控 `msp_override_channels_mask=0`、`msp_override_failsafe=OFF`，且没有 BOX ID 50。
  Python 候选 mask 为 15，不能生效。Python AUX gate 使用 RC5 高位，而 RC5 是低位 ARM，
  两条件互斥；必须改用经发射机确认的独立 AUX，不能直接改参数猜测。
- Orange Pi 于 22:16:54 重启，原因日志不可用；`rtc-hym8563` 重启后回到 2021-01-01。
  `chrony` 无可达时间源，保留 `clock_not_synchronized` 与 `rtc_not_aligned` blocker。
systemd 仍为 disabled/inactive，本轮没有执行 RC 注入、ARM 或电机控制。

## 2026-07-11 src 动力供电无桨静默测试

在动力电池已连接、全部桨叶拆除且未执行 ARM/OVERRIDE 的条件下，对板端原
`/home/orangepi/src/circle_pilot` 做了两轮 60 s 测试。新增文件仅用于 bench，原飞行配置和
源码未修改：

- `config/strike_png_bf_bench.yaml`：`dry_run=true`，SHA256
  `bb25abe49650fa15f4760318915a2a8444c6551c51998b2fa83a1aab94f15dd8`。
- `config/bf_flight_bench_common.yaml`：`passthrough_in_dry_run=false`，SHA256
  `131da3f12ea3b75014759237103271701aa228268d0b01b291d003e6beb870fe`。
- `config/camera_bf_mono_011_bench.yaml`：相机节点改为 `/dev/video0`，SHA256
  `fed038419d3098c1cddba50a06a6a8c7a21e229818059b273f6fff2e16e55708`。

首轮确认 `/dev/video1` 是无视频格式的 UVC metadata 节点，程序安全回退后相机启动失败。
修正到支持 1280x1024 MJPEG 的 `/dev/video0` 后，第二轮完成 3952 次抓帧和推理更新，最终
`grab_fail=24` 为启动期累计值且未继续增长；RKNN 单次总耗时约 5.48 ms。MSP 最终
`ok=5912/fail=3`，STATUS/ATTITUDE/RC 均新鲜，发送审计全程
`send_hz=0.0`、`MSP_SET_RAW_RC=0`、`msp/live=0`。日志为
`logs_ws/bf_flight_png_bench_camera_20260711_225133.log`，SHA256
`9b04d408b07c90e788ca46dc3c1b9662081f4046c02dd4ce7e04da670b37b5d4`；进程已退出。

剩余警告为：mask=0 导致 ID 50 未公布、零拷贝初始化失败后使用单槽回退、相机 sharpness
从 50 裁剪到硬件上限 7、gain 控件不受支持。该轮证明 src 的 MSP 静默读取、相机和 RKNN
通路可运行，不证明电池量测、目标识别准确率、RC 接管、Rate/油门映射或电机控制可用。

8080 页面需要 `bf_flight_png` 和 `bf_debugd` 同时运行；单独执行 flight 二进制不会监听
HTTP。新增 `config/strike_png_bf_debug_bench.yaml`，指向 bench flight 配置、关闭自动图片落盘，
并将不出帧的 H.264/WebRTC 改为 MJPEG。配置 SHA256 为
`9ad889accc1d38279e1f1ae2ffaa4317022dea16f07e9a322790e7ad4b2cc1d9`。工作站实测：

- `/` 返回 HTTP 200、132851 字节；
- `/api/series.json` 返回 `ok=true`、backend=`bf` 和 256 个实时样本；
- `/api/video/status` 返回 telemetry/preview open、transport=`mjpeg`；
- `/api/video/mjpeg` 在 3 s 内收到 197968 字节，包含合法 multipart 边界和 JPEG SOI。

双进程复测的 flight/debug 日志分别为
`/home/orangepi/logs_ws/bf_flight_png_20260711_230353.log` 和
`/home/orangepi/logs_ws/bf_debugd_strike_png_20260711_230353.log`，SHA256 分别为
`0f4765a20fa8a0dc70545cd565dd5304a41029d05610f9ef516b0e242a8ded7a`、
`00c8f5a826239943818ea6c8b0d702277ddc539a51fb5fe91175731505257022`。flight仍为
`MSP_SET_RAW_RC=0`、非零 `send_hz=0`。debug日志中的一次 `jpeg_send_failed` 是限时curl
主动关闭MJPEG连接所致；测试结束后两个进程均已退出。

## 2026-07-12 src 动力供电无桨接管测试

当次测试记录的飞控CLI为 `msp_override_channels_mask=15`、`msp_override_failsafe=OFF`，模式配置为
ARM=`aux 0 0 0 900 1300 0 0`、ANGLE=`aux 1 1 3 1700 2100 0 0`、MSP
OVERRIDE=`aux 2 42 2 1700 2100 0 0`。测试时动力电池已连接、四个桨叶已拆除；src先持续
发送MSP RC帧，随后RC5低位ARM，再将RC7高位切入OVERRIDE。操作员现场确认电机仅正常怠速，
切入接管时转速无突跳。该结果证明正常启动顺序下首帧前885 us没有进入电机控制，并初步确认
RC5/RC7分工、ID 50模式和mask 15正常工作。

本条结论仍为初步记录：日志路径、配置SHA256、MSP发送频率和错误计数尚待归档；也未覆盖
程序崩溃、UART断开、Orange Pi掉电、MSP短帧、物理RX断链或PNG算法输出。

## 2026-08-27 当前固件CLI模式ID与批准逻辑更新

Configurator重新导出的 `diff all` 和 `dump all` 均给出
`aux 2 50 2 1700 2100 0 0`，不再是旧记录中的42；MSP BOX元数据同时确认
`MSP OVERRIDE` permanent ID=50、index=28。两份导出已复制到
`/home/orangepi/png_betaflight_python/config/`，SHA256分别为
`f2e60f6bb7f7b2d4cc644612f9f90bdb36027909fb84f33851ea7f22ffe066cc`和
`30d7b1cb71f4bcf52cf4541980acad9a5e5ff7086e93a2a6938c310659c7c144`。

新快照为
`logs/betaflight_snapshots/betaflight_snapshot_20260827_153739/manifest.json`：共25个样本、
0个采集错误，`diff all`/`dump all`无跨导出冲突、重复赋值或解析错误。批准工具不再硬编码
旧CLI mode ID 42，而要求无桨配置显式声明 `msp_runtime.override_mode_cli_id=50`；该配置、
快照和最终批准文件通过SHA256绑定，禁止直接编辑批准JSON绕过检查。

批准文件已在Orange Pi生成：`logs/betaflight_noprop_approval.json`，SHA256为
`375a38f82e1aed7ba811cb98a89c4992461d05a5fcfcf772eaddb998cea6552f`。其绑定的本机配置和
快照SHA256分别为 `bd0949414405cb0c44f530b925d50fee33a00299b5d167099e751fefeed3907f`、
`cb4c20140a22a34bad31fb5930da356a35bff9804c17328c2efdba251f7c438f`；运行时复核结果为
`approved=True, reason=approved`。本次操作没有启动runner或发送 `MSP_SET_RAW_RC`。

板端修改前备份为
`logs/deployment_archive/mode_id_50_pre_sync_20260827_153739.tar.gz`，SHA256为
`367def65004fb367ade5e536cbdf4fc02d13a624b48d6fb1fb5e74783d4a338c`；修改后完整包包含代码、
配置、CLI导出、快照和批准文件，保存在
`logs/deployment_archive/mode_id_50_approved_20260827_160000.tar.gz`。

## 2026-08-27 接管遥测界面更新

浏览器首页新增 `Control Readiness`，直接显示批准、RC5 ARM、RC7 OVERRIDE、RC预填、物理RC、
MSP遥测、姿态、电压、Tracker、LOS、TTC、watchdog、Guidance、Command、Publish和最终安全
状态。顶部状态条将“检测跟踪成立”和“命令有效”拆开，阻塞栏同时显示可读说明与原始reason
code，避免把画面中出现bbox误认为算法已接管。

页面同时显示实际发送AETR、命令角速度/推力、SET_RAW_RC成功/尝试/错误、最大RTT和deadline
miss。RC曲线改用已从MSP逻辑RPYT重排后的 `physical_us` 与 `sent_us` 比较，不再把原始
`input_us` 的AERT位置直接画到AETR发送曲线上。MJPEG叠加层新增ARM、OVERRIDE、publish、
tracker和阻塞reason。修改不改变安全状态机、批准逻辑或控制输出；本地221项单元测试通过，
并以1440x1000和390x844视口完成静态布局检查。

## src 对当前问题的实际处理

`src` 提供了若干可复用的安全机制，但没有解决当前飞控配置不兼容和动力标定问题：

|问题|src 处理|审计结论|
|---|---|---|
|ARM/AUX 优先级|算法只覆盖 mask 0--3；未覆盖通道从物理 RC 回填|ARM/AUX 不由算法主动改写，这一点可复用|
|通道顺序|MSP_RC 按 R/P/Y/T 读取，发送前转换为 A/E/T/R|与当前飞控 map 一致，旧“顺序冲突”结论已修正|
|控制接管|要求 armed、OVERRIDE、状态 watchdog 和新鲜检测同时满足|正常路径 fail-closed，但依赖 OVERRIDE 正确存在|
|油门交接|接管前锁存物理油门，0.4 s 线性混合；锁存异常时回退 hover|避免瞬时跳变，但 hover 候选值互相冲突且无动力标定|
|Rate 映射|按当前 100/0/70 的中心斜率 3.491 rad/s 做线性近似|已识别 Super Rate 放大，但未做精确反函数；测试仍期望 5.236|
|时间|控制、watchdog、检测年龄使用 `steady_clock`|RTC 回跳不破坏控制 dt，但日志仍使用系统墙钟|
|动力电池|armed gate 间接阻止未解锁算法输出|没有 VBAT/电流读取、低电压 gate 或电池存在性判断|

存在三个不能直接复用的关键行为：

1. BOXIDS 找不到 permanent ID 50 时，`src` 仅告警并回退 YAML `0x08000000`。在当前
   BOXIDS 中 bit 27 对应 permanent ID 49 `LAUNCH CONTROL`，不是 MSP OVERRIDE；Python
   必须继续采用“ID 50 缺失即阻断”。
2. `src` 假设 `override_channels_mask=15`，但不会读取CLI中的实际值；飞控已于2026-07-12
   人工设置为15，运行时一致性仍未由src自动验证。
3. `dry_run=true` 且默认 `passthrough_in_dry_run=true` 时仍会发送 `MSP_SET_RAW_RC`，因此
   不能把 `src` dry-run 当作本项目的只读 `LOG_ONLY`。

板端执行已有 `bf_runtime_test`，两项共享配置检查仍按 5.236 rad/s 编写，与活动配置
3.491 rad/s 不一致，测试退出码为 1。`src` 没有 NTP/RTC 配置，也没有 Blackbox、CLI
快照或电池标定证据。机器可读哈希、缓解措施和剩余风险见
`config/betaflight.src-reference.json`；原源码和production配置未修改，仅增加上述bench
配置；`bf_flight` systemd服务保持 inactive。

## 2026-07-15 Python PNG 无桨接管实现

根据 src 的正常启动顺序和无桨接管结果，Python 路径新增
`config/betaflight.rk3588.noprop.example.json`。该 profile 只允许
`noprop_bench` scope，要求四桨拆除、RC7/AUX3高位接管、mask 15、ID 50 和当前 JSON
SHA256 全部匹配；输出硬限制为 roll/pitch 3 deg/s、yaw 0 deg/s、throttle 1000--1100 us。

代码迁移和修正如下：

- worker 在 RC7 人工侧读取 MSP_RC，显式把逻辑 R/P/Y/T 转为当前 `AETR1234` wire order，
  连续成功发送至少10帧后才置 `prefill_ready=1`。
- 主通道任一值不在900--2100 us、RC7启动时已打开且没有人工锁存值、物理RC过期或通道数
  不匹配时拒绝发送；不会把首帧前885 us当作有效预填。
- PNG命令超过0.15 s未更新时退回接管前锁存人工RC；进入算法时油门用0.4 s交接；正常
  退出发送3帧人工透传。ARM及全部AUX始终保留物理接收机值。
- RC mapper 使用Betaflight `applyBetaflightRates` 的三轴数值反函数，当前批准工具要求CLI
  rate profile 0与JSON的100/0/70一致。
- CSV新增预填、透传、算法、stale、发布模式和`rc_sent_ch*`，终端按状态变化打印实际发送
  的AETR前四通道。

`tools/create_betaflight_noprop_approval.py` 还核验新快照的diff/dump完整性、artifact哈希、
ID 50、mask 15、`msp_override_failsafe=OFF`、RC7 mode和FC identity。当前仓库只有mask=0时
采集的历史快照，不能据此生成批准；必须在Orange Pi关闭Configurator后重新采集。该实现已
通过单元测试和静态检查，但尚未在 `/dev/ttyS1`、真实相机和无桨电机上执行，因此未记录为
实机通过。

## 2026-08-26 Python PNG 诊断日志 schema v2

为支持下一轮 Orange Pi 无桨 PNG 接管定位，Python 路径补齐飞控反馈、串口调度、映射链和
平台健康证据。本次没有修改 PNG/LOS/TTC 数学、3/3/0 deg/s 限幅、1000--1100 us 油门包线、
RC7 接管门禁或批准清单逻辑。

- MSP 新增 ID 102 `MSP_RAW_IMU`，按 Betaflight 当前载荷解析 3 轴 ACC raw、gyro deg/s 和
  MAG raw；未确认量纲的 ACC/MAG 不转换为 SI。
- 单串口 worker 改为先处理 50 Hz `MSP_SET_RAW_RC`，每个发布周期最多插入一个遥测请求。
  无桨配置为 RAW_IMU 20 Hz、ATTITUDE/RC 10 Hz、STATUS 5 Hz、ANALOG 2 Hz。同步串口请求
  仍可能阻塞，代码记录每命令 RTT、错误和发送 gap，不将其描述为硬实时。
- 遥测改用各消息独立时间戳；姿态缓存只接收新的 ATTITUDE 样本，避免其他 MSP 响应把旧
  姿态误判为新鲜数据。
- `RcCommand` 增加请求/硬限幅 rate、Betaflight 反算 stick、限幅后且 slew 前 PWM 和 thrust
  诊断；worker 记录油门交接 source/target/alpha/output。
- 每轮除 CSV/meta 外生成 `*_events.jsonl`；独立 1 Hz sampler 缓存 RK3588 温度、频率、
  内存、磁盘、负载和进程 RSS。不可读取的硬件字段留空，未增加原始视频或逐包 MSP 写盘。
- `tools/analyze_betaflight_noprop_log.py` 从 meta 读取实际无桨包线，检查 885 us、RC/rate/
  throttle 越界、门禁不满足时算法发送、SET_RAW_RC 错误和超过三个发布周期的成功帧间隔，
  输出同名前缀 `*_audit.json` 并用非零退出码表示失败。

测试增加 RAW_IMU 正负值和短载荷、每命令统计、SET 优先轮询、分消息时间戳、发送时序、
映射中间量、事件边沿、平台采样和安全/不安全审计样例。该版本最初尚未复制到 Orange Pi；
2026-08-27 已随只读浏览器遥测部署并取得真实 `/dev/ttyS1`、相机和 RKNN/ByteTrack 日志，
结果见下节。无桨电机和 Blackbox 对齐数据仍未取得；代码通过不等于实机通过。

## 2026-08-27 Python PNG 只读浏览器遥测

Python 路径新增独立只读 Web 层，目标是让 PC 浏览器直接观察当前 Python PNG，而不是复用
`src/bf_debugd` 的 C++ 共享内存、旧 series schema 和参数写入接口。HTTP 服务使用 Python
标准库，Orange Pi 不新增 Flask/FastAPI 依赖。

- `TelemetryHub` 保存最新结构化快照和 5 Hz、60 s 有界历史；HTTP 提供最新值、历史和 SSE。
- RKNN/ByteTrack 在完成同帧检测后投递图像和 bbox；MJPEG 编码线程只在客户端连接时运行，
  队列容量为一，过载覆盖旧帧。
- API 只有 GET/HEAD；没有 ARM、OVERRIDE、参数保存或 RC 写入端点。局域网部署绑定
  `0.0.0.0:8080`，只允许 loopback 和 `192.168.124.0/24`。
- CSV 升级为 schema v3，新增 Web 发布、客户端、请求、编码、丢帧和错误字段；meta 保存 Web
  配置和只读声明，JSONL 记录 Web 错误边沿。无 Web 的 schema v2 日志仍可审计。
- Web 服务绑定发生在打开 `/dev/ttyS1` 和相机之前；端口冲突直接拒绝运行。服务启动后的
  浏览器断开、慢客户端和编码异常不参与安全决策，也不改变 MSP worker 调度。

验收必须同时满足：PC 根页面/JSON/SSE/MJPEG 可达，页面没有写接口，`MSP_SET_RAW_RC=0`，
MSP/Web 错误为零，单浏览器下 20 Hz 主循环和 RKNN/ByteTrack 稳态性能不出现不可解释退化。
Python runner 与 `bf_debugd` 仍互斥；本功能不作为飞行看门狗或控制反馈来源。

### Orange Pi 联调结果

2026-08-27 将本节实现同步到 `/home/orangepi/png_betaflight_python`，保留板端硬件配置并新增
`telemetry_web`；修改前配置归档为
`config/betaflight.rk3588.noprop.local.pre_web_20260827_122317.json`。使用
`log_only + rknn_bytetrack` 运行 44.96 s，输出
`logs/web_nopower_rknn_20260827_122413*`：

- schema v3 CSV 共 895 行，Web 发布 894 次、预览编码 5 帧、单槽覆盖丢弃 4 帧、Web 错误 0；
- MSP 请求错误 0，浏览器健康检查为 fresh，工作站可读取 JSON 并看到真实 640x512 MJPEG；
- RKNN 单帧总耗时最大 18.165 ms，板端最高温度 56.384 C；
- `MSP_SET_RAW_RC` attempt/success/error 均为 0，审计 `passed=true`、violations 为 0；
- `publish_deadline_miss_count=228`。本次没有 RC 输出，该值不代表漏发命令，但说明单 UART
  轮询在 50 Hz 发布周期下仍有调度超期，进入任何真实控制测试前必须降低轮询占用或重新验证
  发布周期和最长发送间隔。

工作站截图归档为 `logs/web_ui_validation/orangepi_live.png`。测试进程退出后页面显示 `STALE`
属于预期行为；本次结果只关闭浏览器遥测接入项，不构成有桨飞行许可。

首轮页面检查还发现 MSP_RC 输入采用 `A/E/R/T` 逻辑顺序，而发送 wire map 为 `A/E/T/R`；
旧页面直接套用 AETR 标签，会把 885 us 油门误标为 R。API 现同时发布 `input_order`、原始
`input_us`、`wire_order` 和重排后的 `physical_us`，页面按 wire map 显示。20 s 复验中页面
正确显示 `T=885`、`R=1500`，397 行日志再次审计通过，Web 错误和 RC 写入均为 0。两轮日志、
审计和配置前后版本已归档为 `logs/web_telemetry_validation_20260827_122413.tar.gz`，SHA256 为
`10ee3f075fdb612c8dbfde7097ba81e4ae79af04dd4aeb0ba998c873ccaea6d7`。

## 2026-08-27 无桨实时接管日志复盘与异步间隙修正

现场运行使用 Orange Pi `/home/orangepi/png_betaflight_python`、`/dev/ttyS1`、动力电池、
拆桨状态和已批准的 `noprop_bench` 配置。原始日志为
`logs/betaflight_log_20260827_171517.csv`，时长 1242.82 s、24579 个主循环、19288 个去重
感知结果。退出顺序经日志确认是先 RC7 回人工，再 DISARM，最后向 Python 发送 SIGINT。

复盘结论如下：

- `MSP_SET_RAW_RC` 最终 attempt/success 为 40836/40836，worker send error 为 0；批准、
  ID 50、mask 15 和串口发送链路不是本次主要故障。
- 9876 个真实跟踪输出中，主轨迹 ID 133 持续 5965 帧，后续 ID 206 持续 1049 帧。页面所见
  大量 `ID -> None -> ID` 主要来自 20 Hz 主循环夹在 13--26 Hz 感知结果之间产生的
  `perception_no_new_result`，不能按真实 ByteTrack 换 ID 统计。
- 导引拒绝以 `timestamp_after_buffer=5310`、`bbox_top_clipped=2848`、
  `area_not_expanding=867` 和 `ttc_out_of_range=549` 为主，仅 203 个感知结果产生有效导引。
  曝光返回时间相对最新 ATTITUDE 样本偏差 P50/P90 为 7.79/89.55 ms，当前 10 Hz ATTITUDE
  严格插值无法覆盖大量新图像；本轮只增加诊断，不放宽姿态门控。
- 安全状态虽然出现 176 个 `ACTIVE` 主循环，但大多只有单个 20 Hz 周期。异步 worker 实际
  `publish=algorithm` 104 行，发送 A/E 仅 1499--1501 us，油门交接最高约 1002 us，因此
  无桨电机没有肉眼明显转速变化符合日志。
- 板温中位/最高为 79.46/84.08 C；温度超过 80 C 时 Tracker FPS 中位为 13.98。冷机、散热
  和关闭不必要 MJPEG 客户端后的对照测试必须先于 ByteTrack 阈值调整。
- 原 schema v3 日志安全审计为 `passed=false`：最大 SET_RAW_RC 成功间隔 182.255 ms，超过
  50 Hz 输出的 60 ms 审计门限；另因旧日志的 `publish_mode` 与 staged gate 不在同一时间
  基准，无法证明 19 个算法帧的发送当刻 gate。后者不是直接认定越权，而是证据不足；前者
  是必须在冷机复测中关闭的真实时序缺口。

针对已确认的异步问题，CSV 升级为 schema v4：新增 `perception_new_result` 和
`detection_attitude_offset_ms`。Web schema v2 在且仅在 `perception_no_new_result` 时保留
上一份真实视觉显示，并发布 `new_result`、`display_held`、增长后的 `result_age_ms` 和姿态
偏差；真实无候选、断轨或低分结果会立即清空显示，保持值不进入 PNG。

控制侧新增 `GuidanceSetpointHold`：只允许上一条有效 setpoint 跨越异步无新结果空档，最长
等于现有 0.25 s watchdog；任一真实感知/LOS/TTC 拒绝、watchdog 超时、RC7/ARM/AUX、批准、
MSP/姿态/物理 RC gate 关闭都会清空保持并退回物理 RC。该修改不放宽 3/3/0 deg/s、
1000--1100 us 无桨包线，也不把静止目标的 `area_not_expanding` 改成有效 TTC。

MSP worker 同时增加独立防线：算法发布条件显式包含 worker 自己观察到的 OVERRIDE active。
每个成功发送帧记录发送当刻的 output enable、algorithm authorization、OVERRIDE、prefill、
physical RC freshness 和 staged command freshness。schema v4 审计只使用这些
`msp_last_publish_*` 字段核验算法帧；schema v2/v3 的算法输出日志因缺少同时间基准证据保持
fail closed，不再用当前 staged gate 对上一帧作误导性归因。

修正后的板端 log-only 先后完成两轮复验。90 s 日志
`logs/betaflight_log_20260827_181908.csv` 共 1794 行，SET_RAW_RC attempt/success 均为 0，
Web/MSP error 均为 0，最高板温 61.923 C，审计通过。180 s 日志
`logs/betaflight_log_20260827_182229.csv` 共 3590 行，SET_RAW_RC 仍为 0/0，Web error 为 0，
最高板温 66.538 C，审计通过；2406 个去重感知结果中，1961 个为
`no_detection_candidates`，445 个为 `no_tracked_output`。出现候选时 raw 最大为 1，
`tracker_high_count` 始终为 0、`tracker_low_count` 最大为 1，说明当前目标/光照下模型偶尔只给出
低分候选，ByteTrack 按设计不能用低分候选初始化新轨迹。该证据不支持直接降低阈值。

为区分“模型没有候选”“只有低分候选”和“跟踪器已关联输出”，CSV 升级为 schema v5，新增
`detector_best_score`，并继续记录既有 raw/class/high/low/output、hits、association stage、
match IoU 和 selector reason。Web schema v3 将这些字段组成可读的感知诊断区；异步显示保持仍
只作用于页面，不改变控制输入。修改后本地 `unittest` 共 227 项通过。

### schema v5 动力供电无桨复验

动力电池接入、桨叶拆除条件下又完成三组受控测试：

- `betaflight_diag_20260827_184129.csv`：179.96 s、3579 行，3215 个新感知结果；890 帧有
  候选，749 帧达到 high 阈值，766 帧形成 confirmed output。最佳候选分数按
  `<0.10/0.10--0.25/>=0.25` 分组为 90/51/749 帧；最高板温 72.076 C，RC 写入 0/0，审计
  通过。日志归档为 `logs/archives/perception_diagnostics_20260827_184129.tar.gz`，SHA256
  `4b538991ceca2633f7c9993852642d75d8eff0434cb553f7740bb85c301f3827`。
- `betaflight_noprop_timing_20260827_184741.csv`：180 s 内成功发送 6944 帧、错误 0，最大
  成功间隔 50.108 ms，最高板温 68.384 C，审计通过。该轮覆盖预填、ARM 后人工透传和 RC7
  接管无目标 fail-closed，但没有算法帧。
- `betaflight_noprop_algorithm_20260827_185318.csv`：289.93 s、10414 个成功发送、错误 0，
  含 180 个 algorithm 行和 71 个 `guidance_hold` 行。发送 A/E/T/R 范围分别为
  1499--1501/1500/989--1041/1500 us，限幅后 roll/pitch/yaw 为
  -0.225--0.232/-0.117--0.120/0 deg/s，发送当刻门禁和无桨包线均通过。审计仍因唯一类别
  `set_raw_rc_gap` 失败：最大间隔 164.302 ms。缺口发生在 armed+OVERRIDE 期间，MSP publish
  tick 本身被延迟，物理 RC age 越过 0.25 s 后按设计退回 `physical_rc_stale -> prefill`；
  没有串口 send error。

长间隔与共享进程负载相关：超过 60 ms 的 publish tick 共 105 行，其中 100 行有一个 MJPEG
客户端；无客户端时也出现过 120.728 ms。候选存在时相机读取/ByteTrack 中位耗时从
9.314/0.253 ms 增至 18.714/2.349 ms，实际 Tracker FPS 约 14.2；原配置仍按 30 Hz 尝试感知，
叠加 JPEG 编码后负载和温度上升。为做可复现对照，runner 新增
`--rknn-perception-rate-hz` 和 `--disable-web-preview`，两项均记录在 meta，不改变批准配置或
3/3/0 deg/s、1000--1100 us 控制包线。

降载基线 `betaflight_noprop_load_shed_20260827_190920.csv` 使用 15 Hz 感知并关闭 MJPEG：
233.52 s、9147 个成功发送、错误 0、最大间隔 49.769 ms、最高板温 59.153 C，审计通过。现场
未执行 ARM/RC7/目标动作，因此该轮没有算法行，不能替代带目标复验。三份控制测试及审计归档
为 `logs/archives/noprop_control_validation_20260827.tar.gz`，SHA256
`1dcd780489fa73c69f118796a9a9a94c300d6c242409ec84f5586b15838822fa`。当前放桨仍被“降载配置下
带目标 algorithm 发送间隔未通过”和 `timestamp_after_buffer` 高频拒绝共同阻断。
