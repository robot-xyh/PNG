# Betaflight src 参考与迁移记录

## 目的与安全边界

本文记录从 Orange Pi `/home/orangepi/src/circle_pilot` 参考或移植到当前 Python 工程的
非视觉能力。原目录不修改；历史 Python 实机验证保持无桨、`LOG_ONLY`。2026-07-15 新增的
Python `noprop_bench` 已完成多轮无桨实测；2026-08-27 新增的异步单 UART 调度仍须重新部署
验证，不能把代码测试当作实机验收。机器可读来源、哈希和候选参数保存在
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
|S7 Python PNG 无桨 RC 验证|完成但未放桨|多轮 ARM/RC7/动态目标 CSV；包线正确但同步 MSP 有 65--69 ms 尖峰|
|S8 异步单 UART 调度|代码完成、实机待执行|本地 244 项测试；Orange Pi 断电，尚无 schema v7 板端证据|
|S9 接管平滑与倾角包络|代码完成、实机待执行|本地 252 项测试；旧批准失效，尚无 schema v8 板端证据|

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

## 2026-08-27 延迟融合、感知进程隔离与再次无桨实控

为消除图像时间戳晚于最新姿态样本造成的系统性拒绝，schema v6 增加有界延迟融合。检测结果
最多等待 200 ms，直到姿态缓存形成前后包围样本；待处理队列上限为 8，真实无检测会立即清空
队列，超时结果仍进入原有 fail-closed 时间戳检查。Web schema v4 同步发布融合状态、等待时间、
队列深度和丢弃计数。该机制没有外推姿态，也没有放宽 watchdog、TTC 或控制门禁。

runner 新增 `--isolate-rknn-process`，使用 spawned 子进程承载相机、RKNN 和完整 ByteTrack，避免
感知负载持有 MSP worker 所在进程的 Python GIL。隔离模式必须同时使用
`--disable-web-preview`；父进程仍负责 MSP、安全状态机、导引、CSV/Web JSON 和退出顺序。子进程
忽略终端 SIGINT，通过父进程 stop event 清理，现场退出未再产生子进程 traceback。

板端验证结果如下：

- `betaflight_noprop_isolated_perception_20260827_200055.csv`：411.39 s、16058/16058 次成功发送、
  错误 0，最大成功间隔 49.714 ms，审计通过；覆盖持续真实目标跟踪，但未 ARM/RC7。
- `betaflight_noprop_isolated_algorithm_final_20260827_201202.csv`：552.85 s、21167 次成功发送、
  错误 0、1262 个 algorithm 行；唯一审计违规为一次 65.964 ms 发送间隔。
- `betaflight_log_20260827_202613.csv`：动力电池、拆桨、15 Hz 隔离感知下运行 1548.36 s，
  60284/60284 次成功发送、错误 0、522 个 algorithm 行。ARM、RC7、`ACTIVE/algorithm`、目标
  无效后 `FAILSAFE/passthrough` 及“先 RC7 人工、再 DISARM”均由日志确认。实际 A/E/T/R 范围
  为 1497--1503/1498--1502/990--1078/1500 us，最高温度 68.384 C。审计仍仅因一次
  68.568 ms 发送间隔失败；当时为 armed+OVERRIDE 下的 `DEGRADED/passthrough`，没有 RC 错误、
  885 us 或算法包线越界。

静止目标可维持 confirmed ByteTrack，但因面积不扩张会触发 `area_not_expanding` 或
`ttc_out_of_range`，因此不应期待持续算法输出。目标靠近且横移时产生有效 TTC 和 R/P/T 命令；
ID 4、8 分别连续记录 2531、1702 个主循环，后续 ID 变化与目标出框、顶部裁切和重新进入对应，
不能仅凭页面瞬时 ID 认定 ByteTrack 自发跳变。

延迟融合已消除本轮的 `timestamp_after_buffer` 主导拒绝，感知进程隔离也把此前百毫秒级缺口
降为单次 65--69 ms 尖峰。剩余瓶颈是 115200 baud 单 UART 上同步 SET_RAW_RC 与 STATUS、
RAW_IMU、RC、ATTITUDE、ANALOG 轮询共享时隙；本轮各命令最坏 RTT 为 28.695--36.143 ms。
在提高并验证 MSP baud、降低/重排轮询预算、改用独立遥测链路或重新定义可证明的发布频率前，
不得安装桨叶。三轮证据已归档为
`logs/archives/noprop_isolated_algorithm_validation_20260827.tar.gz`，SHA256 为
`49a55c3cd3641a30188cac24584bbd2910ffc3f47a9e2a8cbd0eae84f3f407b9`。修改后本地
`python3 -m unittest discover -s tests -v` 为 234/234 通过。

## 2026-08-27 异步单 UART MSP pipeline（离线实现）

历史 C++ `src/circle_pilot` 的关键设计是 `writeRawRc()` 写后即返、持久 RX buffer、每周期先
SET 后查询并限制响应 drain 时间。Python 旧实现虽然有独立 worker，仍对每个 SET_RAW_RC 和
遥测请求同步等待响应，实机 RTT 13--36 ms 时会直接拉长 50 Hz 发送间隔。本次按来源架构迁移，
但没有复制其单一 SET 时间戳缺陷，而是用 FIFO 为每个 SET 写入保存独立 request ID 和时间。

实现分层如下：

- `BetaflightMSPAdapter` 保留同步 `request()` 供启动 identity、版本和 BOXID 探测；运行时切换
  `async_pipeline` 后串口 timeout 为 0，SET 写入不调用 `flush()`、不等待 ACK。
- RX parser 使用跨周期 byte buffer，支持分片/粘包/jumbo 帧，遇噪声或坏 checksum 后逐字节
  重同步；遥测按 command 保证最多一个未决请求，SET ACK 按 FIFO 返回原 request ID。
- worker 每 20 ms 先写最新 SET，再排入最多一个到期 STATUS/ATTITUDE/RAW_IMU/RC/ANALOG，
  最后 drain 最多 3 ms；阻塞后从当前完成时间重新排期，不突发补发遗漏周期。
- 当前预算从历史 5/10/20/10/2 Hz 调整为 5/10/5/10/1 Hz。OVERRIDE active 且已有有效人工
  锁存后暂停 RC 请求，STATUS 报告关闭后恢复，避免 885 us 回读既占带宽又污染人工基线。
- `prefill_success_count` 只由人工透传 ACK 增加，批准配置仍要求 10 帧。最后 SET ACK 超过
  250 ms 时主安全状态机和 worker 双重阻断算法，worker 继续发送人工透传以支持恢复。
- 当前 RC7 同时是 OVERRIDE 和 AUX 控制许可；配置显式声明
  `satisfied_by_override_mode=true`，避免暂停 RC 查询后用切换前 RC7 低位误判 `aux_disabled`。
  独立 AUX 方案不得启用该声明。

CSV 升级为 schema v7，区分 SET write attempt/success/error 与 ACK count/age/pending，记录真实
write interval、max、平均 Hz、P50/P95/P99/P99.9，以及 RX discarded/checksum/parser error 和
发送当刻 ACK gate。Web schema v5直接显示这些字段。审计 v7 以写成功时间而不是同步 ACK
完成时间验收：写入率至少 49 Hz、P99.9 gap 不超过 40 ms、max gap 不超过 60 ms、ACK stall
不超过 250 ms，且 885 us、写错误、parser/checksum error 和算法门禁违规均为 0。

新增伪串口测试覆盖分片/粘包、噪声和坏 checksum、延迟 ACK 不阻塞 SET、SET 优先、单未决
遥测、ACK stale 降级、OVERRIDE 后 RC 暂停和错过周期不突发。完整本地测试为 244/244 通过。
实现期间 Orange Pi 按用户要求断电，未复制文件、未修改 Betaflight baud、未发送 RC；下一次
实机工作先在 `/dev/ttyS1@115200` 做 log-only 和无桨 schema v7 复验，再比较 230400。只有两种
波特率均不满足门限时，才设计 FC UART4 到 Orange Pi `/dev/ttyS7` 的双适配器 fallback。

## 2026-08-27 接管平滑与倾角包络（离线实现）

本阶段借鉴 `circle_ai_strike` 的
`core/include/circle/strike_png/entry_handoff.hpp` 和
`core/src/strike/modules/tilt_envelope.cpp`，没有修改 LOS/TTC、完整 ByteTrack、MSP ownership、
ID 50/mask 15 或 3/3/0 deg/s、1000--1100 us 无桨包线。命令链路明确为：

```text
guidance -> GuidanceSetpointHold -> entry handoff -> tilt envelope
         -> Betaflight Rate inverse -> PWM slew -> MSP_SET_RAW_RC
```

接管上升沿捕获新鲜 `MSP_RAW_IMU` roll/pitch 角速度，超过 0.25 s 则以零为起点；0.8 s 内使用
`h=u^2(3-2u)` 从起点混合到导引目标。该层不改 yaw/thrust，worker 原有油门交接保持唯一。
门禁关闭或 setpoint 无效会复位，下次接管重新捕获，不沿用上次状态。

倾角包络先对“继续外倾”的命令施加线性 soft factor，再以
`w=smoothstep((|att|-max)/margin)` 混合到限幅的 `-kp*attitude` 回平 rate。正负 roll/pitch 对称，
向内命令不受软限幅；达到 `max+margin` 后必须输出反向回平命令。无桨配置启用
`35 deg/10 deg/5 deg/kp 3/max 3 deg/s`，姿态缺失时 fail closed。参考 C++ 的可选 LPF/jerk
没有迁移，避免与下游现有 PWM slew 重复整形。

CSV 升级为 schema v8，保留 `sp_*` 表示最终整形后 setpoint，并增加整形前 R/P、接管起点/
来源/进度、姿态、soft factor、level weight、hardcap 和拒绝原因。Web schema v6 在只读页面显示
同一组数据。离线审计兼容 schema v2--v8，并在 v8 检查有限数、`[0,1]` 因子、算法行整形有效
以及硬区输出方向。无桨批准生成器要求两项保护开启，并限制 handoff、gyro age、倾角几何和
最大回平 rate。

本阶段仅在工作站离线实现并运行测试；Orange Pi 保持断电，没有串口写入或硬件结论。无桨
配置哈希已经改变，旧 approval 必然失效。下一次板端工作必须重新采集 CLI 快照、生成批准，
再完成 schema v8 的 log-only、prefill、ARM、动态目标、RC7 接管、软/硬倾角和人工退出流程。
通过前继续保持拆桨，且不更新 S8/S9 为实机完成。当前完整本地回归为 252/252 通过。

## 2026-08-27 Orange Pi 信号供电只读复验

局域网同时发现 `192.168.124.42` 和 `192.168.124.48` 两台 SSH 主机。仅 `.42` 的 ED25519
指纹与历史 `10.168.1.103/.42` 部署记录一致；连接后又以 `orangepi5max`、`aarch64` 和
`/home/orangepi/png_betaflight_python` 三项确认目标。`.48` 只做端口和公钥扫描，未登录、未复制
文件、未访问串口。

在只有信号电、没有功率电的条件下，将当前公共代码、测试、示例配置和文档同步到 `.42`，不
覆盖硬件专用 `*.local.json`。同步后关键运行文件 SHA256 与工作站一致，板端 Betaflight 聚焦
测试为 94/94 通过。随后先备份
`logs/deployment_backups/20260828_110604/betaflight.rk3588.noprop.local.json`，再只合并异步
MSP、5 Hz RAW_IMU、ACK stale、entry handoff、tilt envelope 和 RC7 mode gate；串口、相机、
模型、Web、rate gain 和无桨 RC 包线保持不变。配置 SHA256 从
`bd0949414405cb0c44f530b925d50fee33a00299b5d167099e751fefeed3907f` 变为
`e93536557aef4874e16edde43719ff9e708bc5518e6a422aa488264b8139e4c7`，旧 approval 按设计报告
`parameters_sha256_mismatch`。

90 s 运行固定使用 `--control-mode log_only`，未传 `--allow-control`。日志
`logs/betaflight_log_20260828_110650.csv` 为 schema v8，共 1791 行、89.955 s；审计
`passed=true`、violations 为 0。SET_RAW_RC attempt/success/error 均为 0，MSP request、worker、
RX discard/checksum/parser 和 Web error 均为 0，遥测/姿态无 stale 行，相机 failed frame 为 0。
RKNN 总耗时 P50/最大为 6.122/13.191 ms，最高板温 45.307 C。主循环中的 459 个
`perception_no_new_result` 是 20 Hz 主循环等待 15 Hz 感知结果，不是相机失败；本轮没有目标
轨迹，因此 entry handoff 和 tilt hardcap 均未触发，不能作为 S9 动态接管验收。完整证据归档为
`logs/archives/signal_only_schema8_20260828_110650.tar.gz`，SHA256 为
`20acf61b946527f871e04b9f3dffb73931d10210b7e1b1f5debd8d512d8ddeca`。

## 2026-08-28 动力电 LOG_ONLY 与相机标定门禁

仅连接确认过的 `192.168.124.42/orangepi5max`；未访问局域网中的 `.48`。四桨保持拆除，RC5
为DISARM、RC7为人工侧，运行90 s `LOG_ONLY + RKNN/ByteTrack`，未传 `--allow-control`。
`logs/power_logonly_intrinsics_check_20260828_132320.csv` 共1788行、89.957 s，审计通过且0违规。
电池24.8--24.9 V、电流0.04--0.19 A、最高温度55.461 C；MSP worker、RX parser/checksum、
相机和Web错误均为0，SET_RAW_RC attempt/success/error均为0。证据归档为
`logs/archives/power_logonly_intrinsics_check_20260828_132714.tar.gz`，SHA256为
`6dbf10667469baa71f307790e010bdde807eb1e5610d28b977cc1654460efa0c`。

1601个新感知结果中338帧有YOLO候选、329帧为confirmed ByteTrack；两段连续轨迹分别为123和
206帧，只有3次有/无目标边沿，不是逐帧ID抖动。其余1251帧为`no_detection_candidates`。
329个目标帧全部`fusion_status=processed`；`detection_attitude_offset_ms`中位约-70 ms、最差
-212 ms，表示最新姿态样本晚于历史图像并可用于插值，不等同于曝光同步误差。实际曝光时刻
仍使用`capture.read()`返回后的单调时间近似，尚无V4L2硬件曝光时间戳。

代码复核发现当前实机配置只有`pitch_up_deg=90`。在OpenCV相机`x右/y下/z光轴`和Betaflight
FRD`x前/y右/z下`定义下，旧矩阵把中心光轴映射到机体`+X`，相对上方`-Z`偏差90 deg。本次
新增严格外参门禁：矩阵必须有限、正交且det=+1；启动meta记录三根相机轴、上视误差、来源、
验证状态和时间戳来源；任何RC输出在打开串口前要求显式`R_BC`、FRD/OpenCV声明、人工验证和
不超过阈值的上视误差。无桨批准工具执行同一校验并把诊断写入批准文件。示例配置保持
`verified=false`，不得在未完成实物画面方向测试时生成新批准或发送RC。

公共文件部署包SHA256为
`725fc5469036a7d77c80de6ba4f561cb710a28632e2223db3b0482318ac1edaa`；覆盖前备份位于
`logs/deployment_backups/20260828_133958/camera_extrinsic_public_files_before.tar.gz`，SHA256为
`cef88afa3bfda04ca6789c7790ebfe2eb8c73ce5141685a43fe77691e6da714c`。板端几何、日志和批准
工具共27/27项测试通过，硬件local配置哈希仍为`e9353655...139e4c7`。随后30 s LOG_ONLY产生
`camera_guard_logonly_20260828_134141.csv`，共599行、审计0违规、相机/MSP parser错误和所有
SET_RAW_RC attempt/write均为0；meta实测报告`legacy_pitch_up_deg/90 deg/control_ready=false`。
同一配置带`--allow-control`时以退出码1在打开`/dev/ttyS1`前拒绝，串口保持释放。证据包为
`logs/archives/camera_extrinsic_guard_20260828_134345.tar.gz`，SHA256为
`24b3b20e64e6206d1b5b45b78900ed94748759ad78d231ef0fdd6bd2698e8933`。

## 2026-08-28 上视相机轴向与 Betaflight 姿态符号实测

仅使用`.42/orangepi5max`，保持拆桨、DISARM、RC7人工侧和`LOG_ONLY`。目标沿机头移动时图像
`+v`，沿机右移动时图像`+u`，得到OpenCV到FRD候选矩阵
`R_BC=[[0,1,0],[1,0,0],[0,0,-1]]`；其光轴映射到机体`-Z`，正交误差为0且det=+1。三份轴向
日志归档为`logs/archives/extrinsic_axis_labeled_20260828_140724.tar.gz`，SHA256为
`4efc2634b4501ea2d0db79dbd4ebd61aa76d079cba134557da84035b733dd462`。矩阵先以
`extrinsic_validation.verified=false`写入板端配置，旧approval继续失效。

墙面固定测试使用偏左但稳定的静态目标。基线`extrinsic_wall_center_check_20260828_150535.csv`
末6秒bbox中心为`(153.41,199.81)`，像素标准差`(0.43,1.04)`，姿态约roll/pitch=`1.3/1.5 deg`。
抬机头后`extrinsic_wall_pitch_up_20260828_151058.csv`中MSP pitch为`-12.9 deg`、bbox v为
`328.44`；直接使用MSP pitch使惯性LOS相差`29.4 deg`，仅将pitch取反后降至`2.36 deg`。
右侧下降约10 deg的`extrinsic_wall_right_roll_20260828_151506.csv`中MSP roll为`+11.6 deg`；
保留roll符号时惯性LOS残差`4.77 deg`，取反会恶化到`20.64 deg`。三轮审计均通过，所有
SET_RAW_RC attempt/write计数均为0。

因此`AttitudeTelemetry`保留原始MSP显示值，同时定义FRD角为
`(roll,-pitch,yaw)`；`R_IB`和倾角包络使用FRD角。修正部署后必须重新做静态基线、抬机头和
右滚LOG_ONLY，确认惯性LOS不再出现约两倍角误差后，才能把外参标记为verified。RAW_IMU pitch
rate的符号尚未通过动态转动日志确认，entry handoff继续是待验证项，不能从静态姿态试验推断。

修正后板端定向测试40/40通过，完整工作站回归255/255通过。新基线
`extrinsic_fixed_baseline_20260828_152608.csv`为roll/pitch/yaw=`1.2/1.6/5.0 deg`；与新右滚日志
`extrinsic_fixed_right_roll_20260828_152236.csv`比较，roll变化`10.4 deg`且yaw相同，滤波后
惯性LOS仅相差`0.79 deg`。新抬头日志`extrinsic_fixed_pitch_up_20260828_152928.csv`中MSP pitch
变化`-10.7 deg`、bbox v变化`117.6 px`；滤波后惯性LOS相差`4.53 deg`，该次摆放同时产生
`10 deg` yaw变化，固定为基线yaw重新计算时残差为`1.74 deg`，旧pitch符号则为`25.03 deg`。
这些结果关闭了相机轴向和MSP静态姿态pitch符号缺口，但不关闭yaw磁航向精度或RAW_IMU动态
rate符号缺口。

板端local配置随后仅把`extrinsic_validation.verified`改为true，配置SHA256为
`ac6fbec6e95dd2d931b8eb2ca30ad4ad77dd2cb89125992c4cb65a00d7b0e31c`；修改前备份位于
`logs/deployment_backups/20260828_153726/`。`extrinsic_verified_logonly_20260828_153744.csv`
启动报告`source=R_BC`、`verified=1`、光轴误差0和`control_ready=1`，审计0违规且全部RC写计数
为0。新快照为`logs/betaflight_snapshots/betaflight_snapshot_20260828_153855/manifest.json`；新
无桨批准文件SHA256为`3286c626f5a73f844468c568771aa642daf51509444dbfb4606b3b058918bed3`。
完整证据包为`logs/archives/extrinsic_attitude_validation_20260828_154112.tar.gz`，SHA256为
`fd489279211862b4e58887d585a54be2cd8cb1299a6e372329430dd23d033d22`。该批准只适用于拆桨台架，
不构成系留或有桨飞行许可。

## 2026-08-28 RAW_IMU 动态语义与 circle 对照

拆桨、DISARM、RC7人工侧下使用临时只读配置将ATTITUDE/RAW_IMU请求提高到各20 Hz，执行三组
抬头/回正和右滚/回正动作。`imu_axis_dynamic_20hz_20260828_155837.csv`共3660行、74.99 s，
roll/pitch跨度为21.3/26.2 deg，审计0违规且全部RC写计数为0。按独立sample age恢复采样时刻后，
FRD roll角差分与gyro X相关系数为+0.973，FRD nose-up pitch角差分与gyro Y相关系数为-0.972，
确认raw轴符号为`(+X,-Y)`。逐个动作积分得到roll raw/角度比15.72--16.68、pitch比
15.92--16.27 raw unit/deg。

该结果与官方Betaflight 4.4/4.5 `MSP_RAW_IMU`序列化调用`gyroRateDps(i)`的语义不一致，不能在
不知道MICO定制固件实现和Blackbox比例的情况下硬编码`1/16.4`。进一步检查已飞行的
`/home/orangepi/src/circle_pilot`发现其MSP客户端只请求STATUS/ATTITUDE/RC，不读取RAW_IMU；
运行时明确把vehicle body rate留空，PNG entry handoff实际从零rate开始。circle也直接保存MSP
pitch显示值，因此只借鉴其“零rate接管”策略，不复制其姿态符号。

Python据此升级日志schema v9：载荷改记`gyro_msp_raw_x/y/z`，无可信转换时旧
`gyro_roll/pitch/yaw_deg_s`留空；Web图表明确显示MSP raw。`entry_handoff.rate_source`默认且无桨
批准强制为`zero`，runner拒绝`gyro`，即使输入新鲜的大幅raw值也从零平滑进入。未来只有取得
当前固件源码/协议说明并用同一时段Blackbox gyro完成比例、符号和时延交叉验证后，才能新增
显式校准对象并开放measured-rate handoff。

## 2026-08-28 circle 零速率交接部署与 schema v9 复验

工作站完整回归256/256通过后，将23个公共代码、测试、示例配置和文档文件同步到已确认的
`.42/orangepi5max`，不覆盖板端硬件local配置。部署包为
`png_betaflight_circle_reference_20260828.tar.gz`，SHA256为
`a8a5668fca2b85deeb42afa58b898a2284a08050e72334bfdfcabb0d571008ac`；覆盖前配置和部署包备份
位于`logs/deployment_backups/20260828_162939/`。板端聚焦回归104/104通过。

随后仅在`guidance_command.entry_handoff`中显式加入`rate_source=zero`，保留0.8 s时长、原串口、
相机、检测器、外参、rate映射和无桨PWM包线。local配置SHA256变为
`02944e32a03669cfbbab936d531ee889c30f9e823b2a3d8987f8697c94f19ce7`，旧批准文件按设计因参数
哈希不匹配而失效。30 s只读日志
`schema_v9_circle_zero_20260828_163311_20260828_163311.csv`共598行，597行具有
`gyro_msp_raw_*`，未经验证的`gyro_*_deg_s`全部留空；状态始终为`LOG_ONLY`、publish始终为
`disabled`，SET_RAW_RC request/write/success/ACK均为0，离线审计通过且0违规。

新飞控快照为
`logs/betaflight_snapshots/betaflight_snapshot_20260828_163720/manifest.json`，SHA256为
`7949b2a6bfcb1f8e3bfc5ae6667e4afdeb7379fd86cd5bf892caffc36f0f2696`。据此生成的新无桨批准文件
SHA256为`a8f6351b2029a5dff2950052e67d602c985283e25bf6e4aa6def1b90b7c62aae`，其scope仍为
`noprop_bench`。批准后再运行10 s `LOG_ONLY`，启动明确报告`approved=1`；
`schema_v9_approved_logonly_20260828_164027_20260828_164027.csv`共199行，审计0违规且RC写入仍
全部为0。程序退出后无残留进程，`/dev/ttyS1`已释放。

本阶段完整证据位于
`logs/deployment_archives/20260828_circle_reference_zero_handoff/`，其`SHA256SUMS`文件SHA256为
`00ea7ee4573a222fbb8ee2a37415fe973bfe132100aaa8766a5d5bfc6832826f`。本阶段只证明circle零rate
策略、schema v9语义、批准绑定和只读链路正确；未发送RC，也不构成有桨或飞行批准。下一阶段
仍需在拆桨条件下分别完成人工侧启动/prefill、ARM怠速、稳定目标、RC7接管、目标动态移动、
RC7退出和DISARM，并以schema v9日志验收实际rate/PWM方向、交接连续性和看门狗降级。

## 2026-08-28 MSP 热路径与 CPU 分区复验

人工侧45 s纯MSP基线最初从49.3 Hz逐步降至31.5 Hz，证明不是固定UART带宽上限。代码检查定位
到worker每个发布周期构造完整统计快照、反复排序不断增长的间隔样本，并在迟到后按完成时刻
重置下一周期。现已改为轻量读取最近ACK/write时间、写入时增量维护有序分位数、固定时基跳过
遗漏周期，同时保留16 ms最小间隔，避免阻塞恢复后追赶式连发。板端聚焦测试59/59、工作站完整
回归259/259通过。

修复后的`transport_incremental_fix_20260828_190022_20260828_190023.csv`审计0违规：平均
49.799 Hz、最大28.700 ms、P99.9 28.015 ms、deadline miss为0。完整隔离RKNN但CPU不分区的
`isolated_hotpath_fix_20260828_190321_20260828_190323.csv`仍只有38.148 Hz、最大159.706 ms，
并出现3行`physical_rc_stale`，证明进程隔离本身不等于调度隔离。

runner新增`--main-cpu-affinity`和`--rknn-cpu-affinity`，支持列表/范围，拒绝重叠并在无法应用时
fail-fast。最终不依赖外部`taskset`的
`isolated_explicit_affinity_20260828_192131_20260828_192132.csv`明确应用main `[6,7]`、detector
`[4,5]`；完整RKNN+ByteTrack下平均49.955 Hz、最大29.578 ms、P99.9 27.035 ms、RC最大年龄
145.610 ms，deadline miss及发送/请求/checksum/parser错误均为0，审计0违规。测试全程RC5
DISARM、RC7人工侧，仅发送锁存人工透传值，未产生algorithm帧。

证据目录为`logs/deployment_archives/20260828_msp_affinity_validation/`，其中`SHA256SUMS`的
SHA256为`9ba8f9ce04bc993daf196609a45257a43b95c5eef0d792c8076fede9e75d2dbc`；压缩包
`20260828_msp_affinity_validation.tar.gz`的SHA256为
`e99af736d761996a6fda8bb74258ba080a6b83ce270a041fdeb940c46d2eb8eb`。退出后无残留runner/RKNN
进程且`/dev/ttyS1`已释放。下一步仍是拆桨条件下完成ARM怠速、动态目标、RC7 ACTIVE、算法
PWM方向/连续性、RC7退出、DISARM和断流故障注入；本节不构成有桨或飞行批准。

## 2026-08-28 Betaflight 固定 Vm PNG 接入

Betaflight Python入口此前固定使用TTC增益调度，不能复现实验报告中的`fixed_vm_png`。现增加显式
`guidance.law`选择，缺省仍为`ttc_png`；固定速度模式严格计算
`a_cmd = N * Vm * (omega_LOS x lambda_I)`，再按向量模长限制到
`max_guidance_accel_mps2`。该模式只旁路TTC有效性，不旁路LOS、检测、姿态同步、watchdog、倾角
包络、RC命令限幅、MSP Override或批准清单。`N`、`Vm`和模长上限必须为有限正数，未知导引律
或缺项会拒绝启动。

无桨批准工具同步执行同一类显式校验，并额外限制
`max_guidance_accel_mps2 <= 1.0`。批准JSON记录`law/N/Vm/N*Vm`和TTC是否必需；因此以后修改
导引律或增益不仅会触发整份配置SHA256失配，也会在重新批准时接受独立的参数完整性检查。

日志schema升级到v10。CSV逐行新增`guidance_law`、`guidance_navigation_constant`、
`guidance_fixed_vm_m_s`、`guidance_fixed_gain`、`guidance_max_accel_mps2`和
`guidance_ttc_required`；meta和`run_start`事件保存同一组参数。Web页面显示导引律和`N*Vm`，在
固定Vm模式把TTC门明确显示为`BYPASS`，不能误读成TTC有效。公式、TTC旁路、LOS拒绝、模长限幅、
配置拒绝和网页字段均有单元测试，工作站完整回归267/267通过。

板端测试必须复制独立`betaflight.rk3588.noprop.vm.local.json`并使用独立
`betaflight_noprop_vm_approval.json`；不得原地修改已批准的TTC配置。首轮参数仅用于拆桨验证：
`N=3.0`、`Vm=1.0 m/s`、`max_guidance_accel_mps2=1.0`，下游仍受roll/pitch 3 deg/s、yaw 0和
1000--1100 us无桨包线约束。静止目标的预期结果是LOS和guidance有效、TTC可无效、VM命令接近
零；移动目标才应产生有符号的小幅roll/pitch候选命令。

板端部署确认主机为`.42/orangepi5max`且部署前无残留控制进程。原TTC local配置SHA256保持
`02944e32a03669cfbbab936d531ee889c30f9e823b2a3d8987f8697c94f19ce7`；独立VM配置SHA256为
`5c140d1d71fecaf6b18912056309aa756751435adbb7303b7476e1642825189b`，独立批准文件SHA256为
`026dbab0bf001fc1f82a8408d3d75670d10d236cc57fc8d1ec0192037a2c8056`。板端VM、runner和Web聚焦
测试33/33通过。

真实相机只读日志`fixed_vm_logonly_20260828_215642.csv`共598行，全部为
`fixed_vm_png/LOG_ONLY/disabled`，SET_RAW_RC attempt/write/success均为0且MSP错误为0；画面未
形成confirmed track。为单独验证导引集成，使用固定80x80面积、同一track ID、每50 ms横移2 px
的100帧CSV回放。`fixed_vm_controlled_replay_20260828_220743.csv`共140行，其中97行guidance
有效且TTC因`area_not_expanding`无效；逐行重算`3*(omega_LOS x lambda_I)`的最大误差为
`2.22e-9`，`|g_eval|`最大0.230，roll/pitch候选范围分别为0--0.226和-0.013--0.056 deg/s，
RC写入仍为0。两份日志及长时间相机预览日志均通过无桨审计、0违规。

抓取的640x512预览仅包含天花板、灯具和室内结构，没有无人机目标；实时遥测中有2个低分候选，
最高约0.088，低于跟踪阈值，故`no_tracked_output`正确。证据目录为
`logs/deployment_archives/fixed_vm_20260828_214918/`，归档包SHA256为
`5e63aca6db0776e39588b894442e313fbcb9f95e80aa523ec9881fe3bc771c3b`。下一步必须在该相机画面内
放置模型类别的单个无人机目标，先做真实`LOG_ONLY`静止/横移测试；当前结果不批准RC7接管，
更不构成有桨飞行验证。

## 2026-08-29 固定 Vm 真实目标与合成手持目标回放

RK3588恢复后已同步固定Vm实现并生成与VM配置绑定的新快照和批准文件。真实目标静止阶段使用
`fixed_vm_real_target_logonly_20260829_104830.csv`记录：同一`track_id=9`可持续确认，20 s抽样中
目标有效率98%，LOS质量均值0.992；roll/pitch候选角速度绝对值最大分别约0.009/0.005 deg/s。
系统始终为`LOG_ONLY`，RC5保持DISARM、RC7保持人工侧，SET_RAW_RC attempt/write/success均为0。

手持横移暴露出测试输入不可重复：约280--330 s内目标横跨约101--565 px，但发生多次丢失和重新
建轨，累计tracker switch为8、fragment为15；候选roll同时出现正负号且保持在无桨包线内。这一结果
只能证明方向响应存在，不能作为ByteTrack连续性验收，也不能据此进入RC7接管。完整CSV无桨审计
通过、0违规，SET_RAW_RC和MSP解析错误均为0。证据已加入
`logs/deployment_archives/fixed_vm_online_20260829_104631/real_target_handheld/`；刷新后的归档包SHA256为
`59a69be2141688eb392138f4683742f941e2f9c4f696c31c3c9f9850adb6ae5b`。

为消除手持速度、姿态和出框误差，使用内置图像生成工具分别生成透明底视“单手握持四旋翼”前景
和无目标室内天花板背景，保存于`tools/assets/betaflight_synthetic_handheld/`。新增
`tools/generate_betaflight_handheld_sequence.py`，以固定随机性为0的解析轨迹合成640x512、20 FPS、
30 s视频：前3 s静止，中间20 s按10 s周期左右往返并叠加6 px纵向晃动和正负8%尺度变化，后7 s
静止；同时输出逐帧真值框和参数/素材SHA256。该生成器只产生测试输入，不调用飞控或控制代码。

`tools/run_rknn_bytetrack_video_eval.py`逐帧调用与实机相同的RKNN桥和完整ByteTrack。板端实际加载
`drone_v8n_v21_kd_relu_lambda008_640_640-rk3588.rknn`，600/600帧均有YOLO候选，598/600帧在最初
3帧确认门后形成可用测量；全程仅`track_id=1`，switch=0、fragment=0，高阶段关联IoU均值0.990、
最小0.932，检测中心与合成真值横坐标相关系数0.999986。置信度均值/最小值均为0.835，RKNN推理
均值11.83 ms、最大15.90 ms。

598帧检测结果随后以CSV送入批准的固定Vm配置，日志
`fixed_vm_synthetic_handheld_replay_20260829_20260829_111249.csv`共619行，595行guidance有效。
逐行重算`3*(omega_LOS x lambda_I)`最大误差`2.19e-9`，`|g_eval|`最大0.676，低于1.0上限；移动段
roll候选范围为-0.651--0.656 deg/s，四个左右半程均随运动方向正确反号，像面横向速度与roll相关
系数最高0.994。停止后稳定段roll绝对值最大0.00195 deg/s。运行始终为
`LOG_ONLY/disabled/armed=0`，SET_RAW_RC attempt/write/success、MSP checksum/parser/request/send错误
均为0，无桨审计通过、0违规。

合成序列证明当前模型能识别该类底视手持无人机，并证明ByteTrack连续性和固定Vm方向/公式在可控
输入下正确；它不经过真实相机、镜头畸变、曝光、运动模糊和真实目标域，因此不能替代实物测试，
也不批准RC7接管或有桨飞行。全部素材和结果位于
`logs/deployment_archives/fixed_vm_online_20260829_104631/synthetic_handheld/`；最终归档包
`fixed_vm_online_20260829_104631.tar.gz`的SHA256为
`fb2a434fffe14fd6e8a703751e22e8e3583d855855a66e9963514adcee4e5c71`，其`SHA256SUMS`文件SHA256为
`e5a9819d65a927a25b0ceb6eeffa6aa78531e435de2403e21e356bfa29b5d860`。退出后无残留runner/RKNN
进程，`/dev/ttyS1`已释放。

## 2026-08-29 真实相机手持双轴复测与 pitch 映射阻断

使用`.42/orangepi5max`、真实上视相机和实物无人机目标运行固定Vm入口，配置为`N=3`、
`Vm=1 m/s`、`max_guidance_accel_mps2=1`。全程拆桨、RC5 DISARM、RC7人工侧和`LOG_ONLY`；日志为
`fixed_vm_real_handheld_camera_20260829_repeat_20260829_144654.csv`。静止区间38.16--107.52 s内
1304/1304个新感知结果均有效，只有`track_id=5`且无切换，检测置信度均值0.792、匹配IoU均值
0.960。该区间证明真实目标在稳定输入下可以连续识别和跟踪。

横移区间252.727--304.026 s内YOLO为990/990，ByteTrack输出984/990，导引有效983/990；输出始终
为`track_id=16`。空窗包括两个单帧、一个3帧约0.10 s及区间边界单帧，未在连续输出中换ID。
目标中心横向跨度426.5 px，横向像速与roll候选指令的最佳相关系数为+0.938（约0.30 s滞后）；
roll范围为-0.996--+0.452 deg/s。该结果通过横向LOG_ONLY方向响应，但约0.14 s平均感知结果年龄
仍需在控制包线中计入。

纵移区间402.125--447.889 s暴露出两个独立阻断项。YOLO为854/854，但ByteTrack仅输出788/854，
导引有效784/854，输出ID依次出现42、55、58、59，最长跟踪空窗1.357 s；因此当前纵移速度和
尺度组合超出已验证的关联包线。与此同时，图像纵向像速与`g_eval_x`的最佳相关系数为+0.956
（约0.25 s滞后），说明`R_BC=[[0,1,0],[1,0,0],[0,0,-1]]`下的LOS和固定Vm导引链路按预期把
图像`y`运动映射到机体前向分量。当前板端`rate_gain_matrix`却使用roll行`[0,1,0]`、pitch行
`[0,0,1]`，使pitch取自`g_eval_z`；实测纵向像速与pitch候选相关系数仅-0.141。该矩阵不能用于
RC7接管。

下一步必须先在独立LOG_ONLY候选配置中把pitch来源改为`g_eval_x`，即仅比较`[+k,0,0]`和
`[-k,0,0]`，再通过拆桨单轴符号试验确定Betaflight pitch杆方向；未经该试验不能猜测符号或
更新控制批准文件。ByteTrack还需对纵移区间的尺度变化、IoU门限和短时失配单独调参并复测。
本轮完整日志无桨审计`passed=1`、0违规，SET_RAW_RC attempt/write/success和MSP请求/worker错误
全部为0；退出后无残留runner/RKNN进程，`/dev/ttyS1`已释放。证据位于归档内
`real_camera_handheld_repeat/`，更新后的`fixed_vm_online_20260829_104631.tar.gz` SHA256为
`d29b27f838e895716f55fedeb8d7c2cc55ad4ff294f517e3c80d0cb121941981`，归档内`SHA256SUMS`文件
SHA256为`a0fe31aa18ffd0a52fb4ffd13365efff054728abefc4449bd85abc475600677e`。

## 2026-08-29 固定 Vm 导引量坐标系修正

纵移复测确认`FixedVmPngGuidance`输出的`g_eval`属于惯性NED系，而旧实现直接将其送入按机体FRD系
定义的`rate_gain_matrix`。这会让飞行器姿态参与导引后产生坐标语义错误；旧候选矩阵的pitch行
`[0,0,1]`还错误地使用了惯性垂向分量，无法代表图像纵向运动对应的机体前向导引量。

运行时现保留每个视觉结果曝光时刻的`R_IB`，并在生成候选角速度前计算
`g_B = R_IB^T g_I`。配置必须显式声明
`guidance_command.guidance_eval_frame=inertial_ned`和
`guidance_command.rate_gain_input_frame=body_frd`；缺失或不匹配时运行入口、批准工具均拒绝继续。
`guidance_eval_to_setpoint()`在姿态无效时也改为fail-closed，不再使用单位阵静默回退。

无桨候选矩阵改为`[[0,1,0],[1,0,0],[0,0,0]]`：roll继续读取机体左向分量，pitch改为读取机体
前向分量，yaw保持禁用。pitch正负号当前只是LOG_ONLY候选，必须通过后续拆桨单轴方向试验比较
`[+k,0,0]`和`[-k,0,0]`后才能批准；本次修改不授权RC7接管。

CSV日志模式升级到v11，新增`guidance_eval_frame`、`rate_gain_input_frame`及
`g_eval_body_frd_x/y/z`，同时保留惯性系`g_eval_x/y/z`，便于逐行复算姿态旋转和矩阵输入。Web
遥测模式升级到v8并同时显示两组向量。无桨审计对v11日志校验坐标系声明和机体系字段，阻止旧
语义日志被误判为合格。相关聚焦测试72/72通过，完整单元测试275/275通过，且
`git diff --check`通过。

## 2026-08-29 机体系 pitch 来源真实相机复测

在`.42/orangepi5max`上备份旧运行文件后同步坐标系修正，未覆盖原
`betaflight.rk3588.noprop.vm.local.json`及其批准文件。新建独立
`betaflight.rk3588.noprop.vm.body_frame_logonly.local.json`，SHA256为
`f42f6e5ee739673656f4c704b445f626f4da00574451c342e9f7a27c2d526173`；其矩阵为
`[[0,1,0],[1,0,0],[0,0,0]]`，并声明`inertial_ned -> body_frd`。板端聚焦测试72/72通过。

真实上视相机纵向往返日志为
`fixed_vm_body_frame_longitudinal_logonly_20260829_152437.csv`。主要动态窗口340--390 s内，
970个新感知结果均有YOLO候选，ByteTrack输出和导引有效各944个，全部为`track_id=5`；检测分数
均值0.729、关联IoU均值0.949。目标中心纵向跨度328.91 px，图像纵向速度与机体前向
`g_eval_body_frd_x`的最佳相关系数为+0.989，与pitch候选的最佳相关系数同为+0.989，响应滞后约
0.30 s。pitch候选范围为-0.9433--+0.5706 deg/s，且逐样本与`g_eval_body_frd_x`一致，最大数值
误差约`4.99e-7`。该结果确认旧`[0,0,1]`来源错误已修复，纵向像面运动现在进入机体前向导引分量
和pitch候选。

动态窗口仍有26个YOLO有候选但跟踪器无输出的样本，最长连续空窗约0.401 s，因此ByteTrack纵移
连续性仍未通过接管门槛。物理Betaflight pitch杆正负号也尚未验证；相关系数只能验证信号链，
不能授权RC7接管。全程8223行均为`LOG_ONLY/disabled/armed=0/override=0`，所有
SET_RAW_RC attempt/write/success计数为0，MSP错误为0，无桨审计`passed=1`且0违规。退出后无残留
进程，`/dev/ttyS1`已释放。证据归档为
`logs/deployment_archives/fixed_vm_body_frame_20260829_152437.tar.gz`，SHA256为
`a8574563a9417f2a874b0f5ae2b31b3d2452f83690f4dd1c7c2f0f27f72f14ce`。

## 2026-08-29 ByteTrack 纵移低分支A/B

上一轮动态窗口的26个跟踪缺失样本中，20个已在ByteTrack低分阶段成功关联，但被
`final_min_score=0.25`过滤；其余6个在实物接近画面边缘、检测分数约0.081时进入
`active_track_lost`。为隔离变量，新建LOG_ONLY候选，仅把`track_low_thresh`从0.10降为0.05、
`final_min_score`从0.25降为0.05，未修改高分建轨阈值、导引、矩阵和控制包线。配置SHA256为
`ff1c554a2a36f181068cf0fd5ba000e5a19479cb93042e6977d1a05cb3bb8e27`。

真实边缘纵移的50--100 s窗口中，987个新结果均有YOLO候选，ByteTrack和导引均输出987次，全部
为`track_id=2`，最长空窗0；211帧检测框触及下边缘，最低输出分数0.228。第二次压力窗口60--105 s
又得到889/889/889的候选、跟踪和导引输出，全部为`track_id=1`。两次均为
`LOG_ONLY/disabled/armed=0/override=0`，SET_RAW_RC attempt/write/success和MSP错误全部为0，
无桨审计通过。第一轮方向链路仍保持图像纵向速度对机体前向/pitch候选相关系数+0.976。

为可重复覆盖低分域，合成手持序列生成器新增`--foreground-opacity`，其默认值1.0保持原行为，
参数写入metadata；0.15透明度序列共120帧，每帧都有RKNN候选，最低可关联输出分数0.055。相同
视频A/B显示：原0.10/0.25配置输出率65.83%、fragment=2；0.05/0.05配置输出率72.50%、
fragment=1，且只有同一track ID、无switch。该结果证明降低最终输出和低分候选门槛能保留一部分
已关联测量，但不能把模型已失去真实目标特征的候选强制视为有效目标。

vendored ByteTrack原先将第二阶段低分IoU距离门限硬编码为0.5，现暴露为显式
`low_match_thresh`且默认仍为0.5，并增加单元测试。实验候选0.8在同一视频上的输出率和fragment
仍为72.50%/1，没有任何收益，因此不进入实物候选；扩大关联门限会增加误关联风险，不能仅因追求
连续率采用。真实低分、合成A/B、配置和源码哈希归档于
`logs/deployment_archives/fixed_vm_tracker_low_conf_ab_20260829_154658.tar.gz`，SHA256为
`fa13993d0371b12154ef6d87f2c7ec4af16d6895a967a712cc713e43522f4856`。当前仅允许继续LOG_ONLY；
物理pitch符号仍需独立拆桨单轴试验，尚未授权RC7接管。

本地完整测试277/277通过。板端tracker与合成生成器聚焦测试10/10通过；板端直接执行全量discover
时完成191项并出现3个收集错误，原因是该部署目录未复制AirSim的gimbal、strapdown和truth示例
模块，不是Betaflight或ByteTrack测试失败。板端聚焦测试输出保存在
`logs/rk3588_tracker_focused_tests_20260829.txt`，不得把不完整部署包的AirSim导入错误解释为实机
运行时回归。

## 2026-08-29 无桨接管纵向响应与 MSP 诊断补强

在`.42/orangepi5max`上使用低分支固定Vm配置完成拆桨、固定机体、功率电供电试验。对应飞控快照为
`logs/betaflight_snapshots/betaflight_snapshot_20260829_161533/manifest.json`，批准文件为
`logs/betaflight_noprop_vm_body_frame_tracker_low_conf_approval.json`；配置SHA256为
`ff1c554a2a36f181068cf0fd5ba000e5a19479cb93042e6977d1a05cb3bb8e27`，批准文件SHA256为
`b6cff90fd7329514a76ace4e62c3141a3dd287cbf4c575410644758ed5d64b34`。日志
`fixed_vm_pitch_sign_longitudinal_20260829_163322_20260829_163323.csv`确认RC7接管进入
`ACTIVE/algorithm`，入场油门交接连续，发送油门不超过1078 us。目标向机头移动时
`g_B.x>0`且pitch通道E最高1501 us；反向回移时pitch变负且E最低1499 us。该结果验证从真实相机、
YOLO/ByteTrack、机体系导引量到pitch RC命令的符号链，但原日志没有电机输出，尚不能证明实际
Betaflight motor mix的物理pitch符号。操作者最终先退出RC7接管，再RC5 DISARM；runner已停止且
`/dev/ttyS1`释放。

原v11审计报告中的3行`command_shaping_nonfinite`和3行
`algorithm_with_invalid_command_shaping`来自主循环记录当前shaping、同时记录发送线程上一拍状态的
时间基准错配，并非发现非有限PWM已发送。日志模式升级到v12，新增发送线程实际最近一次命令的
`active/reason`及发布门字段；审计仅在v12用这些字段判定已发布算法命令，旧日志继续按旧规则并
标记无法恢复的时间基准。发送线程还增加`RcCommand.active`硬门控，防止仅凭上游授权发送无效命令。

退出接管时曾观察到最大SET_RAW_RC间隔100.045 ms。根因是OVERRIDE期间暂停MSP_RC轮询，RC7退出
后等待第一帧恢复的物理RC。运行时现移植src的`override_grace_hold_s=0.35`：只在OVERRIDE下降沿后
短时继续发送退出前锁存的人工通道，算法控制立即停止；收到有效物理RC即提前清除保持。该机制不
延长算法接管，也不替代接收机failsafe。

为验证实际混控，新增低频`MSP_MOTOR (104)`读取，示例配置为`motor_poll_hz=2`，CSV记录最多8路
电机输出、时间戳和命令统计，Web页面直接显示Motor 1--8及数据年龄；审计在启用轮询但无成功帧或
存在错误时失败。该命令在当前定制Betaflight固件上的支持情况仍需LOG_ONLY确认；不支持时应改用
Blackbox，不得猜测电机序号或混控符号。上述运行时配置会改变配置哈希，板端同步后必须重新采集
快照并生成新的无桨批准文件，旧批准文件不得复用。

本轮离线修改完成后，本地`python3 -m unittest discover -s tests -v`为282/282通过，
`git diff --check`通过。尚未把新代码同步到板端，也未执行新的LOG_ONLY或RC7接管测试。

## 2026-08-29 电机遥测验证与单 UART 调度隔离

提交`bba4f69`已同步到`.42/orangepi5max`，板端Betaflight聚焦测试99/99通过。当前固件支持
`MSP_MOTOR (104)`：DISARM时四路输出均为1000，ARM怠速时约为1056--1059。新飞控快照为
`logs/betaflight_snapshots/betaflight_snapshot_20260829_171813/manifest.json`，控制配置
`betaflight.rk3588.noprop.vm.body_frame_tracker_low_conf_motor.local.json`的SHA256为
`b563eff50e9afeb13b0cbee1f79129f68bf3e92eb14d80ec9a8e819c701d76e7`；对应批准文件SHA256为
`d8d35dc14ccfa2fe6af725f4c461bed320dcbdc217e79ca473173a7b88c9078b`。批准范围仍仅为拆桨台架，
不能用于有桨飞行。

长测发现运行状态打印会被989/990 us物理RC抖动和逐帧目标有效性变化触发。控制台状态键现只含
安全状态、原因、ARM、OVERRIDE、预填充和发布模式；目标与实际发送通道仍随真正的状态变化打印，
但1 us抖动不再持续占用stdout锁。该修正板端Betaflight测试108/108通过。

未隔离的完整RKNN运行在浏览器连接MJPEG时产生74.249 ms的SET_RAW_RC写入间隔。首次累计值越过
60 ms的相邻CSV行显示`web_preview_encode_count`从0开始增长，同时SET_RAW_RC最大RTT升至
74.393 ms；手持目标抖动不可能直接阻塞独立50 Hz MSP发送线程。关闭预览但保持同进程后，180 s
DISARM压力测试的最大间隔降至42.547 ms、P99.9为40.007 ms，错误计数为0，但仍因比40 ms门限高
7 us而严格判失败，未据此进入ARM。

最终使用以下运行隔离参数复测，RC5保持DISARM、RC7保持人工侧：

```bash
python3 examples/run_betaflight_log_only.py \
  --config config/betaflight.rk3588.noprop.vm.body_frame_tracker_low_conf_motor.local.json \
  --duration-s 180 --rate-hz 20 \
  --control-mode msp_raw_rc --allow-control \
  --detector-source rknn_bytetrack \
  --disable-web-preview --isolate-rknn-process \
  --main-cpu-affinity 6,7 --rknn-cpu-affinity 4,5 \
  --log-prefix fixed_vm_preview_disabled_isolated_disarm_stress_20260829
```

日志`fixed_vm_preview_disabled_isolated_disarm_stress_20260829_20260829_174501.csv`持续179.961 s，
8962次SET_RAW_RC写入，平均49.798 Hz，最大间隔32.389 ms、P99.9为30.388 ms；写入、请求、校验、
解析和Web错误均为0，最大RKNN耗时22.82 ms，最高温度62.846 C，电机输出始终1000。无桨审计
`passed=1`且0违规。当前实机命令必须保留上述三个隔离选项；Web JSON/SSE继续可用，但MJPEG图像
预览必须关闭。下一门为拆桨固定条件下短时ARM且RC7保持人工侧，确认四电机怠速与调度审计均通过；
之后才允许放置稳定单目标并短时切换RC7验证`ACTIVE/algorithm`和电机混控方向。

## 2026-08-29 固定目标主动接管复盘与电机输出联锁

拆桨固定机体的稳定单目标LOG_ONLY先运行40 s，目标有效率98%，195/195个新跟踪结果保持同一
track，SET_RAW_RC为0且审计通过。主动接管日志
`fixed_vm_isolated_fixed_target_takeover_20260829_20260829_175710.csv`约在109.68--175.58 s处于
`ACTIVE/algorithm`。该区间上位机roll命令约`-0.018..+0.016 deg/s`、pitch命令约
`-0.010..+0.009 deg/s`，未出现足以解释大幅混控的视觉rate候选。

电机遥测抽取窗口显示M1约1281--1363、M2约1346--1456、M3约1056、M4约1325--1431。schema
13工具重新审计完整日志后，最大输出为1625 us、最大极差为569 us；884行超过1200 us，首次在
elapsed 124.781 s，874行极差超过150 us，首次在125.285 s。固定无桨机体上的Betaflight
PID/I-term/mixer积累是候选解释，但当前没有同一时间轴的Blackbox gyro、setpoint、P/I/D和
motor记录，因此不能把该推测写成根因。SET_RAW_RC累计最大写间隔为81.534 ms，首次越过60 ms
约在elapsed 122.519 s。该点MJPEG客户端和预览编码均为0，RKNN总耗时约6.27 ms，说明关闭预览
不是充分修复。

原始CSV/meta/events、新旧审计已复制到工作站归档
`fixed_vm_isolated_fixed_target_takeover_20260829_175710.tar.gz`，SHA256为
`f46d562ed100a38166d9a47f952b8180a7dbb08986b43dfd811cb2ebadd48f43`。新版审计为3项违规：
`armed_motor_output_high`、`armed_motor_spread_high`和`set_raw_rc_gap`。

离线代码新增以下保护和证据：

- 日志schema升级到13，记录Python GC回收次数、代次以及最近/最大/累计暂停；审计报告SET最大与
  P99.9间隔首次越限的elapsed时间。
- `MotorOutputInterlock`检查四路`MSP_MOTOR`的新鲜度、完整性、最大输出和极差。无桨默认上限为
  1200 us/150 us，任一ARM态联锁故障锁存到DISARM；门禁关闭后MSP worker继续发送接管前锁存人工RC，
  不直接断流。
- `noprop_bench`请求控制时强制启用锁存联锁和电机轮询；批准工具同时绑定4路、1200 us、150 us、
  0.75 s和`latch_until_disarm=true`，配置哈希改变后旧批准文件自动失效。
- CSV审计新增`armed_motor_output_high`、`armed_motor_spread_high`和
  `algorithm_without_motor_interlock`；Web schema 10显示联锁原因/锁存/最大值/极差和GC暂停。

本地`python3 -m unittest discover -s tests -p 'test_*.py' -v`为290/290通过，
`git diff --check`通过。该批改动尚未同步到断电的Orange Pi，未做schema 13实机验证。下次上电
必须先同步代码与local配置、重新生成快照和批准文件，从LOG_ONLY验证电机/GC字段开始；取得并对齐
Blackbox前，不重复长时固定机体RC7接管，不进入有桨测试。

## 2026-08-29 schema 13 板端部署与 DISARM 基线

设备重新上电后只连接已确认的`192.168.124.42/orangepi5max`（设备树`RK3588 OPi 5 Max`）。同步
提交`f7e27c4`的运行时文件前，已将板端旧文件备份到`logs/deployment_backups/`；没有覆盖任何
local配置、模型、原生库或历史日志。代码包两端SHA256均为
`0d2f39c64cc477d9f4a2c0eec5abeb7a111e9e856408b585d8ef17b02c753729`。板端Betaflight聚焦测试
105/105、`test_flight_control.py` 24/24通过。

两个固定Vm motor local配置增加相同联锁块。控制配置SHA256为
`a9178d2924501fa4edf56c250d09d0e06bf0c7517b81317c857784b699b8393c`，LOG_ONLY配置SHA256为
`71eab09cd79ed24034c087b697185204fdc7e612af372950c7dc74435f89cba5`。新快照
`logs/betaflight_snapshots/betaflight_snapshot_20260829_195811/manifest.json`采集25/25样本且零错误，
确认BOX ID 50、CLI mode ID 50、mask 15、NTP/RTC一致。重新生成的批准文件SHA256为
`6f209269a45593c7ed2b5073ec034a868e3e54830a406abb4c77c67d641f054b`，绑定上述控制配置和四电机
1200/150 us、0.75 s锁存联锁。

`schema13_interlock_logonly_20260829_20260829_195621.csv`持续59.965 s、1197行，四电机始终1000，
SET_RAW_RC attempt/write/success均为0；1154个新感知结果中1153个确认跟踪和导引有效，全部为
`track_id=1`。MSP错误为0，GC最大暂停2.141 ms，审计`passed=1`且0违规。

`schema13_interlock_disarm_prefill_20260829_20260829_195930.csv`持续89.979 s、1797行，在RC5
DISARM、RC7人工侧完成4491次SET_RAW_RC写入和4490个ACK；平均49.925 Hz、最大间隔34.820 ms、
P99.9为33.849 ms，写入/请求/校验/解析错误均为0，四电机始终1000，GC最大暂停2.323 ms，审计
0违规。两轮证据归档为`schema13_interlock_baseline_20260829_195811.tar.gz`，工作站与板端SHA256
均为`4042da5e45e1c09613967b23fd6a66a1576526247e9ad19bca09da11bff1f669`。

当前runner已停止且`/dev/ttyS1`释放。下一阻断项是导出17:55--18:00附近Blackbox并把gyro、
setpoint、P/I/D和motor与旧主动接管CSV对齐；完成前不执行新的ARM/RC7接管。

## 2026-08-29 LOG00042 Blackbox 根因确认与持续时间联锁

用户导出的42个BFL文件位于`logs/blackbox_import/`，SD时间戳均错误地显示为2015年。通过时长与
四电机波形联合匹配，确认`LOG00042.BFL`对应17:57主动接管：Blackbox时长141.488814 s，主机
ARM时长141.400970 s。BFL SHA256为
`4e9938851e5cad82f4be67591d4d0da2f3ddfc1baa56d9cadb39a0e52dd7285f`。使用官方
`blackbox-tools`提交`f832acf9cd9dbe5ad8220de1a5f4eb4021523d72`按raw rotation解码；定制固件的
`gyro_scale=1.0`使deg/s解码无效，因此gyro未用于物理单位结论。

自动对齐结果为`host=blackbox+36.059486 s`，Blackbox电机raw到MSP us的拟合为
`0.499231573*raw+977.151031`，相关系数0.999895785、RMSE 2.086 us。接管前四电机极差不超过
4.99 us且I项为零。算法发布期间Blackbox三轴setpoint严格为零，但主机油门越过
`min_check=1050 us`后，I项以约`[-2.145,-2.949,+1.733] unit/s`累积到
`[-127,-174,+103]`，电机极差增至581.60 us。电机极差与最大绝对I项相关系数0.999733，显著高于
P项0.342895和D项0.571071。根因由“候选”更新为：固定无桨机体在1078 us算法油门下无法响应，
Betaflight PID回路发生I项积累；不是PNG rate命令导致。

离线代码因此增加`takeover_duration_interlock`：`noprop_bench`连续接管最多3.0 s，超时撤销算法
授权、回到预锁存人工RC并锁存到DISARM。批准工具强制绑定`enabled=true`、
`latch_until_disarm=true`和不超过3.0 s；日志schema 14及Web schema 11记录联锁状态、累计时间和
门限，CSV审计拒绝任何绕过联锁的ACTIVE行。长时视觉测试改用LOG_ONLY；高于`min_check`的电机
方向验证仅使用短脉冲。完整分析见`doc/BETAFLIGHT_BLACKBOX_LOG00042_ANALYSIS.md`及
`doc/evidence/LOG00042_alignment.json`。这些改动尚未同步或在Orange Pi上复验，当前仍禁止ARM和
RC7接管。

## 2026-08-29 schema 14 三秒接管截止实机验证

提交`febf399`及schema 14配置已部署至`.42/orangepi5max`。控制前只读快照
`logs/betaflight_snapshots/betaflight_snapshot_20260829_210525/manifest.json`确认飞控未ARM、
MSP OVERRIDE未激活，物理RC5为2000 us、RC7为1000 us；四电机输出均为1000。随后使用固定单目标、
拆桨固定机体和功率电执行隔离运行，保持`--disable-web-preview`、独立RKNN进程及CPU亲和性配置。

日志`schema14_takeover_timeout_active_retry_20260829_20260829_210708.csv`完整记录以下转换：

- elapsed 22.691 s ARM，人工侧怠速约1056--1057 us，四电机极差1 us；
- elapsed 55.952 s RC7进入MSP OVERRIDE，56.002 s开始发布算法命令；
- elapsed 58.958 s触发`FAILSAFE/takeover_duration_interlock`，最大累计时间3.006313 s；
- 下一采样帧即回到`passthrough`，60.962 s退出RC7，62.013 s DISARM。

60个算法发布采样中，四电机为1056--1089 us，最大同帧极差10 us；整个ARM区间最大输出1091 us、
最大极差12 us，均低于无桨联锁的1200/150 us门限。共完成4485次SET_RAW_RC写入和4484个ACK，
末端平均49.886 Hz，最大写间隔34.045 ms、P99.9为33.948 ms，写入、解析和校验错误均为0。

原审计在截止首帧报告一条`command_shaping_nonfinite`。该帧MSP异步线程仍记录上一条已发布算法命令，
主循环已经`gate_closed`并按设计不再消费姿态，因此当前`tilt_*_attitude_deg`为空；实际发布RC和电机值
均为有限值。审计器现仅在当前shaping确实应用姿态包络时检查该字段，并新增schema 14异步切换回归
测试。工作站及板端审计测试17/17通过，重审实机日志为`passed=1`、0违规。

本轮证据归档为
`logs/deployment_archives/schema14_takeover_timeout_active_retry_20260829_210708.tar.gz`，SHA256为
`430c8a5626623c8fc8f3c81ac718fd4299cbbb765d24027ffae77a7d2e211b2f`。测试进程已停止，最终状态为
RC7人工侧、RC5 DISARM、电机1000。该结论只证明无桨固定台架的短时接管和自动截止有效，不构成
装桨、自由飞行或自主拦截授权。
