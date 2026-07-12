# Betaflight src 参考与迁移记录

## 目的与安全边界

本文记录从 Orange Pi `/home/orangepi/src/circle_pilot` 参考或移植到当前 Python 工程的
非视觉能力。原目录不修改、不启动；当前实机验证保持无桨、`LOG_ONLY`，不得发送
`MSP_SET_RAW_RC`。机器可读来源、哈希和候选参数保存在
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
|S7 无桨 RC 验证|阻断|NTP/RTC、OVERRIDE、AUX、Rate 映射和动力参数未关闭|

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
实际列表不含 `src` 假设的 permanent ID 50，因此 `msp_override_available=false`，控制继续
阻断。快照完成 25/25 样本、错误 0；未提供 CLI 导出，PID/Rate/Failsafe/Blackbox 未关闭。

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
  `STICK COMMANDS DISABLE`、`BEEPER MUTE`、`READY`，均不是 ID 50 的替代项；当前固件
  没有 `MSP OVERRIDE`，控制阻断结论不变。
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
2. `src` 假设 `override_channels_mask=15`，但不会读取 CLI 中的实际值；当前飞控值为 0。
3. `dry_run=true` 且默认 `passthrough_in_dry_run=true` 时仍会发送 `MSP_SET_RAW_RC`，因此
   不能把 `src` dry-run 当作本项目的只读 `LOG_ONLY`。

板端执行已有 `bf_runtime_test`，两项共享配置检查仍按 5.236 rad/s 编写，与活动配置
3.491 rad/s 不一致，测试退出码为 1。`src` 没有 NTP/RTC 配置，也没有 Blackbox、CLI
快照或电池标定证据。机器可读哈希、缓解措施和剩余风险见
`config/betaflight.src-reference.json`；原目录和服务均未修改，`bf_flight` 保持 inactive。
