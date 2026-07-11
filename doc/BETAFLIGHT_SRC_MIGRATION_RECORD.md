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
|MSP OVERRIDE mode 定位|permanent ID 50|移植只读识别；未批准前不发送 RC|
|RC rate/throttle 映射|线性 PWM 映射|保留 Python 限幅/斜率限制，来源值仅作候选|
|物理杆保持与 AUX 保留|`bf_control_host`|移植 fail-closed 策略，禁止伴随机主动 ARM|
|watchdog/新鲜检测/armed 门控|`bf_gating`|扩展 Python safety inputs 和日志|
|油门交接|0.4 s 线性混合|实现但在 hover throttle 标定前阻断控制|
|systemd 打包|`adapters/bf/systemd`|改写为 Python LOG_ONLY 服务，安装但不启用|

## 已发现冲突

- 默认 PNG 文件使用 `hover=0.078`、`strike=0.082`，速度档使用 `0.283/0.50`。
- `bf_flight_common.yaml` 使用最大速率 `3.491 rad/s`，现有 `bf_runtime_test` 仍期望
  `5.236 rad/s`，板端测试有两项失败。
- 来源注释写 AETR，但活动索引为 roll=0、pitch=1、yaw=2、throttle=3，实际通道顺序必须
  以 Receiver tab 和无桨方向测试为准。
- 所有飞行 profile 均设置 `dry_run=false`，不能作为当前工程默认值。
- 未发现 Betaflight CLI `dump/diff all`、Blackbox、动力标定或 Rate/PID 来源记录。

处理决定：全部冲突值只进入来源清单；RK3588 运行配置继续保持零 `rate_gain_matrix`、
`control_authorization.enabled=false`，直到 CLI 快照、无桨方向验证和动力标定完成。

## 阶段记录

|阶段|状态|验收证据|
|---|---|---|
|S0 来源清单与冲突审计|完成|来源 SHA256、候选参数、冲突和禁止生效标记|
|S1 只读飞控快照|完成|MSP identity/BOXIDS/telemetry + CLI 导入 manifest|
|S2 安全 MSP 运行架构|完成|单元测试证明所有授权条件 fail-closed|
|S3 Python systemd|完成|服务已安装且 disabled/inactive，命令固定 LOG_ONLY|
|S4 Orange Pi 验证|完成|60 s 日志、RAW RC 发送计数为 0、文件哈希一致|

## 留档规则

- 代码、示例配置、来源清单、决策和脱敏测试摘要进入 Git。
- 每次实机运行在 `logs/` 下生成独立目录，保存原始 CSV、meta、CLI 导出和 manifest；
  `logs/` 不提交，但 manifest 中必须包含每个文件 SHA256。
- 不记录 SSH 密码、无线凭据或其他秘密。
- 每阶段独立提交；板端验证完成后追加最终验证记录，不改写历史结论。

## 只读快照命令

先在 Betaflight Configurator CLI 中人工执行 `diff all` 或 `dump all` 并保存文本，再运行：

```bash
python3 tools/capture_betaflight_snapshot.py \
  --config config/betaflight.rk3588.example.json \
  --duration-s 5 --rate-hz 5 \
  --cli-export /path/to/betaflight_diff_all.txt
```

未提供 CLI 文件时仍可完成 MSP 只读快照，但 manifest 必须把 PID/Rate/Failsafe/Blackbox
标记为缺失。快照工具没有 RC 发送接口，且始终输出 `control_ready=false`，人工审核不能通过
直接修改原始 manifest 绕过后续授权文件和哈希检查。

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
哈希见 `config/betaflight.rk3588.validation.json`；板端时钟仍不正确，文件名时间不能用于
跨设备对时。
