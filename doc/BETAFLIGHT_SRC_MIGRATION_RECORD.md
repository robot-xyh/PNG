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
- 所有飞行 profile 均设置 `dry_run=false`，不能作为当前工程默认值。
- 未发现 Betaflight CLI `dump/diff all`、Blackbox、动力标定或 Rate/PID 来源记录。

处理决定：全部冲突值只进入来源清单；RK3588 运行配置继续保持零 `rate_gain_matrix`、
`control_authorization.enabled=false`，直到 CLI 快照、无桨方向验证和动力标定完成。

## 阶段记录

|阶段|状态|验收证据|
|---|---|---|
|S0 来源清单与冲突审计|完成|来源 SHA256、候选参数、冲突和禁止生效标记|
|S1 只读飞控快照|待实施|MSP identity/BOXIDS/telemetry + CLI 导入 manifest|
|S2 安全 MSP 运行架构|待实施|单元测试证明所有授权条件 fail-closed|
|S3 Python systemd|待实施|服务安装后 disabled/inactive，命令固定 LOG_ONLY|
|S4 Orange Pi 验证|待实施|60 s 日志、RAW RC 发送计数为 0、文件哈希一致|

## 留档规则

- 代码、示例配置、来源清单、决策和脱敏测试摘要进入 Git。
- 每次实机运行在 `logs/` 下生成独立目录，保存原始 CSV、meta、CLI 导出和 manifest；
  `logs/` 不提交，但 manifest 中必须包含每个文件 SHA256。
- 不记录 SSH 密码、无线凭据或其他秘密。
- 每阶段独立提交；板端验证完成后追加最终验证记录，不改写历史结论。
