# Betaflight LOG00042 Blackbox 对齐分析

## 日志选择与证据链

本次导出的 42 个 BFL 文件均带有错误的 2015 年 SD 卡时间，不能按文件时间选择。目标文件是
`logs/blackbox_import/LOG00042.BFL`，SHA256 为
`4e9938851e5cad82f4be67591d4d0da2f3ddfc1baa56d9cadb39a0e52dd7285f`。其有效时长为
141.488814 s；主机日志中的 ARM 区间为 36.135366--177.536336 s，持续 141.400970 s，差
0.087844 s。四路电机波形进一步给出唯一稳定对齐，因此确认它对应
`fixed_vm_isolated_fixed_target_takeover_20260829_175710.tar.gz`。

使用 Betaflight `blackbox-tools` 提交
`f832acf9cd9dbe5ad8220de1a5f4eb4021523d72` 解码。BFL 的自定义
`gyro_scale=0x3f800000` 会使 `--unit-rotation deg/s` 产生约 5700 万 deg/s 的无效结果；本分析只用
`--unit-rotation raw` 的 CSV，且不把 gyro raw 命名为 deg/s。

## 时间与电机标定

MSP 电机样本使用 `elapsed_s - msp_motor_age_s` 作为采样时刻，并与 Blackbox 四电机通道共同拟合：

```text
host_elapsed_s = blackbox_elapsed_s + 36.059486
MSP_motor_us = 0.499231573 * blackbox_motor_raw + 977.151031
```

拟合相关系数为 0.999895785，RMSE 为 2.086 us。接管前四电机约为
1056.03--1061.02 us，最大极差仅 4.99 us，三轴 I 项均为零。

## 接管阶段结果

主机在 109.631482 s 检出 MSP OVERRIDE，在 109.681546 s 开始算法发布。Blackbox 的三轴
setpoint 在接管窗口内严格为 `[0, 0, 0]`，F 项也严格为零；因此 PNG 输出不是本次电机分化的
直接原因。

算法油门于主机 110.184030 s 越过 Betaflight `min_check=1050 us`。Blackbox throttle 在
110.163850 s 开始生效，I 项于 110.265572 s 首次非零。第一次连续接管期间 I 项斜率为
`[-2.145458, -2.948690, +1.733265] unit/s`，最大绝对值达到 `[127, 174, 103]`。估算四电机峰值为
`[1498.85, 1637.63, 1091.48, 1583.22] us`，最大极差 581.60 us。

电机极差与最大绝对 I 项的相关系数为 0.999733，而与 P、D 项仅为 0.342895、0.571071。固定且
无桨的机体不能响应控制器，油门超过 `min_check` 后 PID 回路持续积分，最终由 I 项驱动 mixer
分化。169.016547 s 的一次旧指令切换使输出短暂回到人工约 990 us，I 项立即清零；算法恢复后又
重新积累，这与上述机制一致。

Blackbox 文本显示 `ANGLE_MODE`，但同次配置中 RC8 为 1000，Angle 范围为 1700--2100。定制固件
与解码器可能存在 flag 定义差异，该标签不能作为已进入 Angle 模式的证据。

## 安全结论与后续门禁

- 固定无桨机体不得再做长时间、超过 `min_check` 的算法接管。
- `noprop_bench` 连续接管限制为最多 3.0 s；超时立即撤销算法授权并锁存至 DISARM。
- 长时间视觉验证只运行 `LOG_ONLY`，或另建油门不超过 `min_check` 的专用配置。
- 高于 `min_check` 的验证只允许短脉冲，每次检查 CSV 和 Blackbox 后再继续。
- 本结果不能替代有桨系留标定；有桨状态下机体可响应，I 项行为必须重新测量。

可复现结果保存在 `doc/evidence/LOG00042_alignment.json`，生成工具为
`tools/analyze_betaflight_blackbox_alignment.py`。

```bash
/tmp/betaflight-blackbox-tools/obj/blackbox_decode \
  --unit-rotation raw --output-dir /tmp/blackbox_raw \
  logs/blackbox_import/LOG00042.BFL

python3 tools/analyze_betaflight_blackbox_alignment.py \
  --host-csv /tmp/host_takeover/logs/fixed_vm_isolated_fixed_target_takeover_20260829_20260829_175710.csv \
  --blackbox-csv /tmp/blackbox_raw/LOG00042.01.csv \
  --blackbox-bfl logs/blackbox_import/LOG00042.BFL \
  --decoder-commit f832acf9cd9dbe5ad8220de1a5f4eb4021523d72 \
  --min-check-us 1050 \
  --output doc/evidence/LOG00042_alignment.json
```
