# Betaflight 速度建立 PNG 无桨电机方向验证

## 结论

2026-09-03 在全部桨叶拆除、机体固定、6S 供电条件下，完成真实相机、RKNN YOLO、
ByteTrack、速度建立 PNG、姿态外环、Betaflight Rate 反解、MSP OVERRIDE 和 QUADX 混控的
双向纵向验证。

初始配置使用 `pitch_rate_sign=+1`。机头侧目标产生正确的机体前向加速度和负 FRD 期望
Pitch，但发送到 Betaflight 后使前电机升高，物理响应为抬头，方向错误。配置改为
`pitch_rate_sign=-1` 后：

- 机头侧目标使后电机 M1/M3 高于前电机 M2/M4，产生低头响应；
- 机尾侧目标使前电机 M2/M4 高于后电机 M1/M3，产生抬头响应；
- 两轮安全审计均通过，未触发电机输出、差速或接管时长联锁；
- 两轮最终均记录 `armed=0`、`override=0`，MSP 写入错误为 0。

因此本机 Betaflight 输出边界必须使用 `roll_rate_sign=+1`、`pitch_rate_sign=-1`。该结论验证
无桨台架上的视觉到电机物理方向，不证明带桨闭环稳定性、视场保持或真实空中拦截命中率。

## 配置绑定

```text
config
  config/betaflight.rk3588.noprop.velocity_establishing.example.json
  SHA256 fc8b2d80dc888f4725b8265fd2a57cc762bc41a1efd95ef3d495c866e6cc905b

snapshot
  logs/betaflight_snapshots/betaflight_snapshot_20260903_110145/manifest.json
  SHA256 c211e539effd2ff0fffd05ecb547b722317a090af5e5b7d2503000f6c4d44fbf

approval
  logs/betaflight_noprop_velocity_establishing_approval.json
  SHA256 c23a69de2bc55d0f02124f352a111c4fb27219d6ea20ce6abbfbb9e45890b730
```

快照包含 25 个样本、0 个采集错误，飞控为 Betaflight `25.12.2`、MSP API `1.47`，
MSP OVERRIDE permanent ID 为 50、mode index 为 28。

## 错误符号对照

日志：

```text
logs/velocity_png_noprop_active/
  VELOCITY_PNG_MOTOR_NOSE_RETRY2_20260903_105053_20260903_105055.csv
SHA256 7738dc39f21fcf064897d40637adcce59994a5f9586751344f994e78ba2a4567
```

机头侧目标产生 `g_eval_body_frd_x=+0.706..+0.791 m/s2`，FRD 期望 Pitch 为
`-4.39..-3.89 deg`。错误的 `pitch_rate_sign=+1` 发送约 `-3 deg/s`，代表性电机输出为
`[M1,M2,M3,M4]=[1056,1101,1097,1129] us`；前电机平均比后电机高 `38.5 us`，与所需
低头响应相反。该轮方向证据有效，但严格审计另有一次与符号无关的 40.658 ms P99.9 MSP
写间隔超限。

## 修正后机头方向

日志：

```text
logs/velocity_png_noprop_active/
  VELOCITY_PNG_MOTOR_NOSE_SIGNFIX_RETRY_20260903_110316_20260903_110317.csv
SHA256 a51b414af427af49909cd95b5930e681bd49d61c10aa36d5dcc6769866f731e0
```

- ACTIVE 持续 `1.532 s`，目标段 `g_eval_body_frd_x=+0.715..+0.768 m/s2`；
- Betaflight Pitch 命令最高 `+3 deg/s`；
- 代表性输出为 `[1114,1056,1198,1115] us`；
- 后电机平均比前电机高 `70.5 us`，方向为低头；
- 最大电机输出 `1198 us`，最大差速 `142 us`；
- SET_RAW_RC 写入速率 `49.723 Hz`，P99.9 间隔 `30.802 ms`，写入错误 0；
- 审计 `passed=true`，0 violations。

该轮距离 `1200 us / 150 us` 联锁仅余 `2 us / 8 us`，说明固定机体上的持续 Rate 命令会快速
积累 PID 差速。后续不应通过延长无桨固定接管来评估控制强度。

## 修正后机尾方向

日志：

```text
logs/velocity_png_noprop_active/
  VELOCITY_PNG_NOPROP_SELFTEST_20260903_111132_20260903_111133.csv
SHA256 f0b7408016010eba10f70c481988c183d4408b65b53c46f52fcd507791634c64
```

- ACTIVE 持续 `1.445 s`，目标段 `g_eval_body_frd_x=-0.912..-0.658 m/s2`；
- Betaflight Pitch 命令最低 `-3 deg/s`；
- 代表性输出为 `[1063,1112,1061,1097] us`；
- 前电机平均比后电机高 `42.5 us`，方向为抬头；
- 最大电机输出 `1112 us`，最大差速 `51 us`；
- SET_RAW_RC 写入速率 `49.588 Hz`，P99.9 间隔 `33.301 ms`，写入错误 0；
- 审计 `passed=true`，0 violations。

## 后续限制

1. 无桨 Roll 左右和 Pitch 前后物理方向均已有双向证据，不再重复固定机体长脉冲。
2. 当前 `3 deg/s`、`1000--1100 us` 和电机联锁只用于无桨验证，不能作为带桨飞行参数。
3. 首次主动飞行前仍需解决正式运动状态输入、真实天空跟踪、失效策略和低权限闭环科目。
4. 飞行候选继续保持 `runnable=false`、`control_authorized=false` 和
   `propeller_flight_authorized=false`。
