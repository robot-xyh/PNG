# Gazebo Betaflight PNG 测试记录

## 目标

参考 `circle_ai_strike-main` 的 `RateCommand -> RC -> Betaflight body-rate` 链路，绕开 AirSim，直接在 Gazebo Harmonic 中测试真值 PNG 输出到 Betaflight SITL 的控制链路。

## 新增入口

- `examples/run_gazebo_betaflight_truth_png.py`
  - 从 Gazebo `/world/<world>/pose/info` 订阅拦截机位姿。
  - 目标位置在脚本内部按真值轨迹生成，默认从机体前方起始。
  - `compute_truth_png -> RateCommand -> body_rate_rc_from_rate_command -> MSP_SET_RAW_RC`。
  - 记录 CSV 到 `logs/gazebo_betaflight_truth_png/`。

- `run_gazebo_betaflight_truth_png_demo.sh`
  - 检查 `5761/9001-9004` 端口，避免重复启动冲突。
  - 加载 `config/betaflight_sitl_truth_png_msp_cli.txt`。
  - 启动 Gazebo Harmonic 和 Betaflight SITL，结束后清理本脚本启动的进程。

## 关键环境修正

`/home/linux/aeroloop_gazebo/plugins/BetaflightPlugin.cc` 按 `<rotor>` 块出现顺序读取 `motorSpeed[i]`，不使用 SDF 中的 `motorNumber`。因此本机已修正：

`/home/linux/aeroloop_gazebo/models/betaloop_iris_with_standoffs/model.sdf`

BF QUADX 顺序应为：

1. `motor[0] -> rotor_3_joint` rear-right, cw
2. `motor[1] -> rotor_0_joint` front-right, ccw
3. `motor[2] -> rotor_1_joint` rear-left, ccw
4. `motor[3] -> rotor_2_joint` front-left, cw

修正前会快速翻倒；修正后中性满油门可稳定起飞。

## 已验证命令

中性满油门 smoke：

```bash
./run_gazebo_betaflight_truth_png_demo.sh --duration-s 3 --rate-hz 30 \
  --trajectory-prefix smoke_launcher_full_throttle_settle \
  --max-tilt-deg 0 --hover-thrust 1.0 --min-thrust 1.0 --max-thrust 1.0 \
  --target-altitude-offset-m 0 --interceptor-speed 0 \
  --speed-hold-kp 0 --altitude-kp 0 --altitude-kd 0 --yaw-p 0
```

结果：`z` 从约 `0.19m` 上升到约 `1.05m`。

PNG full-thrust 8s：

```bash
./run_gazebo_betaflight_truth_png_demo.sh --duration-s 8 --rate-hz 30 \
  --trajectory-prefix smoke_png_full_thrust_8s --start-lateral-m -10 \
  --target-altitude-offset-m 1.5 --interceptor-speed 4 \
  --max-tilt-deg 8 --max-total-accel 3 --max-speed-hold-accel 1.5 \
  --hover-thrust 1.0 --min-thrust 1.0 --max-thrust 1.0 \
  --spool-s 1.0 --spool-thrust 0.8 --attitude-p 3 \
  --yaw-p 1.0 --max-yaw-rate-deg-s 45
```

结果：距离从约 `80.6m` 降到 `65.8m`，CSV 为 `logs/gazebo_betaflight_truth_png/smoke_png_full_thrust_8s.csv`。

默认头对头命中测试：

```bash
./run_gazebo_betaflight_truth_png_demo.sh --duration-s 12 --rate-hz 30 \
  --trajectory-prefix smoke_png_headon_offset10 --hit-radius-m 2.0 \
  --target-altitude-offset-m 1.0
```

结果：触发 `hit=1`，最小距离 `1.979m`，CSV 为 `logs/gazebo_betaflight_truth_png/smoke_png_headon_offset10.csv`。

带横向偏置测试：

```bash
./run_gazebo_betaflight_truth_png_demo.sh --duration-s 12 --rate-hz 30 \
  --trajectory-prefix smoke_png_body_velocity_thr08_12s --hit-radius-m 2.0 \
  --start-lateral-m -10 --target-altitude-offset-m 1.5 \
  --hover-thrust 0.80 --max-thrust 0.90
```

结果：中段稳定收敛，最小距离约 `3.56m`；末端主要误差来自高度保持和姿态/推力耦合。

## 已知真值位置拦截测试

本组测试不使用视觉检测，目标位置和速度来自脚本真值状态，直接计算 PNG 后走 `RateCommand -> MSP RC -> Betaflight`。

| 工况 | 命中半径 | 结果 | 最近距离 | 最近时刻 | 最近点残差 dx/dy/dz |
| --- | ---: | --- | ---: | ---: | --- |
| 80m 头对头 | 2m | 命中 | `1.967m` | `10.00s` | `-0.000 / 0.991 / -1.699m` |
| 80m, lateral -10m | 2m | 命中 | `1.905m` | `11.00s` | `-0.254 / 1.872 / -0.244m` |
| 80m, lateral -20m | 2m | 未命中 | `17.805m` | `13.73s` | `17.727 / 0.051 / 1.660m` |

CSV：

- `logs/gazebo_betaflight_truth_png/truth_known_headon_80m.csv`
- `logs/gazebo_betaflight_truth_png/truth_known_lateral10_80m.csv`
- `logs/gazebo_betaflight_truth_png/truth_known_lateral20_80m.csv`

`-20m` 横向偏置失败时，拦截机约 7s 后触地并停在 `z≈0.03m`，之后虽然 PNG 仍持续输出 RC，但机体已基本失去横向机动能力。因此该失败优先归因于末端姿态/高度/推力耦合，而不是已知真值 PNG 算法本身不可用。

## 当前结论

Gazebo-only 控制链路已跑通，已知真值位置在头对头和 `-10m` 横向偏置下可进入 `2m` 命中半径。后续重点应转向更大横向偏置工况的末端高度控制、Betaflight rate 参数和 Gazebo 模型推力标定。
