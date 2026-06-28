# ABC PX4 body-rate / AirSim detect 80m 测试记录

## 当前状态

本轮已补齐 A 工况所需的最小代码入口：`examples/run_airsim_strapdown_vision_png.py` 新增 `--guidance-source truth`。该模式绕过检测框 LOS/TTC，直接使用 AirSim 目标真值相对位置/速度计算 PNG，并继续复用现有 `accel_body_rate + mavlink_body_rate` 下游控制链路。

默认行为保持 `--guidance-source vision`，B/C 的 AirSim detect 路径不变。

## A/B/C 定义

- A：`--guidance-source truth --guidance-output-mode accel_body_rate --px4-command-mode mavlink_body_rate`，只验证真值 PNG 到 PX4 body-rate/thrust 的控制链路。
- B：云台脚本 + `--detector-source airsim`，用于判断云台锁定后是否仍因视场导致目标丢失。
- C：捷联脚本 + `--detector-source airsim` + body-rate，用 AirSim 理论检测框隔离 YOLO 识别断续影响。

## 已完成验证

已通过：

```bash
python3 -m py_compile examples/run_airsim_strapdown_vision_png.py tests/test_strapdown_vision_png.py
PYTHONPATH=. python3 tests/test_strapdown_vision_png.py
PYTHONPATH=. python3 tests/test_truth_png.py
python3 examples/run_airsim_strapdown_vision_png.py --help | rg -n "guidance-source|guidance-output-mode|px4-command-mode"
```

测试结果：`test_strapdown_vision_png` 共 43 项通过，`test_truth_png` 共 8 项通过。

## 实机仿真尝试

A 的 80m smoke 已按端口流程启动：

- env：`logs/abc_intercept/abc_A_truth_body_rate_20260627_225016.env`
- PX4 log：`logs/abc_intercept/px4_abc_A_truth_body_rate_20260627_225016.log`
- Blocks log：`logs/abc_intercept/blocks_abc_A_truth_body_rate_20260627_225016.log`

结果：PX4 成功等待并连接 AirSim TCP 4560，但 Blocks 在连接后退出，末尾为 `Exception occurred while updating world`，未进入 Python 拦截脚本，未生成 CSV。

随后端口探测显示当前机器已有其他 PX4/Blocks 实例占用默认端口，port guard 会自动改写到 `41452/4561`。后续 A/B/C 实测应在干净仿真环境中继续，或使用改写后的 env 同步启动对应 PX4 instance。

## 后续运行建议

先只跑 A 的 80m smoke，确认 CSV 中：

- `guidance_source=truth`
- `guidance_mode=truth_png`
- `body_rate_control_active=1`
- `body_rate_p/q/r_*` 和 `body_rate_thrust` 有有限值

A 跑通后再按同一端口流程重启栈，分别跑 B 和 C，比较 `detected` 比例、`reject_reason`、bbox clipping、最小距离和 hit 结果。
