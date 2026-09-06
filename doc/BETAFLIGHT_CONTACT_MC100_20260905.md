# Betaflight 接触终端 LUT 绑定 MC100 报告（2026-09-05）

## 1. 场景配置

| 项 | 值 |
| --- | --- |
| 仿真模式 | 三维点质量 + 一阶机体角速度/推力响应 |
| 拦截机/目标机 | 1 / 1 |
| 工况 | 15 个基础工况及其镜像，共 30 个 |
| 每工况试次 | 100 |
| 总结果行数 | 18,000（3 场景 x 2 评估 x 30 工况 x 100） |
| seed | 20260903，按场景稳定派生 |
| 积分步长/最长时长 | 0.01 s / 40 s |
| 相机半视场 | 水平 31.0007 度，垂直 24.9154 度 |
| 控制/感知/运动学频率 | 50 Hz / 30 Hz / 5 Hz |
| 目标检测陈旧门限 | 0.25 s |
| 接触终端 | bbox 0.05 或 TTC 1.0 s 进入 TERMINAL_VISUAL；bbox 0.25 COMPLETE；盲推衰减最长 0.20 s |
| 油门/电压包络 | 1200-1500 us / 22.0-25.2 V |

三个场景分别采用：

| 场景 | 电压 | 感知延迟 |
| --- | ---: | ---: |
| final_chain_software_p95 | 25.2 V | 84.556 ms |
| observed_active_flight_p95 | 22.6 V | 127.700 ms |
| conservative_physical_p95_budget | 22.0 V | 154.139 ms |

## 2. 算法与绑定

运行配置为 `config/betaflight.rk3588.velocity_png.flight_contact_supervised.json`，scope 为
`flight_contact_supervised_v1`。配置从四通道监督飞行配置派生，但使用独立批准路径和接触策略；主动输出仍要求新 scope 的批准文件及显式无时限接管风险豁免。

推力模型为电压相关二维 LUT：

- calibration ID：`6S_2412KG_3115_900KV_1050R_HIST15_PHYSICS_V1_20260905`
- LUT SHA256：`c89babfd2458626e4e26e3ec82a11ae05ae799490e9797b67c9a1e46dc6579be`
- 运行配置 SHA256：`c89185193db246d8ca8db9950d5733720a9a62b8fbeb61d10dc106d2b3c23b47`
- 推力一阶时间常数：68.98 ms
- LUT 留出集：1,482 个样本，中位误差 4.21%，P95 误差 13.97%

控制链使用速度建立 PNG、FOV 优先分配、`7 m/s²` 总加速度上限、60 deg/s Roll/Pitch 上限及 LUT 反插值油门。

## 3. MC100 结果

以下为启用 FOV 优先级的接触候选结果；项目核心门槛要求每个场景命中率和 FOV 命中率均不低于 80%。

| 场景 | 可见样本 | 命中率 | FOV 命中率 | 80%目标 |
| --- | ---: | ---: | ---: | --- |
| final_chain_software_p95 | 2,600 | 99.92% | 99.81% | 通过 |
| observed_active_flight_p95 | 2,600 | 99.96% | 99.81% | 通过 |
| conservative_physical_p95_budget | 2,600 | 88.54% | 87.58% | 通过 |

最差命中率为 88.54%，最差 FOV 命中率为 87.58%。`initial_performance_target_passed=true`，FOV 优先级配对筛选通过并选择 `contact_fov_s75_f95`。

附加指标：

| 场景 | 目标陈旧失败 | 测量有效比例 | 速度项饱和 | 总加速度饱和 |
| --- | ---: | ---: | ---: | ---: |
| final_chain_software_p95 | 0.00% | 98.61% | 5.69% | 0.57% |
| observed_active_flight_p95 | 0.00% | 98.08% | 5.88% | 0.63% |
| conservative_physical_p95_budget | 11.35% | 88.09% | 5.19% | 0.59% |

## 4. 结果分析

核心 80% 命中目标已经在三种电压/延迟场景下通过。LUT 加入后没有造成控制分配饱和：速度项和总加速度饱和均明显低于 40% 门槛。

原 `150 ms` 目标陈旧门限无法覆盖 `127.7-154.139 ms` 感知延迟、约 33 ms 帧间隔和两帧漏检突发，预筛会周期性触发 ABORT。接触配置将该门限调整为 `250 ms`；角速度响应延迟、加速度上限、油门包络以及终端盲推 `200 ms` 上限均未放宽。

完整发布结论仍为 `release_passed=false`。原因是保守场景的目标陈旧失败率为 11.35%，超过附加的 1% 门槛，同时测量有效比例 88.09% 低于 90%。因此本报告证明接触策略的初始命中性能达到项目 80% 目标，但不构成主动飞行批准。

## 5. 验证

- 单元测试：`python3 -m unittest discover -s tests -v`
- 结果：495/495 通过。
- MC100 配对筛选：通过。
- 配置与 LUT 均由结果 JSON 记录绝对路径、calibration ID 和 SHA256。

复现命令：

```bash
python3 tools/run_betaflight_intercept_monte_carlo.py \
  --config config/betaflight.intercept_eval.flight_contact_mc100_20260905.json \
  --output logs/betaflight_intercept_contact_20260905/betaflight_intercept_contact_mc100_20260905.json \
  --csv logs/betaflight_intercept_contact_20260905/betaflight_intercept_contact_mc100_20260905.csv \
  --workers 20
```

## 6. 文件索引

| 文件 | SHA256 |
| --- | --- |
| `logs/betaflight_intercept_contact_20260905/betaflight_intercept_contact_mc100_20260905.json` | `d4a7d9db1a4431a4819e0bc6039004ec8a017bfcd9c34bb60491e5567ac142a9` |
| `logs/betaflight_intercept_contact_20260905/betaflight_intercept_contact_mc100_20260905.csv` | `daf00c0997d5eb789cb090cebf5fda45ed4f6b21b67ea3c375ec3b5120fc2026` |
| `config/betaflight.intercept_eval.flight_contact_mc100_20260905.json` | `db96424939f73215ad337e7ba05e0b7301900c7f6c877c02b00fff3be9f6f5c4` |
| `config/betaflight.rk3588.velocity_png.flight_contact_supervised.json` | `c89185193db246d8ca8db9950d5733720a9a62b8fbeb61d10dc106d2b3c23b47` |
| `config/betaflight.thrust_lut.6s_22v_1500.physics_v1.json` | `c89babfd2458626e4e26e3ec82a11ae05ae799490e9797b67c9a1e46dc6579be` |

本轮未生成图表；JSON 已包含逐场景汇总，CSV 保留全部 18,000 条逐试次结果。
