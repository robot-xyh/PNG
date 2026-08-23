# YOLO+ByteTrack 上视相机 PNG Windows 仿真包

本包面向 Windows 11 22H2 及以上、NVIDIA GPU，用于批量复现 YOLO+ByteTrack、LOS 滤波、TTC/V_m 比例导引和末端盲区外推实验。命中仅以 AirSim collision 为准；near-hit 只作诊断。主测试不启用 AirSim detect 影子链路，真实 LOS 根据真值位置离线计算。

## 一键安装

1. 安装当前 NVIDIA Windows 驱动并重启。
2. 双击 `install_windows.bat`，接受 UAC 提示。
3. 若脚本要求重启 Windows，重启后再次双击同一文件。

安装器会配置 Python 3.10、`.venv`、固定版本 CUDA 依赖、AirSim Blocks 1.8.1、WSL2 Ubuntu 20.04、mirrored networking 和 PX4 v1.11.3。安装器不会安装或更换显卡驱动，也不会把 PX4 自动升级到其他版本。

## 运行实验

先运行快速冒烟测试：

```bat
run_experiments.bat smoke fast
```

批量命令格式：

```bat
run_experiments.bat <smoke^|standard^|overnight> <fast^|sitl^|all>
```

`standard fast` 包含 Matrix15 与 S30/S35/S40/S45/S50、TTC/VM、3 次重复，共 120 个 case。`standard sitl` 包含 M01/M05/M10/M14/S35/S40、TTC/VM、2 次重复，共 24 个 case。

- `fast`：Windows 原生 SimpleFlight；单个 Blocks 进程跨 case 复用，每轮由 runner 调用 `client.reset()`。
- `sitl`：Windows AirSim + WSL2 PX4 v1.11.3；每个 case 严格重启 PX4 和 Blocks。

不要同时启动多个 Blocks。Python RPC 固定使用 `127.0.0.2`；PX4 与 AirSim 的 SITL TCP/UDP 保持 `127.0.0.1`。

## 断点续跑与输出

指定固定 run ID 后可恢复运行：

```bat
run_experiments.bat standard all --run-id run_001 --resume
```

只有已完成且基础设施有效、配置哈希一致的 case 会被跳过。输出位于 `outputs/<run_id>/`：

- `cases/<case>/`：逐 case CSV、meta、命令、控制台日志和判定 JSON；
- `cases.csv`、`summary.json`：批量指标；
- `Windows仿真批量测试报告.md`、`plots/`：中文报告和汇总图；
- `environment.json`：Windows、Python、CUDA、GPU 和依赖版本。

检测/闭环帧率、仿真时钟比、deadline miss 或 RPC/MAVLink 异常超过 `config/windows_scenarios.json` 门限时，该轮标为 `infra_invalid`，自动重试一次，并从算法命中率中剔除。SimpleFlight 与 PX4 SITL 的结果分别统计。

## 参数与验证

场景、重复次数、性能门限和闭环参数集中在 `config/windows_scenarios.json`。修改后建议先检查展开计划：

```bat
.venv\Scripts\python.exe tools\windows_experiments.py --preset standard --tier all --dry-run
.venv\Scripts\python.exe -m unittest discover -s tests -v
.venv\Scripts\python.exe tools\check_package.py --runtime
```

算法说明和历史实验数据仍在 `doc/`、`完整方案/` 和 `logs/`，不参与新的批量统计。
