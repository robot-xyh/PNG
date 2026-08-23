# YOLO+ByteTrack Upward-Camera PNG Win10/RTX 3080 仿真包

本包面向 Windows 10 22H2（build 19045）和 NVIDIA GeForce RTX 3080，用于高性能批量复现固定竖直向上相机的 YOLO+ByteTrack 视觉比例导引实验。闭环包含 LOS 滤波、TTC/固定 V_m 导引和末端盲区外推。命中只以 AirSim collision 为准；near-hit 只用于诊断。主测试不启用 AirSim detect 影子链路。

## 运行架构

- AirSim Blocks 1.8.1：Windows 原生运行，RPC 固定为 `127.0.0.2:41451`。
- YOLO+ByteTrack/Python：Windows Python 3.10 + CUDA 12.6 PyTorch，使用 RTX 3080 FP16 推理。
- PX4 v1.11.3：在专用 `PNG-PX4-Ubuntu20.04` WSL1 发行版中运行。
- PX4/AirSim SITL：利用 WSL1 共享 localhost，保持 `127.0.0.1` 的 TCP 4560 和 UDP 14540/14550。

本包不使用 WSL2 mirrored networking，不修改 `.wslconfig`，不转换或删除已有 WSL 发行版。

## 前置条件

- Windows 10 22H2 x64，build 19045，具备管理员权限。
- RTX 3080（至少 8 GiB 显存）和 NVIDIA Windows 驱动 560.76 或更新版本。
- 至少 30 GB 可用磁盘空间和稳定网络。
- Windows App Installer/`winget`；安装器需要它自动安装 Python 3.10。

安装器不会安装或替换显卡驱动。建议将压缩包解压到短路径，例如 `C:\PNG_Win10_RTX3080`。

## 一键安装

双击或在 `cmd.exe` 中运行：

```bat
install_windows.bat
```

安装器会完成以下操作：

1. 检查 Win10 build、NVIDIA 驱动和 RTX 3080 能力。
2. 安装 Python 3.10、`.venv` 和固定版本 CUDA/YOLO 依赖。
3. 执行 FP16 矩阵计算和随机图像 YOLO 预热/性能自检。
4. 下载并校验 Windows AirSim Blocks 1.8.1。
5. 启用 WSL1，导入固定 Ubuntu 20.04 rootfs，编译 PX4 v1.11.3。

首次启用 WSL1 后可能返回 3010。此时重启 Windows，再次运行同一个 `install_windows.bat`即可继续。安装阶段记录在 `runtime/install_stage.txt`。

## 运行实验

安装完成后先分层执行冒烟测试：

```bat
run_experiments.bat smoke fast
run_experiments.bat smoke sitl
```

批量运行格式：

```bat
run_experiments.bat <smoke^|standard^|overnight> <fast^|sitl^|all>
```

`standard fast` 包含 Matrix15 与 S30/S35/S40/S45/S50、TTC/VM、3 次重复，共 120 个 case。`standard sitl` 包含 M01/M05/M10/M14/S35/S40、TTC/VM、2 次重复，共 24 个 case。

- `fast`：Windows 原生 SimpleFlight，单个 Blocks 进程跨 case 复用，runner 通过 `client.reset()` 隔离轮次。
- `sitl`：Windows AirSim + WSL1 PX4，每个 case 重启 PX4 和 Blocks，避免飞行控制状态污染。

不要同时启动多个 Blocks。运行器会检查 41451、4560、14540、14550、14580 端口；如果已被占用，它会报出 PID 并停止，不会删除非本轮启动的进程。

中断后可使用原 run-id 继续：

```bat
run_experiments.bat standard all --run-id run_001 --resume
```

## 输出与性能判定

每次运行保存到 `outputs/<run_id>/`：

- `environment.json`：Windows build、GPU/显存、驱动、CUDA、FP16、WSL 版本、PX4 标签和依赖版本。
- `case_plan.json`：展开后的全部工况、导引律和重复轮次。
- `cases/<case_key>/`：原始 CSV、命令日志、进程日志、meta 和判定 JSON。
- `summary.csv`、`summary.json`、`report.md`和 `plots/`：聚合结果和曲线。

帧数、YOLO 检测帧率、闭环帧率、仿真时钟比和 deadline miss 超出 `config/windows_scenarios.json` 阈值时，该 case 标记为 `infrastructure_valid=false`，不纳入算法命中率。

## 包校验

```bat
.venv\Scripts\python.exe -m unittest discover -s tests -v
.venv\Scripts\python.exe tools\check_package.py --runtime
.venv\Scripts\python.exe tools\windows_experiments.py --preset standard --tier all --dry-run
```

`MANIFEST.sha256` 用于核对发布包内容。`.venv/`、`runtime/`、`outputs/` 和缓存不进入发布 ZIP。

## 常见问题

- `CUDA unavailable`：升级 NVIDIA Windows 驱动并重启，然后重跑安装器。
- `must be WSL1`：检查 `wsl -l -v`。本包只使用 `PNG-PX4-Ubuntu20.04` version 1，不依赖其他发行版。
- `ports are already occupied`：先关闭已有 Blocks/PX4/SITL 任务，确认端口释放后再运行。
- 检测帧率不达标：停止录屏、GPU 占用程序和 Windows 节能模式，再从新 run-id 重跑。
