# YOLO+ByteTrack 上视相机 PNG Ubuntu 仿真包

本包用于在 Ubuntu 上批量复现 YOLO+ByteTrack、LOS 滤波、TTC/固定 $V_m$ 比例导引和末端盲区外推实验。闭环仅使用 YOLO+ByteTrack 视觉量，不启用 AirSim detect 影子链路；真实 LOS 根据位置真值离线计算。命中仅以 AirSim collision 为准，near-hit 只作诊断。

## 系统要求

- Ubuntu 22.04 或 24.04 x86_64，Python 3.10-3.12；
- NVIDIA GPU 及已正常工作的驱动，建议 8 GB 以上显存和 32 GB 内存；
- 至少 20 GB 可用磁盘空间，安装时能访问 GitHub、PyPI 和 PyTorch wheel 源；
- Ubuntu 桌面会话或可用的 X11 环境。Blocks 以离屏、无界面参数运行。

## 一键安装

```bash
chmod +x install_ubuntu.sh run_experiments.sh
./install_ubuntu.sh
```

安装器会安装 apt 构建库，创建 `.venv`，安装固定版本的 CUDA PyTorch/视觉依赖，部署官方 AirSim Blocks 1.8.1，并构建原生 PX4 v1.11.3 SITL。安装器不会安装或替换 NVIDIA 驱动，也不会将 PX4 升级到其他版本。Ubuntu 24.04 所需的 PX4 类型转换补丁会幂等应用。

已有 Blocks 或 PX4 时可避免重复下载：

```bash
BLOCKS_DIR="$HOME/Downloads/Blocks/LinuxBlocks1.8.1" \
PX4_DIR="$HOME/PX4/PX4-Autopilot" ./install_ubuntu.sh
```

`PX4_DIR` 必须精确位于 `v1.11.3`。实际路径记录在 `runtime/blocks_path.txt` 和 `runtime/px4_path.txt`。

## 运行实验

先执行快速冒烟测试：

```bash
./run_experiments.sh smoke fast
```

批量命令格式为：

```bash
./run_experiments.sh <smoke|standard|overnight> <fast|sitl|all>
```

`standard fast` 包含 Matrix15 与 S30/S35/S40/S45/S50、TTC/VM、3 次重复，共 120 个 case。`standard sitl` 包含 M01/M05/M10/M14/S35/S40、TTC/VM、2 次重复，共 24 个 case。

- `fast`：原生 SimpleFlight 快速统计层；单个 Blocks 进程跨 case 复用，runner 每轮调用 `client.reset()`。
- `sitl`：原生 AirSim + PX4 v1.11.3 软件在环；PX4 通过 TCP 4560 与 AirSim 连接，通过 MAVLink body-rate 接收导引命令，每个 case 严格重启 PX4 和 Blocks。

不要同时启动多个 Blocks。Python RPC 固定使用 `127.0.0.2:41451`，PX4/AirSim SITL 通信保持 `127.0.0.1`。脚本启动前检查 `41451/4560/14540/14550/14580`；如果被占用则直接报错，不会终止非本脚本启动的进程。

## 性能与恢复

配置默认使用 640x480 原始 AirSim 图像、CUDA FP16、YOLO 预热、无预览录像和离屏渲染。检测/闭环帧率、仿真时钟比、deadline miss 或 RPC/MAVLink 异常超过 `config/ubuntu_scenarios.json` 门限时，该轮标记为 `infra_invalid`，自动重试一次，并从算法命中率中剔除。

断点续跑示例：

```bash
./run_experiments.sh standard all --run-id run_001 --resume
```

结果位于 `outputs/<run_id>/`，包含逐 case CSV/meta/日志/判定 JSON、`cases.csv`、`summary.json`、`plots/`、`Ubuntu仿真批量测试报告.md` 和环境版本记录。`fast` 与 `sitl` 分别统计，不直接合并命中率。

## 校验与资料

```bash
.venv/bin/python tools/ubuntu_experiments.py --preset standard --tier all --dry-run
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python tools/check_package.py --runtime
```

算法说明、历史报告和代表性实验数据保留在 `doc/`、`完整方案/` 和 `logs/`，不参与新批次的统计。
