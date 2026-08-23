# YOLO+ByteTrack 上视相机 PNG 算法资料包

生成日期：2026-08-07

## 内容

- `doc/`：算法说明 Markdown、Word 文件，以及文档中使用的曲线、LOS 对比图和指标 CSV。
- `完整方案/`：本文引用的源报告和报告资产。
- `logs/yolo_sitl_ttc_vm/`：文档中四个典型案例对应的原始 CSV 与 meta JSON。
- `vision_guidance/`：LOS 几何、滤波、TTC、PNG、末端外推、YOLO+ByteTrack 检测等核心源代码，包含 `best.pt` 权重文件。
- `examples/`：AirSim/PX4 上视相机闭环 runner 与报告生成脚本。
- `tests/`：与几何、LOS、TTC、末端外推、YOLO+ByteTrack、strapdown 相关的单元测试。
- `config/` 和根目录 shell 脚本：复现实验需要的 AirSim/PX4 配置与启动入口。

## 主要文档

- `doc/YOLO_ByteTrack_upward_camera_PNG算法说明.md`
- `doc/YOLO_ByteTrack_upward_camera_PNG算法说明.docx`

## 典型实验日志

- `S 35m TTC hit`：`logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_upward_baseline_s_maneuver_30_50_20260701_231523_r35_h30.csv`
- `Straight 35m TTC hit`：`logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_upward_final_yolo_35_40_20260628_173525_r35_h30.csv`
- `S 40m TTC miss`：`logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_upward_baseline_s_maneuver_30_50_20260701_231523_r40_h30.csv`
- `Straight M05 40m TTC miss`：`logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_upward_yolo_matrix15_20260701_202024_M05_r40_h30.csv`

## 复现提示

运行前需要本机 AirSim Blocks、PX4 SITL/HIL 和 Python 依赖环境。多 Blocks 并行时建议设置：

```bash
export AIRSIM_RPC_HOST=127.0.0.2
```

单元测试可从仓库根目录运行：

```bash
python3 -m unittest discover -s tests -v
```
