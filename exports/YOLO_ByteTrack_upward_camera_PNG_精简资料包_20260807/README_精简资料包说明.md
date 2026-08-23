# YOLO+ByteTrack 上视相机 PNG 精简资料包

本资料包只保留同行交流和复现实验曲线所需的关键内容，不包含完整日志、单元测试和无关报告资产。

## 内容说明

- `doc/`：当前算法说明文档，包含 Markdown 与 Word 两个版本。
- `doc/assets/YOLO_ByteTrack_upward_camera_PNG/`：当前报告引用的实验曲线、LOS 对比图和指标 CSV。
- `vision_guidance/`：上视相机 PNG 链路的关键代码，包括 YOLO+ByteTrack 检测、LOS 几何、滤波、TTC、末端外推、PNG 评估和 `best.pt` 权重。
- `examples/run_airsim_strapdown_vision_png.py`：AirSim/PX4 上视相机闭环实验主程序。
- `config/airsim_blocks_px4_actor_upward_camera_settings.json`：AirSim Blocks/PX4/Actor 上视相机设置示例。
- `run_yolo_sitl_ttc_vm_batch.sh`：YOLO+ByteTrack TTC/VM 批量实验入口。

## 推荐阅读顺序

1. 打开 `doc/YOLO_ByteTrack_upward_camera_PNG算法说明.docx` 或同名 Markdown。
2. 对照 `doc/assets/YOLO_ByteTrack_upward_camera_PNG/` 查看实验曲线与指标 CSV。
3. 查看 `examples/run_airsim_strapdown_vision_png.py` 中闭环流程。
4. 查看 `vision_guidance/` 中检测、LOS 滤波、TTC/VM、盲区外推等关键模块。

## 运行提示

运行 AirSim/PX4 多实例前建议设置独立 RPC 地址：

```bash
export AIRSIM_RPC_HOST=127.0.0.2
```

具体实验参数以 `config/` 和 `run_yolo_sitl_ttc_vm_batch.sh` 为准。
