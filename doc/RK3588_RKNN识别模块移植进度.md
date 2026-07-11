# RK3588 RKNN 识别模块移植进度

## 1. 移植目标与约束

本次工作将 Orange Pi 5 Max 上 `/home/orangepi/src/circle_pilot` 的修改版无人机识别模块
接入当前 Python Betaflight 工程。模型不是标准 Ultralytics YOLO 输出，因此没有使用
`vision_guidance/best.pt` 的通用解码逻辑，而是复用原工程的 RKNN engine、多头 DFL
解码、letterbox 反变换、NMS 和检测过滤代码。当前阶段只允许无桨 `log_only`，原生识别库
不包含 MSP 或 RC 输出能力。

## 2. 来源与模型契约

移植来源包括：

| 功能 | `src/circle_pilot` 来源 |
|---|---|
| RKNN 初始化与张量读取 | `adapters/common/perception/src/rknn_engine.cpp` |
| RGB letterbox | `adapters/common/perception/include/circle/perception/letterbox.hpp` |
| DFL、坐标恢复和 NMS | `core/src/vision/yolo_postprocess.cpp` |
| score/类别/面积/长宽比过滤 | `core/include/circle/vision/detection_filter.hpp` |
| 修改模型 | `drone_v8n_v21_kd_relu_lambda008_640_640-rk3588.rknn` |

模型已保存到 `models/rknn/`，SHA256 为
`ad905c19e3e2b5386fa1a5d562285a02d5e5a75ad02d89ce2f1d344810c60f59`。
RK3588 实际查询得到 640x640 RGB 输入和四个 NCHW 输出：

```text
box_8  [1, 64, 80, 80]   cls_8  [1, 1, 80, 80]
box_16 [1, 64, 40, 40]   cls_16 [1, 1, 40, 40]
```

box head 的 64 通道对应四条边各 16 个 DFL bin。后处理对每条边执行 softmax 期望值
计算，再乘 stride 得到边界距离；类别 logit 经 sigmoid、置信度门限和 NMS 后，按
letterbox 的 scale/padding 映射回 640x512 图像。不能将其替换为标准 YOLO 单输出解析。

## 3. 已完成修改

### 3.1 原生 RKNN 桥

新增 `native/rknn_detector/`：

- `vendor/circle_pilot/` 保存从板端复制且未修改的识别核心源码。
- `src/rknn_detector_bridge.cpp` 提供稳定 C ABI，负责模型加载、RGB 预处理、NPU 推理、
  DFL/NMS、检测过滤和结果输出。
- `CMakeLists.txt` 在 RK3588 上链接 `/usr/lib/librknnrt.so` 和
  `/usr/local/include/rknn_api.h`。
- bridge 返回 bbox、score、class、候选数量、选择索引和 preprocess/inference/
  postprocess/total 分段耗时。
- C ABI 已升级为 v2：保留原单框接口，并新增容量受限的全候选接口，向 Python 返回 NMS
  后最多 300 个 bbox/score/class；容量不足会设置 `truncated`，不会静默丢框。
- 初始化时导出模型输入、输出 shape、layout、zero point 和 scale，供运行 meta 记录。

### 3.2 Python 接入

新增 `vision_guidance/rknn_native_detector.py`：

- 使用 `ctypes` 加载 `librknn_detector_bridge.so`，检查 ABI 版本和模型文件。
- 校验输入必须为连续 `uint8 HxWx3 RGB`。
- 将原生结果转换为现有 `FrameDetection`，保持 LOS、TTC 和 PNG 接口不变。
- 计算并记录模型 SHA256、原生库路径、输出 schema 和全部识别参数。

修改 `examples/run_betaflight_log_only.py`：

- 新增 `--detector-source rknn_native`、`rknn_bytetrack`、`--rknn-library` 和
  `--rknn-model`。
- 相机仍使用 OpenCV 采集、缩放和去畸变，进入 RKNN 前执行 BGR 到 RGB 转换。
- `detection_exposure_ts` 使用 `capture.read()` 返回后的单调时间；它仍不是硬件曝光时间。
- CSV 新增 `detector_*_count`、`rknn_selected_index`、`rknn_preprocess_ms`、
  `rknn_inference_ms`、`rknn_postprocess_ms` 和 `rknn_total_ms`。
- meta JSON 新增模型哈希、输出张量结构、NPU core mask 和识别阈值。

### 3.3 完整 ByteTrack 与单目标锁定

- 固化 `ultralytics==8.4.71` 的官方 BYTETracker、Kalman、LAP matching 和 track lifecycle
  代码，移除完整 Ultralytics/PyTorch runtime import；板端导入确认 `torch_loaded=false`。
- 使用高分第一阶段和低分第二阶段关联，维护 tracked/lost/removed 状态和真实 track ID。
- 当前目标连续测量 3 帧后才锁定；active track lost 时只保持 ID，不向 LOS/TTC 生成预测框；
  removed 后才允许其他已确认轨迹接管。
- RKNN INT8 低分档需要参与第二阶段，因此实机配置使用 `detector_conf=0.05`、
  `track_low=0.10`、`track_high=0.25`，并显式设置 `fuse_score=false`。报告默认
  `fuse_score=true` 仍作为对照基线保存。
- 新增 tracker state、age、hits、lost frames、高低分候选数、关联 IoU、switch/fragment、
  tracker 耗时和 bbox measurement source 日志。

### 3.4 独立感知 worker

- `rknn_bytetrack` 使用独立 latest-frame worker，目标频率 30 Hz；MSP/CSV 主循环保持 5 Hz。
- worker 到主循环的结果槽容量为 1，新结果覆盖未消费旧结果并累计 queue dropped，禁止形成
  延迟队列。
- 日志记录 perception sequence、实际 tracker FPS、结果帧龄、覆盖数量和 worker error。

### 3.5 配置与测试

新增 `config/betaflight.rk3588.example.json`。识别参数按 `src` 的 PNG 有效配置设置：

```text
conf_threshold=0.20       iou_threshold=0.45
min_score=0.25            max_det=300
min_bbox_area=0           max_bbox_aspect_ratio=3.0
temporal_gating=false     track_hint_max_misses=30
core_mask=7
```

新增 `tests/test_rknn_native_detector.py`，并扩展 `tests/test_betaflight_logging.py`，覆盖
原生结果映射、拒绝原因、RGB 转换、相机时间戳、meta 和 CSV 字段。本地测试结果为
完整测试均通过，新增用例覆盖批量候选、三帧确认、低分关联、lost 不输出、同 ID 恢复、
防抢占和 latest-frame worker。

## 4. 板端部署与验证结果

Python 工程部署在 `/home/orangepi/png_betaflight_python`，原有
`/home/orangepi/src/circle_pilot` 未修改。构建命令：

```bash
cmake -S native/rknn_detector -B native/rknn_detector/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build native/rknn_detector/build -j2
```

验证结果：

- 原生库成功链接板端 `librknnrt.so`，模型初始化成功并确认四输出结构。
- 10 s 相机+MSP+RKNN 联合测试完成 50/50 帧，相机/MSP 错误均为 0，全程
  `LOG_ONLY`、`rc_active=0`。
- 按 `src` 参数修正后完成 5 s、25/25 帧复测：NPU inference 均值/最大为
  6.046/6.905 ms，总识别耗时均值/最大为 6.307/7.283 ms。
- 对 `src` 中前 20 张历史 `*_det-int.jpg` 做静态回归，9 张输出有效 bbox，5 张候选被
  阈值过滤，6 张无候选。样本没有独立 ground truth，该结果只证明解码通路可工作，不能
  作为召回率。
- 完整 ByteTrack 对前 40 张历史帧接收 82 个 NMS 候选，形成两个经三帧确认的轨迹区间；
  lost 帧均未输出 `FrameDetection`。该序列仍没有 ground truth，只用于接口与状态验证。
- 10 s 相机+MSP+RKNN+ByteTrack 同步短测完成 50/50 帧，tracker 均值/最大
  0.285/0.686 ms，NPU 总耗时均值/最大 6.326/7.398 ms，全程 `LOG_ONLY`。
- 30 Hz latest-frame worker 短测实际达到约 27.45 Hz，结果帧龄均值/最大
  1.75/15.87 ms，tracker 均值/最大 0.254/0.305 ms，无 worker 错误或候选截断；MSP
  主循环稳定 5 Hz，全程 `rc_active=0`。
- 最终文件同步后完成 5 s、25 行复测：关键文件与本地 SHA256 一致，控制请求、控制许可和
  `rc_active` 均为 0，MSP/发送/worker 错误及候选截断均为 0；感知约 27.61 Hz，结果帧龄
  均值/最大 1.24/11.02 ms，RKNN 总耗时 5.743/7.531 ms，tracker 耗时 0.251/0.343 ms。

运行方式：

```bash
python3 examples/run_betaflight_log_only.py \
  --config config/betaflight.rk3588.example.json \
  --duration-s 60 --rate-hz 5 \
  --control-mode log_only \
  --detector-source rknn_bytetrack
```

## 5. 完整 ByteTrack 状态与剩余工作

完整 ByteTrack、全候选 ABI、单目标锁定、lost 安全策略和独立 perception worker 已实现。
当前剩余项不是接口移植，而是数据标定和长时间验收：

1. 采集带 ground truth 的真实目标视频，比较 `.pt+ByteTrack` 与 `RKNN+ByteTrack` 的检测率、
   bbox 中心/面积误差、ID switch、fragment 和最长连续漏检。
2. 对 high/low/new/match threshold 和 track buffer 做离线参数搜索，并锁定配置哈希。
3. 完成 30 分钟 NPU 满载、60 分钟相机+MSP 联合老化、温度/频率和内存增长检查。
4. 获取 V4L2 硬件曝光时间戳，并与 Betaflight Blackbox 对齐。

在完成真实目标视频回归、30 分钟 NPU 满载、60 分钟相机+MSP 联合老化和硬件曝光时间戳
改造前，RKNN 检测结果只能用于日志与监督实验，不能直接放开 Betaflight 控制。

## 6. 版本记录

初始 RKNN 移植、配置、测试和部署文档已提交并推送：

```text
0f7d19a Add RK3588 native recognition deployment
```
