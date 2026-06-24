# YOLO+ByteTrack 与 YOLO+KCF terminal_v2 50-100m 对比报告

## 实验设置

|项目|设置|
|---|---|
|控制对象|PX4 SITL 拦截机 + Quadrotor1 Actor 目标|
|终端策略|`TERMINAL_PROFILE=terminal_v2`|
|检测/跟踪方案 A|`DETECTOR_SOURCE=yolo_bytetrack`|
|检测/跟踪方案 B|`DETECTOR_SOURCE=yolo_kcf`，`KCF_YOLO_PERIOD_N=8`，`KCF_YOLO_PERIOD_S=0.5`，`KCF_MAX_COAST_S=0.8`|
|距离|50/60/70/80/90/100 m|
|导引律|TTC PNG 与固定 `V_m` PNG|
|相机|前移 0.5m，pitch 0deg，640x480|
|报告来源|`YOLO_ByteTrack_terminal_v2_50_100测试报告.md` 与 `YOLO_KCF_terminal_v2_50_100测试报告.md`|

## TTC 结果对比

|距离|ByteTrack 命中|ByteTrack 最小距离 m|ByteTrack 检测/有效|KCF 命中|KCF 最小距离 m|KCF 检测/有效|KCF track 占比|KCF lost 占比|KCF 主要拒绝原因|
|---:|---|---:|---|---|---:|---|---:|---:|---|
|50|False|2.972|203/235 / 203|False|4.150|210/232 / 207|49.6%|0.0%|kcf_update_failed|
|60|False|2.860|246/271 / 251|False|3.246|229/269 / 229|47.6%|0.4%|kcf_update_failed|
|70|False|3.349|235/308 / 239|False|4.835|229/305 / 229|43.3%|1.6%|kcf_update_failed|
|80|False|4.148|289/342 / 292|False|4.598|274/335 / 224|47.8%|0.9%|kcf_update_failed|
|90|False|3.521|261/350 / 267|False|62.137|69/352 / 28|11.1%|72.4%|kcf_coast_timeout|
|100|False|5.305|235/349 / 251|False|5.051|227/347 / 231|36.6%|4.0%|kcf_update_failed|

## VM 结果对比

|距离|ByteTrack 命中|ByteTrack 最小距离 m|ByteTrack 检测/有效|KCF 命中|KCF 最小距离 m|KCF 检测/有效|KCF track 占比|KCF lost 占比|KCF 主要拒绝原因|
|---:|---|---:|---|---|---:|---|---:|---:|---|
|50|False|3.290|193/237 / 194|False|4.400|197/235 / 185|45.5%|5.1%|kcf_update_failed|
|60|False|3.323|238/273 / 246|False|3.560|221/261 / 221|46.4%|0.4%|kcf_update_failed|
|70|False|2.790|143/308 / 141|False|3.922|182/297 / 181|33.3%|17.8%|kcf_update_failed|
|80|False|3.446|266/342 / 274|False|38.387|144/335 / 109|26.6%|42.1%|kcf_coast_timeout|
|90|False|2.603|246/351 / 252|False|5.061|188/342 / 191|30.1%|10.2%|kcf_update_failed|
|100|False|3.428|214/351 / 221|False|25.674|141/346 / 100|23.4%|41.3%|kcf_coast_timeout|

## 结论

- 本轮 24 个工况均未发生碰撞命中。
- ByteTrack 在大多数距离上最小距离更小，检测/有效帧比例整体更稳定；KCF 在 80-100m 多个工况出现 `lost` 或 `kcf_coast_timeout` 占比升高，导致目标跟踪中断后导引发散。
- KCF 并没有显著提高有效闭环频率；本轮 `detector_fps` 仍约 8.5-9.3 FPS，说明当前耗时主要仍受 AirSim 图像获取、YOLO 周期校正、PX4/仿真循环共同限制。
- 如果继续优化 KCF，优先方向不是单纯延长 coast，而是降低 KCF 漂移：缩短 YOLO 校正周期、提高 `kcf_min_yolo_iou` 后强制重识别、或者改为 CSRT/光流+YOLO 校正做对比。

## 文件

- ByteTrack 报告：`完整方案/YOLO_ByteTrack_terminal_v2_50_100测试报告.md` / `.docx`
- KCF 报告：`完整方案/YOLO_KCF_terminal_v2_50_100测试报告.md` / `.docx`
- ByteTrack stamp：`yolo_bytetrack_terminal_v2_20260624_083910`
- KCF stamp：`yolo_kcf_terminal_v2_20260624_085728`
