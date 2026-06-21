# YOLO + ByteTrack PX4 SITL frame_centering hold velocity 50m 测试报告

## 1. 实验目的

按照此前已命中的 YOLO 案例配置，改用真正 PX4 SITL actor 场景，比较两种捷联视觉比例导引：

- `TTC` 组：`ttc_png`，TTC 只参与增益调度，并保留 LOS/Vm soft guidance。
- `VM` 组：`fixed_vm_png`，不使用 TTC，固定 `N * V_m` 导引增益。

只测试 50m，验证 loss_hold 保持上一帧速度后的效果。

## 2. 基准条件

|参数|值|
|---|---|
|stamp|`frame_centering_holdv_50m_20260621_055053`|
|settings|`/home/linux/Documents/PNG/config/airsim_blocks_px4_actor_clock0p2_settings.json`|
|拦截机|`PX4 SITL / velocity_yaw_rate`|
|目标 actor|`IntruderActor`|
|actor asset|`Quadrotor1`|
|actor scale|`1.0`|
|检测源|`yolo_bytetrack`|
|YOLO model|`vision_guidance/best.pt`|
|YOLO device|`0` runtime `cuda:0`|
|YOLO conf / iou / imgsz|`0.1` / `0.7` / `640`|
|tracker|`bytetrack.yaml`，single target `1`|
|相机外参|`x=0.5, y=0.0, z=0.0`|
|FOV / resolution|`120.0 deg`, `640x480`|
|高度差|`20.0 m`|
|目标速度 / speed ratio|`5.0 m/s` / `2.0`|
|rate_hz|`8.0`|
|LOS filter|`1`|
|frame_guard|`True`|
|bbox noise|`0`|

## 3. 总览图

![summary](assets/YOLO_SITL_frame_centering_holdv_50m测试报告/summary_frame_centering_holdv_50m_20260621_055053.png)

## 4. 汇总表

|组别|命中数|命中距离m|未命中距离m|最小中心距离m|检测帧/总帧|有效帧/总帧|平均检测FPS|
|---|---:|---|---|---:|---:|---:|---:|
|TTC|0/1|-|50|1.872|188/222|129/222|7.95|
|VM|0/1|-|50|2.267|135/223|133/223|7.98|

## 5. 明细表

|组别|距离m|碰撞|碰撞时间s|最小距离m|终点距离m|检测帧率|有效帧率|YOLO FPS|sim FPS|实际过载max g|指令P95 g|导引评估P95 g|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|TTC|50|0|-|1.872|19.311|84.7%|58.1%|7.95|7.50|1.04|3.09|0.51|
|VM|50|0|-|2.267|42.846|60.5%|59.6%|7.98|7.52|1.08|3.13|0.49|

## 6. 分距离曲线

每个距离一张图，包含真实中心距离、bbox 面积、TTC 估计、实际过载/导引评估过载和 YOLO 检测 FPS。

![50m](assets/YOLO_SITL_frame_centering_holdv_50m测试报告/yolo_sitl_ttc_vm_050m.png)

## 7. 结论

- TTC: 命中 `0/1`，命中距离 `-`，未命中 `50m`，检测帧比例 `84.7%`，有效导引帧比例 `58.1%`，平均检测 FPS `7.95`。
- VM: 命中 `0/1`，命中距离 `-`，未命中 `50m`，检测帧比例 `60.5%`，有效导引帧比例 `59.6%`，平均检测 FPS `7.98`。
- 本轮使用真实 YOLOv8 + ByteTrack，因此检测连续性和 GPU 推理速度会直接进入闭环；结果不能和 AirSim detect 函数的理想 bbox 直接等价比较。
