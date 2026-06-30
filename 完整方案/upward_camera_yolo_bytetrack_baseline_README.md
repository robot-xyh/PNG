# upward-camera YOLO+ByteTrack baseline

本文档把当前固定竖直上视相机闭环结果固定为 upward-camera baseline。后续 upward-camera 优化应优先和这组结果对比，不应和 AirSim detect 理想检测结果混用结论。

## 1. baseline 定义

- baseline 名称：`upward_camera_yolo_bytetrack_collision_baseline_20260628`
- 检测闭环：`DETECTOR_SOURCE=yolo_bytetrack`
- tracker：`bytetrack.yaml`，启用 single-target 和 untracked fallback
- 影子检测：`SHADOW_AIRSIM_DETECT=1`，仅用于对照分析，不参与制导
- 成功标准：AirSim collision 命中；near-hit 只作为诊断
- AirSim RPC：`AIRSIM_RPC_HOST=127.0.0.2`
- 端口策略：运行前后检查 PX4/Blocks/SITL 端口和残留进程

## 2. 初始条件

|项目|设置|
|---|---|
|相机|固定上视，`camera=(0,0,0)`，`pitch=-90 deg`|
|控制|`accel_body_rate` + `mavlink_body_rate`，legacy body-rate|
|拦截高度|`50 m`|
|目标高度差|目标在拦截机上方 `30 m`|
|目标速度|`(0, 5, 0) m/s`，world/NED `+Y`|
|速度比|`2.0`，拦截速度上限 `10 m/s`|
|水平距离|`25/30/35/40/45/50 m`|
|横向偏置|`-10 m`|
|目标 actor|`Quadrotor1`，scale `1.5`|
|upward centering|开启，gain `8.0`，max accel `4.0 m/s^2`|

水平距离不是直接作为 world `dx`。程序使用 `forward=sqrt(range^2-lateral^2)`，再按 PX4 拦截机当前 yaw 的前向/右向轴放置目标。

## 3. baseline 结果

|算法|距离|结果|最近距离|
|---|---|---|---|
|TTC|25/30/35/40/45/50m|6/6 命中|1.32-1.88m|
|VM|35/40/45m|命中|1.51-1.62m|
|VM|25/30/50m|未命中|3.06m / 2.67m / 1.99m|

结论：当前 upward-camera YOLO+ByteTrack baseline 下，TTC 对真实 YOLO 检测误差和 ByteTrack ID 不稳定更鲁棒；VM 在 25m、30m、50m 终端段出现擦过后拉开。

## 4. 对应报告和日志

- 35/40m：`upward_final_yolo_35_40_20260628_173525`
- 25/30/45/50m：`upward_final_yolo_more_20260628_174009`
- 报告：
  - `完整方案/YOLO_ByteTrack_upward_final_35_40测试报告.md`
  - `完整方案/YOLO_ByteTrack_upward_final_more_25_30_45_50测试报告.md`
- 日志前缀：
  - `logs/yolo_sitl_ttc_vm/yolo_sitl_*_upward_final_yolo_35_40_20260628_173525_r*_h30.csv`
  - `logs/yolo_sitl_ttc_vm/yolo_sitl_*_upward_final_yolo_more_20260628_174009_r*_h30.csv`

## 5. 复现实验命令

运行前先检查端口和残留进程：

```bash
ss -ltnup | rg ':(41451|41452|4560|14540|14541|14550|1455[0-9])\b' || true
ps -eo pid,ppid,stat,etime,cmd | rg -i 'Blocks|PX4|sitl|run_yolo|run_upward|python.*run_airsim' || true
```

35/40m：

```bash
AIRSIM_RPC_HOST=127.0.0.2 \
AIRSIM_REWRITE_HOST_IPS=0 \
AIRSIM_PORT_POLICY=strict \
DETECTOR_SOURCE=yolo_bytetrack \
SHADOW_AIRSIM_DETECT=1 \
RANGES="35 40" \
RUN_TTC=1 RUN_VM=1 \
TERMINAL_BLIND_REQUIRES_VISUAL_LOSS=1 \
TERMINAL_CLIPPED_LOS_KF_PREDICT=1 \
REPORT_PATH="$PWD/完整方案/YOLO_ByteTrack_upward_final_35_40测试报告.md" \
ASSET_DIR="$PWD/完整方案/assets/YOLO_ByteTrack_upward_final_35_40测试报告" \
bash run_upward_body_rate_ttc_vm_smoke.sh
```

25/30/45/50m：

```bash
AIRSIM_RPC_HOST=127.0.0.2 \
AIRSIM_REWRITE_HOST_IPS=0 \
AIRSIM_PORT_POLICY=strict \
DETECTOR_SOURCE=yolo_bytetrack \
SHADOW_AIRSIM_DETECT=1 \
RANGES="25 30 45 50" \
RUN_TTC=1 RUN_VM=1 \
TERMINAL_BLIND_REQUIRES_VISUAL_LOSS=1 \
TERMINAL_CLIPPED_LOS_KF_PREDICT=1 \
REPORT_PATH="$PWD/完整方案/YOLO_ByteTrack_upward_final_more_25_30_45_50测试报告.md" \
ASSET_DIR="$PWD/完整方案/assets/YOLO_ByteTrack_upward_final_more_25_30_45_50测试报告" \
bash run_upward_body_rate_ttc_vm_smoke.sh
```

## 6. 使用规则

- 后续改动必须至少报告 TTC 和 VM 的 25/30/35/40/45/50m collision 命中率。
- VM 的 near-hit 不计成功；只有 AirSim collision 才算命中。
- 若更换检测源、关闭 fallback、修改 actor scale、初始几何或相机姿态，应作为新 baseline 或 ablation，不覆盖本 baseline。
