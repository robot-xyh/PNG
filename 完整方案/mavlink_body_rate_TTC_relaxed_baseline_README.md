# mavlink_body_rate TTC relaxed baseline 复现说明

本文档把当前真正机体系角速度链路的最好结果固定为 baseline：

- baseline 名称：`TTC_accel_body_rate_loskf_relaxed_20260623_073738`
- 控制链路：`guidance_output_mode=accel_body_rate` + `px4_command_mode=mavlink_body_rate`
- 导引律：`ttc_png`
- 视觉：`YOLOv8 + ByteTrack`
- 结果：6 个距离工况命中 4 个，命中 `50m, 80m, 90m, 100m`，未命中 `60m, 70m`

这份 README 的目标是让新机器从 AirSim/PX4 环境配置开始，复现同一套实验。后续 v2 优化应以这个 baseline 为对照，不应和 `velocity_yaw_rate` 速度链路混用结论。

## 1. 实验结果

baseline 日志：

```text
logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_accel_body_rate_loskf_relaxed_20260623_073738_summary.csv
```

逐距离结果：

|距离|是否命中|最小中心距离|终点距离|检测帧|有效帧|总帧|
|---:|---:|---:|---:|---:|---:|---:|
|50m|是|1.600m|1.630m|133|134|137|
|60m|否|2.317m|115.431m|138|141|272|
|70m|否|2.132m|13.641m|265|214|307|
|80m|是|1.610m|1.610m|144|159|163|
|90m|是|0.836m|0.985m|179|207|250|
|100m|是|1.545m|1.545m|164|197|221|

汇总：

|指标|数值|
|---|---:|
|命中数|4/6|
|最佳最小距离|0.836m|
|平均最小距离|1.674m|
|总检测帧率|75.8%|
|总有效导引帧率|77.9%|

## 2. 硬件与系统建议

已验证环境大致如下：

- OS：Ubuntu Linux
- GPU：NVIDIA GPU，CUDA 可用
- Python：3.10 到 3.12 均可，但建议使用独立虚拟环境
- AirSim：Blocks Linux 1.8.1
- PX4：`PX4-Autopilot v1.11.3`
- 仿真模式：AirSim Blocks + PX4 SITL + actor 目标

建议使用 NVIDIA 独显运行 Blocks。脚本 `run_blocks_nvidia.sh` 默认会设置：

```bash
__NV_PRIME_RENDER_OFFLOAD=1
__GLX_VENDOR_LIBRARY_NAME=nvidia
__VK_LAYER_NV_optimus=NVIDIA_only
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
```

如果机器没有这个 Vulkan ICD 路径，需要按本机环境修改 `VK_ICD_FILENAMES` 或直接设置环境变量。

## 3. 获取代码和权重

```bash
git clone git@github.com:robot-xyh/PNG.git
cd PNG
```

确认 YOLO 权重存在：

```bash
ls -lh vision_guidance/best.pt
```

如果没有该文件，需要把训练好的权重放到：

```text
vision_guidance/best.pt
```

baseline 使用 `yolo_class_id=0`，因此权重的第 0 类应为待拦截目标。

## 4. Python 环境

建议使用 venv：

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip setuptools wheel
```

安装基础依赖：

```bash
python3 -m pip install \
  numpy \
  airsim \
  pymavlink \
  matplotlib \
  python-docx \
  opencv-contrib-python \
  ultralytics \
  lap
```

安装 PyTorch CUDA 版。具体命令取决于本机 CUDA 版本，示例：

```bash
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
```

如果 CUDA 版本不同，请使用 PyTorch 官网给出的对应安装命令。安装后检查 GPU：

```bash
python3 - <<'PY'
import torch
print("torch:", torch.__version__)
print("cuda:", torch.cuda.is_available())
print("device:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "cpu")
PY
```

项目内当前机器曾使用过的主要包版本：

|包|版本|
|---|---|
|airsim|1.8.1|
|pymavlink|2.4.49|
|ultralytics|8.4.71|
|torch|2.11.0+cu128|
|opencv-contrib-python|4.13.0.92|
|lap|0.5.13|
|matplotlib|3.10.9|
|python-docx|1.2.0|

## 5. 安装 AirSim Blocks

把 Linux Blocks 放在默认路径：

```text
PNG/Blocks/LinuxBlocks1.8.1/LinuxNoEditor/Blocks.sh
```

如果 Blocks 在其他目录，运行前设置：

```bash
export BLOCKS_DIR=/path/to/LinuxNoEditor
```

确认可执行：

```bash
chmod +x Blocks/LinuxBlocks1.8.1/LinuxNoEditor/Blocks.sh
chmod +x Blocks/LinuxBlocks1.8.1/LinuxNoEditor/Blocks/Binaries/Linux/Blocks
```

baseline 使用的 AirSim settings：

```text
config/airsim_blocks_px4_actor_settings.json
```

关键字段：

|字段|值|
|---|---|
|`SimMode`|`Multirotor`|
|`ClockType`|`SteppableClock`|
|`ClockSpeed`|`1.0`|
|`ViewMode`|`NoDisplay`|
|`VehicleType`|`PX4Multirotor`|
|`UseTcp`|`true`|
|`LockStep`|`true`|
|`TcpPort`|`4560`|
|camera|`640x480`, FOV `120deg`|

`OpenXR` 报错通常不影响 NoDisplay/offscreen 运行；脚本默认追加 `-nohmd`。

## 6. 安装 PX4 SITL

建议按 AirSim 稳定参考版本安装 PX4 `v1.11.3`：

```bash
mkdir -p ~/PX4
cd ~/PX4
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.11.3
bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
make px4_sitl_default none_iris
```

本项目默认查找：

```text
~/PX4/PX4-Autopilot
```

如果 PX4 在其他目录：

```bash
export PX4_DIR=/path/to/PX4-Autopilot
```

启动脚本：

```bash
./run_px4_sitl.sh
```

该脚本默认执行：

```bash
make px4_sitl_default none_iris
```

如果遇到 `WorkQueueManager.cpp` 的 `math::max(PTHREAD_STACK_MIN, ...)` 编译错误，通常是较新编译器和 PX4 老版本类型不匹配，可在 PX4 源码中把该处两个参数显式转成同一类型，或使用较老 GCC 工具链。修完后重新：

```bash
make clean
make px4_sitl_default none_iris
```

## 7. 手动启动检查

开三个终端。

终端 1：启动 PX4。

```bash
cd /home/linux/Documents/PNG
source .venv/bin/activate
./run_px4_sitl.sh
```

等待 PX4 输出类似：

```text
Waiting for simulator to accept connection on TCP port 4560
```

终端 2：启动 Blocks。

```bash
cd /home/linux/Documents/PNG
source .venv/bin/activate
./run_blocks_px4_actor.sh
```

如果出现：

```text
TcpClientPort socket bind failed with error: 98
```

说明 4560 端口已被旧 Blocks/PX4 占用，先清理：

```bash
pkill -f 'Blocks/Binaries/Linux/Blocks|Blocks.sh' || true
pkill -f 'px4_sitl_default|PX4-Autopilot.*px4|px4-simulator|make px4_sitl_default' || true
```

终端 3：检查 AirSim 连接和车辆名。

```bash
python3 - <<'PY'
import airsim
c = airsim.MultirotorClient(timeout_value=5)
c.confirmConnection()
print(c.listVehicles())
PY
```

应看到 `Interceptor`。目标不是 AirSim multirotor，而是在脚本中动态生成的 actor：`IntruderActor`，asset 为 `Quadrotor1`。

## 8. baseline 关键算法配置

baseline 不是当前脚本默认值的简单运行。后续代码已引入 body-rate v2 和新的推力模型默认值，因此复现 baseline 时必须显式固定以下参数。

### 8.1 控制链路

|参数|值|
|---|---|
|`guidance_law`|`ttc_png`|
|`guidance_output_mode`|`accel_body_rate`|
|`px4_command_mode`|`mavlink_body_rate`|
|`body_rate_control_profile`|`legacy`|
|`max_guidance_accel_mps2`|`15.0`|
|`min_speed_ratio`|`0.6`|
|`body_rate_max_tilt_deg`|`20.0`|
|`body_rate_max_roll_rate_deg`|`60.0`|
|`body_rate_max_pitch_rate_deg`|`60.0`|
|`body_rate_attitude_p`|`4.0`|

### 8.2 baseline 推力参数

这批 4/6 baseline 使用旧 body-rate 推力参数：

|参数|值|
|---|---:|
|`body_rate_hover_thrust`|0.5|
|`body_rate_thrust_gain`|0.5|
|`body_rate_min_thrust`|0.25|
|`body_rate_max_thrust`|0.75|
|`body_rate_speed_hold_gain`|1.2|
|`body_rate_speed_hold_max_accel_mps2`|6.0|
|`body_rate_total_accel_limit_mps2`|18.0|

不要用当前 `airsim_generic_quad` 新默认值直接替代，否则结果会漂移。

### 8.3 视觉和 LOS

|参数|值|
|---|---|
|检测源|`yolo_bytetrack`|
|权重|`vision_guidance/best.pt`|
|YOLO device|`0`|
|YOLO conf / iou / imgsz|`0.1 / 0.7 / 640`|
|tracker|`bytetrack.yaml`|
|single target|开启|
|LOS KF|开启|
|`q_lambda / q_lambda_dot`|`5e-4 / 2e-2`|
|`r`|`8e-3`|
|`innovation_reject`|`0.75`|
|terminal image KF|开启|
|image KF predict|`0.35s`|
|image KF accel noise|`8.0 rad/s^2`|
|image KF reject|`0.20 rad`|

### 8.4 场景和相机

|参数|值|
|---|---|
|目标 actor|`IntruderActor`|
|actor asset|`Quadrotor1`|
|actor scale|`1.0`|
|拦截高度|`50m`|
|入侵目标高度差|`20m`|
|入侵目标速度|`5m/s`|
|速度倍率|`2.0`|
|初始横向偏置|`-20m`|
|相机外参|`x=0.5, y=0, z=0, pitch=0deg`|
|图像|`640x480`, FOV `120deg`|
|控制频率|`8Hz`|
|显示窗口 / 预览保存|关闭|
|AirSim detect 影子测试|关闭|

## 9. 一键复现 baseline

推荐使用批处理脚本。它会对每个距离自动重启 PX4 SITL 和 Blocks。

```bash
cd /home/linux/Documents/PNG
source .venv/bin/activate

STAMP=accel_body_rate_loskf_relaxed_reproduce_$(date +%Y%m%d_%H%M%S) \
RANGES="50 60 70 80 90 100" \
RUN_TTC=1 \
RUN_VM=0 \
GUIDANCE_OUTPUT_MODE=accel_body_rate \
PX4_COMMAND_MODE=mavlink_body_rate \
BODY_RATE_CONTROL_PROFILE=legacy \
BODY_RATE_HOVER_THRUST=0.5 \
BODY_RATE_THRUST_GAIN=0.5 \
BODY_RATE_MIN_THRUST=0.25 \
BODY_RATE_MAX_THRUST=0.75 \
BODY_RATE_MAX_TILT_DEG=20 \
BODY_RATE_MAX_ROLL_RATE_DEG=60 \
BODY_RATE_MAX_PITCH_RATE_DEG=60 \
BODY_RATE_ATTITUDE_P=4.0 \
BODY_RATE_SPEED_HOLD_GAIN=1.2 \
BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2=6.0 \
BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2=18.0 \
MAX_GUIDANCE_ACCEL_MPS2=15.0 \
MIN_SPEED_RATIO=0.6 \
LOS_FILTER=1 \
LOS_FILTER_PROCESS_LAMBDA=5e-4 \
LOS_FILTER_PROCESS_LAMBDA_DOT=2e-2 \
LOS_FILTER_MEASUREMENT_NOISE=8e-3 \
LOS_FILTER_INNOVATION_REJECT=0.75 \
TERMINAL_IMAGE_KF_MAX_PREDICT_S=0.35 \
TERMINAL_IMAGE_KF_ACCEL_NOISE_RAD_S2=8.0 \
TERMINAL_IMAGE_KF_INNOVATION_REJECT_RAD=0.20 \
TERMINAL_IMAGE_KF_MAX_RATE_RAD_S=8.0 \
DETECTOR_SOURCE=yolo_bytetrack \
YOLO_MODEL=vision_guidance/best.pt \
YOLO_DEVICE=0 \
YOLO_CONF=0.1 \
YOLO_IOU=0.7 \
YOLO_IMGSZ=640 \
INTRUDER_ACTOR_ASSET=Quadrotor1 \
INTRUDER_ACTOR_SCALE=1.0 \
CAMERA_X=0.5 \
CAMERA_Y=0 \
CAMERA_Z=0 \
CAMERA_PITCH_DEG=0 \
ALTITUDE_OFFSET=20 \
INTERCEPT_ALTITUDE_M=50 \
INTRUDER_SPEED=5 \
SPEED_RATIO=2 \
RATE_HZ=8 \
SHADOW_AIRSIM_DETECT=0 \
CASE_TIMEOUT_S=180 \
REPORT_PATH="$PWD/完整方案/mavlink_body_rate_TTC_relaxed_baseline_reproduce.md" \
ASSET_DIR="$PWD/完整方案/assets/mavlink_body_rate_TTC_relaxed_baseline_reproduce" \
REPORT_TITLE="mavlink_body_rate TTC relaxed baseline 复现实验报告" \
RANGE_NOTE="TTC relaxed baseline: accel_body_rate + mavlink_body_rate, 50-100m." \
./run_yolo_sitl_ttc_vm_batch.sh
```

输出位置：

```text
logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_<STAMP>_summary.csv
logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_<STAMP>_r50_h20.csv
logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_<STAMP>_r50_h20_meta.json
完整方案/mavlink_body_rate_TTC_relaxed_baseline_reproduce.md
完整方案/assets/mavlink_body_rate_TTC_relaxed_baseline_reproduce/
```

## 10. 单工况手动命令

如果只想跑 50m，可在已启动 PX4 和 Blocks 后运行：

```bash
python3 examples/run_airsim_strapdown_vision_png.py \
  --enable-motion \
  --duration-s 30.5 \
  --rate-hz 8 \
  --intruder-speed 5 \
  --speed-ratio 2 \
  --guidance-law ttc_png \
  --guidance-output-mode accel_body_rate \
  --px4-command-mode mavlink_body_rate \
  --max-guidance-accel-mps2 15 \
  --min-speed-ratio 0.6 \
  --body-rate-control-profile legacy \
  --body-rate-max-tilt-deg 20 \
  --body-rate-max-roll-rate-deg 60 \
  --body-rate-max-pitch-rate-deg 60 \
  --body-rate-attitude-p 4.0 \
  --body-rate-hover-thrust 0.5 \
  --body-rate-thrust-gain 0.5 \
  --body-rate-min-thrust 0.25 \
  --body-rate-max-thrust 0.75 \
  --body-rate-speed-hold-gain 1.2 \
  --body-rate-speed-hold-max-accel-mps2 6.0 \
  --body-rate-total-accel-limit-mps2 18.0 \
  --intercept-altitude-m 50 \
  --intruder-altitude-offset-m 20 \
  --start-horizontal-range-m 50 \
  --start-lateral-offset-m -20 \
  --settings-path "$PWD/config/airsim_blocks_px4_actor_settings.json" \
  --no-show-window \
  --no-record-preview \
  --preview-max-frames 0 \
  --no-plot \
  --print-every-n 0 \
  --reset \
  --px4-interceptor \
  --intruder Intruder \
  --intruder-actor \
  --intruder-actor-name IntruderActor \
  --intruder-actor-asset Quadrotor1 \
  --intruder-actor-scale 1.0 \
  --intruder-actor-respawn \
  --mesh IntruderActor \
  --detector-source yolo_bytetrack \
  --yolo-model vision_guidance/best.pt \
  --yolo-class-id 0 \
  --yolo-conf 0.1 \
  --yolo-iou 0.7 \
  --yolo-imgsz 640 \
  --yolo-device 0 \
  --yolo-tracker bytetrack.yaml \
  --yolo-allow-untracked-fallback \
  --yolo-single-target-mode \
  --yolo-single-target-max-center-jump-px 260 \
  --no-shadow-airsim-detect \
  --los-filter \
  --los-filter-process-lambda 5e-4 \
  --los-filter-process-lambda-dot 2e-2 \
  --los-filter-measurement-noise 8e-3 \
  --los-filter-innovation-reject 0.75 \
  --frame-guard \
  --frame-centering \
  --frame-centering-enter-error-ratio 0.52 \
  --frame-centering-terminal-error-ratio 0.68 \
  --frame-centering-area-ratio 0.006 \
  --frame-centering-loss-hold-s 0.8 \
  --frame-centering-speed-ratio 1.45 \
  --terminal-capture-speed-ratio 1.2 \
  --frame-centering-max-lateral-speed 1.2 \
  --terminal-capture-max-lateral-speed 0.55 \
  --frame-centering-lateral-scale 0.2 \
  --terminal-capture-lateral-scale 0.08 \
  --yaw-control \
  --yaw-error-gain 1.6 \
  --max-yaw-rate-deg 45 \
  --ttc-soft-guidance \
  --terminal-extrapolation \
  --terminal-image-kf \
  --terminal-image-kf-max-predict-s 0.35 \
  --terminal-image-kf-meas-noise-rad 0.006 \
  --terminal-image-kf-accel-noise-rad-s2 8.0 \
  --terminal-image-kf-innovation-reject-rad 0.20 \
  --terminal-image-kf-max-angle-rad 1.0 \
  --terminal-image-kf-max-rate-rad-s 8.0 \
  --terminal-image-kf-guidance \
  --no-terminal-image-kf-soft-reject-predict \
  --no-reject-top-clipped-pitch \
  --climb-timeout-s 90 \
  --no-px4-command-join \
  --min-command-duration-s 0.12 \
  --command-duration-margin-s 0.04 \
  --max-command-duration-s 0.25 \
  --camera-x 0.5 \
  --camera-y 0 \
  --camera-z 0 \
  --camera-pitch-deg 0 \
  --camera-roll-deg 0 \
  --camera-yaw-deg 0 \
  --trajectory-dir "$PWD/logs/yolo_sitl_ttc_vm" \
  --trajectory-prefix manual_bodyrate_ttc_relaxed_r50_h20
```

## 11. 复现后检查

生成 summary：

```bash
python3 examples/batch_strapdown_accuracy.py \
  --trajectory-dir logs/yolo_sitl_ttc_vm \
  --summarize-prefix yolo_sitl_TTC_<STAMP>
```

检查命中结果：

```bash
python3 - <<'PY'
import csv
from pathlib import Path
p = Path("logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_<STAMP>_summary.csv")
rows = list(csv.DictReader(open(p, encoding="utf-8")))
for r in rows:
    print(r["start_horizontal_range_m"], r["hit"], r["min_range_m"], r.get("detected_frames"), r.get("valid_frames"), r.get("frames"))
print("hits:", sum(str(r["hit"]).lower() == "true" for r in rows), "/", len(rows))
PY
```

确认 CSV 里是角速度链路：

```bash
python3 - <<'PY'
import csv
p = "logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_<STAMP>_r50_h20.csv"
r = next(csv.DictReader(open(p, encoding="utf-8")))
for k in [
    "guidance_output_mode",
    "px4_command_mode",
    "body_rate_control_active",
    "body_rate_control_profile",
    "body_rate_p_deg_s",
    "body_rate_q_deg_s",
    "body_rate_r_deg_s",
    "body_rate_thrust",
]:
    print(k, r.get(k))
PY
```

期望至少看到：

```text
guidance_output_mode accel_body_rate
px4_command_mode mavlink_body_rate
body_rate_control_active 1
body_rate_control_profile legacy
```

## 12. 常见问题

### 12.1 AirSim 报 OpenXR runtime 找不到

类似：

```text
Failed to find default runtime with RuntimeInterface::LoadRuntime()
Failed to enumerate extensions
```

NoDisplay/offscreen 模式下一般不影响本实验。脚本默认传入 `-nohmd`。

### 12.2 Vulkan device lost

如果 Blocks 启动时报：

```text
VK_ERROR_DEVICE_LOST
```

优先检查 NVIDIA 驱动、Vulkan ICD、是否同时开了多个 Blocks、是否有桌面/录屏占用 GPU。可尝试：

```bash
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
./run_blocks_px4_actor.sh
```

### 12.3 TCP 4560 端口占用

这是 PX4 SITL/AirSim 混合环境最常见问题。清理：

```bash
pkill -f 'Blocks/Binaries/Linux/Blocks|Blocks.sh' || true
pkill -f 'px4_sitl_default|PX4-Autopilot.*px4|px4-simulator|make px4_sitl_default' || true
```

### 12.4 YOLO 没有使用 GPU

日志 CSV 里有 `yolo_device_runtime`、`yolo_cuda_available`、`yolo_device_name` 等字段。也可以直接检查：

```bash
python3 - <<'PY'
from ultralytics import YOLO
import torch
print(torch.cuda.is_available())
print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "cpu")
m = YOLO("vision_guidance/best.pt")
print(m.device if hasattr(m, "device") else "model loaded")
PY
```

### 12.5 结果和 baseline 不完全一致

这是闭环仿真，以下因素会导致结果波动：

- GPU 推理 FPS 变化，baseline 约 8Hz 控制和 YOLO 推理闭环。
- PX4/Blocks 是否每个工况严格重启。
- 当前代码默认推力模型已经变更，必须显式设置旧推力参数。
- AirSim actor 初始状态是否完全 reset。
- NVIDIA 驱动、CPU 调度和窗口渲染负载。

因此复现目标不是逐帧相同，而是同一设置下达到相近命中率和最小距离分布。若结果大幅退化，先检查是否误跑成 `velocity_yaw_rate`、`accel_attitude` 或 body-rate v2。

## 13. 与 v2 优化的边界

这个 baseline 暂时作为后续 v2 的对照组。v2 优化可以改：

- `body_rate_control_profile=v2`
- quaternion error 到 `p/q/r` 的映射
- thrust projection / thrust reserve
- frame guard 下 PNG 和 speed-hold 的缩放
- 末端 LOS KF/terminal image KF 的门限

但对比时必须保留同样的：

- `YOLOv8 + ByteTrack`
- `Quadrotor1 actor`
- `camera_x=0.5m, pitch=0deg`
- `50/60/70/80/90/100m`
- `intruder_altitude_offset_m=20`
- `ClockSpeed=1.0`
- 每个工况重启 PX4 SITL 和 Blocks

只有这样，v2 的提升或退化才可归因于控制链路，而不是识别、场景或仿真状态差异。
