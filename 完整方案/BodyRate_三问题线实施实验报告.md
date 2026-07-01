# BodyRate 三问题线实施实验报告

- stamp: `body_rate_three_20260628_075242`
- trajectory_dir: `/home/linux/Documents/PNG/logs/body_rate_three_lines`
- cases: `41`

## 1. 总览结论

- `A_truth_actor` A truth+actor: collision `0/6`, geometric<1.5m `6/6`, LOS reject `0`, mean thrust saturation `2.2%`.
- `B0` B0 gimbal baseline: collision `2/6`, geometric<1.5m `0/6`, LOS reject `7`, mean thrust saturation `0.1%`.
- `B1` B1 gimbal no LOS filter: collision `4/6`, geometric<1.5m `0/6`, LOS reject `0`, mean thrust saturation `18.6%`.
- `B2` B2 gimbal yaw feedback: collision `5/6`, geometric<1.5m `0/6`, LOS reject `0`, mean thrust saturation `18.1%`.
- `C0` C0 strapdown AirSim detect: collision `6/6`, geometric<1.5m `6/6`, LOS reject `0`, mean thrust saturation `4.8%`.
- `C1` C1 strapdown YOLO no LOS filter: collision `0/5`, geometric<1.5m `0/5`, LOS reject `0`, mean thrust saturation `5.0%`.
- `C2` C2 strapdown YOLO relaxed LOS: collision `0/6`, geometric<1.5m `0/6`, LOS reject `6`, mean thrust saturation `2.8%`.

## 2. 汇总图

![hit_matrix](assets/BodyRate_三问题线实施实验报告/hit_matrix.png)

![min_range](assets/BodyRate_三问题线实施实验报告/min_range.png)

![gimbal_body_bearing](assets/BodyRate_三问题线实施实验报告/gimbal_body_bearing.png)

![cde_quality](assets/BodyRate_三问题线实施实验报告/cde_quality.png)

## 3. 实验明细

|实验|导引|距离m|collision|geom<1m|geom<1.5m|geom<2m|min m|final m|检测率|有效率|body-rate率|推力饱和|LOS拒绝|最近点前body bearing|最近点状态|CSV|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
|A truth+actor|TTC|50|0|1|1|1|0.06|6.89|100.0%|65.4%|100.0%|1.5%|0|||`body_rate_three_A_truth_actor_TTC_body_rate_three_20260628_075242_r50_h20.csv`|
|A truth+actor|TTC|80|0|1|1|1|0.17|0.66|100.0%|75.1%|100.0%|1.4%|0|||`body_rate_three_A_truth_actor_TTC_body_rate_three_20260628_075242_r80_h20.csv`|
|A truth+actor|TTC|100|0|1|1|1|0.22|5.70|100.0%|74.5%|100.0%|0.9%|0||not_closing|`body_rate_three_A_truth_actor_TTC_body_rate_three_20260628_075242_r100_h20.csv`|
|A truth+actor|VM|50|0|1|1|1|0.05|0.67|100.0%|72.1%|100.0%|6.4%|0||not_closing|`body_rate_three_A_truth_actor_VM_body_rate_three_20260628_075242_r50_h20.csv`|
|A truth+actor|VM|80|0|1|1|1|0.11|5.99|100.0%|68.2%|100.0%|2.4%|0||not_closing|`body_rate_three_A_truth_actor_VM_body_rate_three_20260628_075242_r80_h20.csv`|
|A truth+actor|VM|100|0|1|1|1|0.21|2.25|100.0%|70.6%|100.0%|0.9%|0||not_closing|`body_rate_three_A_truth_actor_VM_body_rate_three_20260628_075242_r100_h20.csv`|
|B0 gimbal baseline|TTC|70|0|0|0|0|2.55|159.15|31.6%|30.5%|100.0%|0.0%|1|67.9|BlindPush/bbox_area_large|`body_rate_three_B0_TTC_body_rate_three_20260628_075242_r70_h20.csv`|
|B0 gimbal baseline|TTC|90|0|0|0|0|2.45|177.88|32.8%|31.9%|100.0%|0.0%|1|67.8|BlindPush/bbox_area_large|`body_rate_three_B0_TTC_body_rate_three_20260628_075242_r90_h20.csv`|
|B0 gimbal baseline|TTC|100|0|0|0|0|2.56|175.74|34.8%|34.2%|100.0%|0.3%|1|59.8|BlindPush/bbox_area_large|`body_rate_three_B0_TTC_body_rate_three_20260628_075242_r100_h20.csv`|
|B0 gimbal baseline|VM|70|1|0|0|0|2.60|2.60|98.8%|95.2%|98.8%|0.0%|1|46.5|BlindPush/bbox_area_large|`body_rate_three_B0_VM_body_rate_three_20260628_075242_r70_h20.csv`|
|B0 gimbal baseline|VM|90|0|0|0|0|2.48|199.23|31.9%|31.3%|100.0%|0.3%|1|50.3|BlindPush/bbox_area_large|`body_rate_three_B0_VM_body_rate_three_20260628_075242_r90_h20.csv`|
|B0 gimbal baseline|VM|100|1|0|0|0|2.79|2.79|99.1%|97.4%|99.1%|0.0%|2|48.0|TerminalVisual/los_innovation_reject|`body_rate_three_B0_VM_body_rate_three_20260628_075242_r100_h20.csv`|
|B1 gimbal no LOS filter|TTC|70|1|0|0|0|2.63|2.63|98.8%|98.8%|98.8%|2.4%|0|53.0|BlindPush/bbox_top_clipped|`body_rate_three_B1_TTC_body_rate_three_20260628_075242_r70_h20.csv`|
|B1 gimbal no LOS filter|TTC|90|1|0|0|0|3.01|3.01|97.2%|97.2%|99.1%|3.8%|0|54.0|BlindPush/bbox_right_clipped|`body_rate_three_B1_TTC_body_rate_three_20260628_075242_r90_h20.csv`|
|B1 gimbal no LOS filter|TTC|100|0|0|0|0|2.52|277.52|34.5%|34.5%|100.0%|1.2%|0|63.7|BlindPush/bbox_top_clipped|`body_rate_three_B1_TTC_body_rate_three_20260628_075242_r100_h20.csv`|
|B1 gimbal no LOS filter|VM|70|1|0|0|0|2.67|2.67|98.9%|98.9%|98.9%|62.6%|0|46.7|BlindPush/bbox_area_large|`body_rate_three_B1_VM_body_rate_three_20260628_075242_r70_h20.csv`|
|B1 gimbal no LOS filter|VM|90|0|0|0|0|2.63|142.06|41.5%|41.5%|100.0%|38.1%|0|80.2|TerminalVisual/no_detection|`body_rate_three_B1_VM_body_rate_three_20260628_075242_r90_h20.csv`|
|B1 gimbal no LOS filter|VM|100|1|0|0|0|2.83|2.83|99.1%|99.1%|99.1%|3.5%|0|47.9|BlindPush/bbox_area_large|`body_rate_three_B1_VM_body_rate_three_20260628_075242_r100_h20.csv`|
|B2 gimbal yaw feedback|TTC|70|1|0|0|0|2.67|2.67|100.0%|100.0%|98.8%|3.6%|0|23.2|BlindPush/bbox_top_clipped|`body_rate_three_B2_TTC_body_rate_three_20260628_075242_r70_h20.csv`|
|B2 gimbal yaw feedback|TTC|90|1|0|0|0|2.61|2.61|95.4%|95.4%|99.1%|3.7%|0|25.5|BlindPush/bbox_top_clipped|`body_rate_three_B2_TTC_body_rate_three_20260628_075242_r90_h20.csv`|
|B2 gimbal yaw feedback|TTC|100|1|0|0|0|3.32|3.32|99.1%|99.1%|99.1%|45.3%|0|16.0|Tracking|`body_rate_three_B2_TTC_body_rate_three_20260628_075242_r100_h20.csv`|
|B2 gimbal yaw feedback|VM|70|1|0|0|0|2.71|2.71|100.0%|100.0%|98.8%|40.5%|0|20.0|BlindPush/bbox_area_large|`body_rate_three_B2_VM_body_rate_three_20260628_075242_r70_h20.csv`|
|B2 gimbal yaw feedback|VM|90|0|0|0|0|2.55|19.86|76.8%|76.8%|100.0%|5.9%|0|22.2|TerminalVisual/no_detection|`body_rate_three_B2_VM_body_rate_three_20260628_075242_r90_h20.csv`|
|B2 gimbal yaw feedback|VM|100|1|0|0|0|2.60|4.03|80.2%|80.2%|99.7%|9.4%|0|19.4|BlindPush/bbox_area_large|`body_rate_three_B2_VM_body_rate_three_20260628_075242_r100_h20.csv`|
|C0 strapdown AirSim detect|TTC|50|1|0|1|1|1.30|1.30|99.0%|100.0%|100.0%|5.2%|0|4.1|Tracking|`body_rate_three_C0_TTC_body_rate_three_20260628_075242_r50_h20.csv`|
|C0 strapdown AirSim detect|TTC|60|1|0|1|1|1.15|1.27|100.0%|100.0%|100.0%|4.7%|0|4.3|Tracking|`body_rate_three_C0_TTC_body_rate_three_20260628_075242_r60_h20.csv`|
|C0 strapdown AirSim detect|TTC|70|1|0|1|1|1.22|1.32|96.5%|100.0%|100.0%|4.4%|0|7.7|Tracking/bbox_area_jump|`body_rate_three_C0_TTC_body_rate_three_20260628_075242_r70_h20.csv`|
|C0 strapdown AirSim detect|TTC|80|1|0|1|1|1.38|1.38|98.5%|100.0%|100.0%|3.8%|0|3.6|Tracking|`body_rate_three_C0_TTC_body_rate_three_20260628_075242_r80_h20.csv`|
|C0 strapdown AirSim detect|TTC|90|1|0|1|1|1.47|1.47|98.2%|100.0%|100.0%|6.0%|0|4.1|Tracking|`body_rate_three_C0_TTC_body_rate_three_20260628_075242_r90_h20.csv`|
|C0 strapdown AirSim detect|TTC|100|1|0|1|1|1.43|1.43|96.6%|100.0%|100.0%|4.5%|0|2.0|Tracking|`body_rate_three_C0_TTC_body_rate_three_20260628_075242_r100_h20.csv`|
|C1 strapdown YOLO no LOS filter|TTC|50|0|0|0|0|34.07|136.24|14.1%|16.1%|100.0%|9.0%|0|103.9|Tracking/area_not_expanding|`body_rate_three_C1_TTC_body_rate_three_20260628_075242_r50_h20.csv`|
|C1 strapdown YOLO no LOS filter|TTC|60|0|0|0|0|48.26|213.54|10.3%|12.4%|100.0%|5.1%|0|123.7|LossHold/no_detection|`body_rate_three_C1_TTC_body_rate_three_20260628_075242_r60_h20.csv`|
|C1 strapdown YOLO no LOS filter|TTC|80|0|0|0|0|69.02|209.70|7.9%|9.9%|100.0%|4.8%|0|153.1|LossHold/no_detection|`body_rate_three_C1_TTC_body_rate_three_20260628_075242_r80_h20.csv`|
|C1 strapdown YOLO no LOS filter|TTC|90|0|0|0|0|78.10|291.64|6.9%|7.8%|100.0%|3.4%|0|143.2|LossHold/no_detection|`body_rate_three_C1_TTC_body_rate_three_20260628_075242_r90_h20.csv`|
|C1 strapdown YOLO no LOS filter|TTC|100|0|0|0|0|88.01|293.01|6.6%|7.2%|100.0%|2.7%|0|145.6|LossHold/no_detection|`body_rate_three_C1_TTC_body_rate_three_20260628_075242_r100_h20.csv`|
|C2 strapdown YOLO relaxed LOS|TTC|50|0|0|0|0|35.65|115.33|12.1%|10.1%|100.0%|4.8%|6|116.0|LossHold/no_detection|`body_rate_three_C2_TTC_body_rate_three_20260628_075242_r50_h20.csv`|
|C2 strapdown YOLO relaxed LOS|TTC|60|0|0|0|0|47.82|157.76|9.7%|10.6%|100.0%|3.4%|0|134.9|LossHold/no_detection|`body_rate_three_C2_TTC_body_rate_three_20260628_075242_r60_h20.csv`|
|C2 strapdown YOLO relaxed LOS|TTC|70|0|0|0|0|60.96|229.60|8.7%|9.5%|100.0%|1.9%|0|130.5|LossHold/no_detection|`body_rate_three_C2_TTC_body_rate_three_20260628_075242_r70_h20.csv`|
|C2 strapdown YOLO relaxed LOS|TTC|80|0|0|0|0|67.53|211.31|7.5%|8.2%|100.0%|2.4%|0|133.0|LossHold/no_detection|`body_rate_three_C2_TTC_body_rate_three_20260628_075242_r80_h20.csv`|
|C2 strapdown YOLO relaxed LOS|TTC|90|0|0|0|0|80.24|234.17|6.5%|7.2%|100.0%|1.9%|0|129.0|LossHold/no_detection|`body_rate_three_C2_TTC_body_rate_three_20260628_075242_r90_h20.csv`|
|C2 strapdown YOLO relaxed LOS|TTC|100|0|0|0|0|89.16|272.39|6.8%|7.4%|100.0%|2.2%|0|129.6|LossHold/no_detection|`body_rate_three_C2_TTC_body_rate_three_20260628_075242_r100_h20.csv`|

## 4. baseline 与 C0/C1/C2 对比分析

本节的 `baseline` 指历史 YOLO body-rate 基线 `TTC_accel_body_rate_loskf_relaxed_20260623_073738`，不是表中的 `B0 gimbal baseline`。该历史基线使用 `YOLOv8 + ByteTrack`、relaxed LOS KF、`guidance_output_mode=accel_body_rate`、`px4_command_mode=mavlink_body_rate`，50-100m 共 `4/6` 命中，命中 `50/80/90/100m`，未命中 `60/70m`。

|对象|识别/LOS|命中|平均最小距离|总检测率|总有效率|最近点前 body bearing|关键结论|
|---|---|---:|---:|---:|---:|---:|---|
|历史 YOLO body-rate baseline|YOLOv8 + ByteTrack，同步检测，relaxed LOS KF|4/6|1.67m|75.8%|77.9%|未统一统计|真实 YOLO 闭环可命中多数距离，但仍受检测断续和 LOS/KF 门控影响。|
|C0 strapdown AirSim detect|AirSim detect，关闭 LOS 滤波|6/6|1.33m|98.0%|100.0%|约 4.3deg|理论检测框连续时，固定相机 + body-rate 控制链路可以稳定拦截。|
|C1 strapdown YOLO no LOS filter|YOLOv8 + ByteTrack async，关闭 LOS 滤波|0/5|63.5m|8.6%|10.1%|约 133.9deg|关闭 LOS 滤波不能弥补目标早期出框/检测断续，主失败是 `no_detection`。|
|C2 strapdown YOLO relaxed LOS|YOLOv8 + ByteTrack async，relaxed LOS KF|0/6|63.6m|8.3%|8.6%|约 128.8deg|LOS 滤波放松后仍无效，因为有效视觉测量太少，导引长期处于 LossHold。|

### 4.1 C0 给出的内部上限

C0 与 C1/C2 使用同一条 `accel_body_rate + mavlink_body_rate` 控制链路、同一 actor 目标和同一固定相机安装参数：`camera_x=0.0m`、`camera_z=-0.5m`、`camera_pitch=0deg`。它只把检测源换成 AirSim detect，并关闭 LOS 滤波。结果 C0 在 `50/60/70/80/90/100m` 全部碰撞成功，最近点前 `target_body_bearing_deg` 基本压在 `2-8deg` 内。

这说明当前失败不能简单归因于 PNG 公式、PX4 `mavlink_body_rate` 指令发不出去，或固定相机方案天然不可行。在检测框连续、LOS 不被门限打断时，body-rate 链路能够把机体指向目标并完成碰撞。

### 4.2 C1/C2 为什么明显差于历史 baseline

C1/C2 虽然也使用 YOLOv8 + ByteTrack，但它们不是历史 baseline 的严格复现，至少有两个关键差异：

- 历史 baseline 使用同步 `yolo_bytetrack`；C1/C2 使用 `yolo_bytetrack_async`，并且 `async_detection_return_reused=false`、`async_detection_max_age_s=0.18`。这使日志中的有效检测帧显著减少，闭环更容易在末端进入无测量状态。
- 历史 baseline 的相机参数为 `camera_x=0.5m, camera_z=0.0m`；C1/C2 为 `camera_x=0.0m, camera_z=-0.5m`。该安装位置在本轮 AirSim detect 下没有问题，但对真实 YOLO 来说会改变目标在画面中的尺度、背景和边缘裁切时序。

结果上，历史 baseline 的总检测率为 `75.8%`，有效导引率为 `77.9%`；C1/C2 只有约 `8-10%`。C1 关闭 LOS 滤波后仍然 `0/5`，C2 打开 relaxed LOS 后仍然 `0/6`，说明本轮 C1/C2 的主矛盾不是 LOS KF 是否过严，而是 YOLO/ByteTrack 输入已经断续到导引无法维持。最近点前 body bearing 达到 `129-134deg`，也说明机体早已没有把目标保持在前向视场内。

### 4.3 LOS 滤波在本轮 C2 中没有发挥作用的原因

C2 的 relaxed LOS KF 只在有足够测量更新时才可能改善相位和抗抖。当前 C2 的有效率只有 `8.6%`，多数失败状态是 `LossHold/no_detection`。在这种条件下，KF 只能短时外推，不能凭空恢复目标；测量缺失时间超过外推窗口后，导引仍会失效。C2 的 `LOS拒绝` 总量只有 `6`，且主要集中在个别工况，因此它不是本轮 C2 失败的主因。

### 4.4 对后续实验的直接建议

后续要把历史 baseline 和 C0/C1/C2 的结论接起来，应先做严格复现实验：使用历史 baseline 的同步 `yolo_bytetrack`、`camera_x=0.5m, camera_z=0.0m`、relaxed LOS KF、相同 body-rate legacy 参数，再跑 `50-100m`。如果能恢复接近 `4/6`，说明 C1/C2 的退化主要来自 async 检测和相机外参变化；如果仍然退化，再检查当前代码默认参数和 PX4/Blocks 时序。

在严格复现之后，再逐项改动：

- 先固定同步 YOLO，单独比较 `camera_x=0.5,z=0.0` 与 `camera_x=0.0,z=-0.5`。
- 再比较同步 YOLO 与 async YOLO，并记录 measurement age、连续丢检长度和最近点前 body bearing。
- 最后再打开 relaxed LOS KF，验证它是否改善短时漏检，而不是在长期无检测时被误认为导引问题。

当前结论是：C0 证明 body-rate 控制链路本身可用；C1/C2 证明本轮 YOLO async 固定相机链路没有保持目标连续可见；历史 baseline 仍应作为真实 YOLO body-rate 的对照基线，直到严格复现实验完成。

## 5. 判读口径

- `collision` 仍然是 AirSim collision 判据。
- `geom<1m/1.5m/2m` 是独立几何评价，不改写 `hit`。
- B 线重点看最近点前 `target_body_bearing_deg` 是否压到 `10deg` 内。
- CDE 线重点看检测率、LOS reject 和推力饱和是否与未命中同步出现。
