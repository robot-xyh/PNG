# Betaflight 完整飞行专家确认清单

## 用途与当前结论

本文用于评审 Orange Pi `/home/orangepi/src/circle_pilot` 的 `bf_flight_png` 完整飞行链路，
将责任明确分成两部分：**src/Betaflight runtime 自身必须解决的问题**，以及
**src 与 PNG 联合闭环必须共同确认的问题**。相机、RKNN、MSP只读和8080遥测已跑通，
但当前仍存在OVERRIDE首帧前四通道显示885 us、首帧后断流可能保持旧值、固定mode bit回退、Rate/油门未标定等
P0问题，当前production YAML不能作为飞行许可。

## 当前实机基线

|项目|已确认状态|
|---|---|
|飞控|MICOAIR743V2，STM32H743，Betaflight 2025.12.2 `norevision`，MSP API 1.47|
|RC|RadioMaster/EdgeTX + CRSF，`AETR1234`，16通道可读|
|飞手必要开关|RC5/AUX1已配置ARM；RC7/AUX3已配置Orange Pi接管，范围1700--2100 us|
|其他AUX|最新 `aux` 输出仅确认RC8/AUX4配置ANGLE；此前BEEPER配置不再作为当前事实|
|MSP|Orange Pi `/dev/ttyS1@115200`；STATUS/ATTITUDE/RC约97 Hz|
|关键异常|MSP OVERRIDE在第一帧有效MSP RC到达前激活时，R/P/Y/T同时变为885 us|
|相机|`/dev/video0`为视频节点，1280x1024 MJPEG输出640x512；`/dev/video1`是metadata|
|视觉|修改版RKNN模型可运行，推理约5--8 ms；真实目标精度尚未验收|
|网页|Python JSON/SSE/MJPEG可用；实测MJPEG与热负载会放大MSP线程调度间隙，控制时必须关闭|
|Python无桨输出|180个algorithm日志行已产生，包线和发送当刻门禁通过；最大发送间隔164.302 ms，未放行|

# 第一部分：src 必须独立解决

这一部分不讨论PNG制导效果。即使把PNG替换成固定零指令，src也必须先通过全部项目。
下列检查项多数是飞控、软件和故障恢复要求，不代表飞手还要增加更多遥控器开关。

## S1. MSP OVERRIDE首帧与断流行为

专家已确认当前固件在程序未启动时出现885 us属于首帧前现象：MSP通道缓冲区尚无有效帧，
OVERRIDE先激活时0会被限制为 `rx_min_usec=885`。较新的Betaflight上游代码已经出现300 ms
逐通道freshness保护，但本机是厂家 `norevision` 2025.12.2，不能据此推断其断流行为；第一帧
后停止发送、短帧和UART故障仍必须分别实测。

规定切换顺序：RC7保持人工侧，src先连续发送与当前物理RC一致的完整通道帧并通过发送频率、
回读值和watchdog检查，再允许RC7进入OVERRIDE；退出时先把RC7切回人工侧，再停止src。程序
不得在启动时自动改变RC7、ARM或OVERRIDE状态。

Python `noprop_bench` 已把正常顺序固化为代码门禁：900--2100 us主通道连续成功发送至少
10帧才置prefill ready；RC7启动时已打开、人工值未锁存或读到885时拒绝发送。该实现尚需
实机日志与Blackbox验收，不能替代本节的故障注入要求。

必须选择并验收一种架构：

- 可追溯Betaflight固件：MSP帧陈旧后自动回退物理RX；或
- 为MICOAIR743V2构建并审计对应固件；或
- 独立安全MCU/CRSF-SBUS硬件mux，Orange Pi掉电时默认连接物理接收机。

验收必须区分并覆盖：首帧前误开OVERRIDE、正常预填后切换、停止发送、`SIGKILL`、拔UART、
Orange Pi掉电、短MSP帧和部分通道未提供。特别注意 `rxMspFrameReceive()` 会把短帧中未提供的
通道清零，随后被限制为885 us。所有情况下R/P/Y/T都必须进入书面定义的安全值，不能产生
885 us极限指令或无限期保持危险的最后一帧。

## S2. ID 50、mode flag和mask必须fail-closed

src当前找不到permanent ID 50时会回退 `0x08000000`；已采集BOX列表中该bit对应
`LAUNCH CONTROL`。必须改成：BOXIDS/BOXNAMES查询失败、ID 50缺失、两者数量不一致、
实际mask不是15时直接拒绝控制。配置fallback必须为0，不能猜测bit。

飞控和src双方应明确记录：FC identity、BOX index、permanent ID 50、计算出的mode flag、
`msp_override_channels_mask=15`、`msp_override_failsafe=OFF`和配置快照SHA256。

## S3. RC所有权、ARM和人工接管

飞手界面目标采用最低两开关设计，摇杆之外只要求：

|RC通道|物理开关职责|OFF|ON|
|---|---|---|---|
|RC5/AUX1|ARM/DISARM|DISARM|允许ARM|
|RC7/AUX3|控制源选择|物理遥控优先|满足全部gate后Orange Pi接管R/P/Y/T|

2026-08-27当前CLI已确认 `aux 0 0 0 900 1300 0 0`（ARM）、
`aux 1 1 3 1700 2100 0 0`（ANGLE）和 `aux 2 42 2 1700 2100 0 0`
（旧记录）；当前固件重新导出为 `aux 2 50 2 1700 2100 0 0`
（MSP OVERRIDE/AUX3）。应以当前 `diff all`/`dump all` 中的 `50` 为准，不得手工改回
`42`。同时确认 `msp_override_channels_mask=15`、
`msp_override_failsafe=OFF`。程序启动和预填MSP帧不会改变该物理模式范围。

RC7关闭就是人工接管，不再要求第三个“人工接管”开关；紧急情况下RC5切到DISARM。PREARM或
独立急停可以作为专家建议的增强项，但不是当前两开关交互的必要条件。RC6 BEEPER和RC8
ANGLE可保留或由专家决定是否固定飞行模式，它们不参与Orange Pi控制源选择。

src只允许拥有mask bits 0--3，即roll/pitch/yaw/throttle。RC5、RC7以及其他AUX必须始终由
物理接收机提供；伴随计算机不得自动ARM、改变控制源或阻止物理DISARM。

两开关的状态定义必须固定：

|ARM|Orange Pi接管|系统状态|
|---|---|---|
|OFF|任意|DISARM，算法不得控制|
|ON|OFF|人工遥控R/P/Y/T|
|ON|ON且gate健康|Orange Pi控制R/P/Y/T|
|ON|ON但gate异常|按专家批准策略恢复人工RC或进入安全状态|

需要专家确认：

- RC5当前低位ARM是否改成更直观的高位有效；
- 两开关设计是否满足现场规范，是否确有必要增加PREARM或独立急停；
- RC7关闭是否在所有软件和固件状态下立即恢复人工R/P/Y/T；
- OVERRIDE切换前需要多少个连续、与物理RC一致的预填充MSP帧；
- 切换时每通道允许的最大跳变和油门交接时间；
- RF丢失、接收机重启和AUX HOLD时的最终状态。

## S4. RC协议、顺序、端点和发送调度

逐项确认EdgeTX CH1--CH8、CRSF、Betaflight `AETR1234`、MSP_RC逻辑RPYT和
MSP_SET_RAW_RC wire AETR。记录实际端点、中心、deadband、抖动、反向和failsafe；不能只看
3D预览。MSP发送前必须保留所有非mask通道，并记录read/sent 16通道。

确认单UART预算：SET_RAW_RC优先级、STATUS/ATTITUDE/RC/RAW_IMU/ANALOG轮询频率、最大RTT、
陈旧阈值和串口独占。Configurator、Python和src不得同时占用 `/dev/ttyS1`。

Python实测中，低负载180 s发送6944帧、最大间隔50.108 ms；带目标并打开MJPEG后，10414次
发送虽然错误为0，但最大publish/send间隔达到164.302 ms并触发physical RC stale重预填。
15 Hz感知、关闭MJPEG的233.52 s无目标基线降到49.769 ms。专家必须确认生产方案是独立
MSP进程/实时调度、降低感知与遥测预算，还是硬件mux；带目标、热稳态、生产遥测负载下最大
间隔连续满足60 ms前不得装桨。

## S5. src状态门禁和状态机

src当前主要读取STATUS、ATTITUDE和RC；Betaflight路径中的三轴角速度、电压、电流和电机
状态不完整。完整飞行至少需要：armed、OVERRIDE、physical RC fresh、MSP fresh、gyro/attitude
fresh、camera fresh、detection fresh、VBAT/cell/current/LQ、watchdog、target和物理DISARM门禁。

需要确认MSP RAW IMU或其他gyro来源及BMI270缩放，并和Blackbox交叉校验。任何门禁失败
都必须明确进入人工RC、受控保持或DISARM之一，不能默认发送中心杆量或上次指令。

## S6. 供电、进程和systemd

确认Orange Pi、飞控、相机和接收机的BEC容量、公共地、USB/5V反灌、上电顺序、brownout和
散热。伴随计算机重启不得自动恢复live control，必须重新经过物理AUX和完整门禁。

production service应从disabled开始，显式加载approved配置。专家需决定 `Restart=`：
进程崩溃后自动重启可能在OVERRIDE仍开启时二次接管。启动前检查配置哈希，退出时停止发送，
并验证进程结束后串口释放。

## S7. src日志与故障证据

至少记录：monotonic/UTC/boot ID、FC identity、mode flags、物理/发送16通道、armed、所有gate、
MSP RTT/age/error、gyro/姿态、VBAT/current/LQ、watchdog、进程信号和退出原因。同步保存
Blackbox，并用AUX边沿对齐。RTC/NTP和持久journal必须验收，控制逻辑只使用单调时钟。

## 只问src/飞控专家的问题

1. 哪个可追溯固件commit包含MSP逐通道freshness和物理RX回退？
2. 厂家固件是否与上游一致：首帧前为885 us、首帧后断流保持最后一帧、短帧缺失通道归零？
3. `msp_override_failsafe=OFF`只影响RXLOSS判定，还是也改变MSP样本选择和断流回退？
4. 人工优先应使用MSP OVERRIDE还是外部安全mux？其掉电默认态是什么？
5. RC5 ARM和RC7 OVERRIDE的两开关设计是否足够，RC5低位有效是否应改为高位？
6. 可用哪些MSP命令可靠获得gyro、VBAT/current和LQ，串口预算是多少？
7. 进程崩溃和自动重启时，允许的通道状态转换和恢复流程是什么？

# 第二部分：src + PNG 必须联合确认

这一部分要求飞控适配、动力、视觉和PNG在同一时间轴上闭环验证，不能由单模块测试替代。

## J1. Betaflight Rate反算和三轴响应

当前Rates为 `RC Rate=1.00, Expo=0, Super Rate=0.70`，中心斜率约200 deg/s，满杆约
667 deg/s；`rate_limit=1998 deg/s`不是实际满杆速度。src仍使用线性
`PWM=1500+500*rate/3.491`，会低估Super Rate放大。Expo=0时基线关系为：

```text
rate_deg_s = 200*x/(1-0.70*abs(x))
abs(x) = abs(rate_deg_s)/(200+0.70*abs(rate_deg_s))
```

正式实现必须复用对应固件公式或逐轴LUT，包含deadband、端点、斜率限制和饱和统计。通过
Blackbox建立 `PNG requested rate -> PWM -> gyro rate`，分别校准R/P/Y，并关闭src测试中
5.236 rad/s与活动配置3.491 rad/s的冲突。

## J2. 动力和油门

src存在冲突值：默认 `hover/strike=0.078/0.082`，速度profile为 `0.283/0.50`。必须提供
整机质量、电池、电机KV、电调协议、桨、idle/dynamic idle、推重比、静态推力、电流、电压
下陷和实际悬停PWM。先建立PWM到总推力曲线，再确认hover、最大油门、0.4 s handover和
低电压降级。任何现有候选值都不能直接批准。

## J3. 相机、时间戳和坐标系

确认640x512输出对应的内参、畸变、上视外参 `R_BC`、安装刚性、振动和R/P输出符号。当前
时间戳不等于可信曝光时刻；需使用V4L2单调时间戳，记录dequeue、预处理、推理、PNG和发送
时间，并用LED或机体动作与Blackbox gyro对齐。零拷贝当前失败，延迟必须按单槽回退路径统计。

## J4. RKNN检测与跟踪

固定模型SHA256、class、conf/iou、输入尺寸、NMS和跟踪参数。真实目标数据必须覆盖距离、
方位、背景、曝光、姿态和运动，报告precision/recall、连续漏检、ID switch、中心误差和帧龄。
当前bench没有受控目标，不能把少量valid帧当作精度验收。至少完成30 min NPU/相机热稳定。

## J5. PNG参数和安全包线

用真实Rate LUT和端到端延迟复核 `nav_ratio_x/y=2.5`、输出符号、最大rate、LOS微分滤波、
视觉外推、丢目标coast、倾角限制、边缘保护和终端逻辑。gyro不可用时不得声称完成
de-rotation。先验收视场保持和非碰撞近距通过，再扩大速度和终端包线。

## J6. 联合日志和分阶段放行

联合日志还需记录检测框/score/track ID、LOS/LOS rate、PNG各分量、请求rate/throttle、
反算PWM、限幅/斜率、最终RC和飞控实际gyro。按以下顺序放行：

1. 无桨RC passthrough：ID 50自动识别、无fallback、接管前后无跳变。
2. 无桨故障注入：首帧前、短帧、进程/UART/Orange Pi/RX断链均不产生885 us危险输出或无限保持旧指令。
3. 无桨电机验证：编号、方向、协议、idle、failsafe、Blackbox和电流计正确。
4. 无桨满载时序：15 Hz或批准后的生产感知率、真实目标、热稳态和生产遥测客户端下，发送
   最大间隔小于60 ms，且没有physical RC stale重预填。
5. 系留悬停：人工主控、算法仅小幅辅助，验证Rate LUT和油门交接。
6. 非碰撞移动目标：验证检测、延迟、视场保持和丢目标降级。
7. 扩大包线：每一级由CSV和Blackbox证据评审，不以“程序能启动”作为飞行许可。

## 联合评审直接提问

1. 是否应把Betaflight Rates改成线性profile，还是保留100/0/70并实现精确LUT？
2. 本机三轴指令到gyro响应的允许误差、延迟和饱和比例是多少？
3. 动力系统hover PWM、推重比、最大连续电流和安全油门包线是多少？
4. 相机曝光到gyro响应的偏移/抖动上限是多少，如何标定？
5. 真实目标的有效距离、允许连续漏检和跟踪切换指标是什么？
6. 首次带桨系留前需要哪些签字确认的无桨和故障注入证据？

# 专家批准后的真实完整src入口

## 必须由专家生成并签署的文件

不要直接使用当前 `strike_png_bf_flight.yaml`。批准后应建立独立、不可与bench混淆的文件：

```text
config/bf_flight_live_approved_common.yaml
config/strike_png_bf_live_approved.yaml
config/strike_png_bf_debug_live_approved.yaml
config/live_approved.sha256
```

最低要求：flight中 `dry_run=false`、`require_armed_to_command=true`；common中MSP设备和baud
正确、`override_mode_flag_auto=true`、fallback=0、mask=15；所有Rate/油门/电池/安全值已由
专家填写。debug必须指向approved flight。`live_approved.sha256`由专家在最终修改后生成，
评审记录应写明飞机、固件、配置hash、日期、测试包线和批准人。

## 可直接执行的受检启动命令

以下命令只有在上述文件存在且专家验收完成后才成立；当前执行会因缺文件而停止：

```bash
cd /home/orangepi/src/circle_pilot
set -euo pipefail

sha256sum -c config/live_approved.sha256

python3 - <<'PY'
from pathlib import Path
import yaml

root = Path.cwd()
flight_rel = Path("config/strike_png_bf_live_approved.yaml")
debug_rel = Path("config/strike_png_bf_debug_live_approved.yaml")
flight = yaml.safe_load((root / flight_rel).read_text())
debug = yaml.safe_load((root / debug_rel).read_text())
common_rel = Path(flight["bf_flight_config"])
common = yaml.safe_load((root / common_rel).read_text())

png = flight["strike_png"]
msp = common["bf_flight"]["msp"]
assert png["dry_run"] is False
assert png["require_armed_to_command"] is True
assert msp["device"] == "/dev/ttyS1"
assert int(msp["baud"]) == 115200
assert msp["override_mode_flag_auto"] is True
assert int(msp["override_mode_flag"]) == 0
assert int(msp["override_channels_mask"]) == 15
assert debug["bf_debug"]["strike_flight_config"] == str(flight_rel)
print("LIVE_CONFIG_PRECHECK_OK")
PY

if pgrep -x bf_flight_png >/dev/null || pgrep -x bf_debugd >/dev/null; then
  echo "Refusing: an existing BF process is running" >&2
  exit 1
fi
if fuser /dev/ttyS1 >/dev/null 2>&1; then
  echo "Refusing: /dev/ttyS1 is already in use" >&2
  exit 1
fi

./scripts/launch/launch_bf.sh \
  --png \
  --flight-config config/strike_png_bf_live_approved.yaml \
  --debug-config config/strike_png_bf_debug_live_approved.yaml
```

## 实际起飞顺序

该命令只启动完整src栈，不会也不应自动ARM或自动起飞。正确顺序是：RadioMaster在线、
RC7人工模式、RC5 DISARM；执行命令；先确认日志自动检测ID 50且无fallback、遥测和相机
正常；飞手通过RC5 ARM并用物理RC起飞、稳定悬停；最后才按批准包线打开RC7让Orange Pi
接管。任何异常先关闭RC7恢复人工R/P/Y/T，必要时再用RC5 DISARM。命令、配置hash和专家
批准不能替代现场飞手和测试场地安全。
