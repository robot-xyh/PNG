# Betaflight VM比例导引最终链路室内LOG_ONLY测试（2026-09-02）

## 1. 结论

本轮在拆桨、DISARM、RC7人工侧条件下，将生产RKNN YOLO、完整ByteTrack、LOS滤波、
`fixed_vm_png`和`accel_tilt_rate`候选映射放入同一个120 s运行链路。安全审计通过；全程
`LOG_ONLY`，`MSP_SET_RAW_RC=0`，没有向飞控发布控制。

静止20 s、连续水平往返、一次纵向往返、边缘丢失和重捕获均形成有效数据。最终配置下有效目标
结果年龄P50/P95/P99/max为`56.87/84.56/95.67/113.44 ms`，P95低于100 ms；姿态融合等待P95由
此前F03/F04的约100 ms降至`40.39 ms`。有效检测中ByteTrack confirmed为100%，LOS/导引有效率
为99.71%。无效导引行的非零Roll/Pitch候选数为0，导引有效行没有NaN或无限值。

因此，本轮通过VM比例导引实现所需的最终链路、连续水平符号、候选姿态/Rate映射、时延和
fail-closed检查。纵向仅形成一次清晰往返，不单独满足“三次重复压力”描述；结合此前
`R_BC/Pitch`相关系数`+0.969`的独立物理动作验证，已足够关闭实现层轴向检查，不要求为凑次数
重跑。本结果不证明动力闭环、真实空中视场保持或拦截命中能力，`release_passed=false`不变。

## 2. 配置与运行

Orange Pi为`orangepi5max`，地址`192.168.124.42`。运行参数为：

```text
duration                 120 s
main loop                50 Hz
perception               30 Hz
MSP attitude             20 Hz
MSP GPS/altitude         5/5 Hz（保留预期串口负载）
RKNN                      独立进程，CPU 4,5
main/MSP/web              CPU 6,7
guidance                  fixed_vm_png
N / Vm / acceleration    3.0 / 1.0 m/s / 1.0 m/s2
mapping                   accel_tilt_rate
tilt/rate limits          20 deg / 60 deg/s
control authorization     disabled
control mode              log_only
```

专用配置由`betaflight.rk3588.kinematics_log_only.example.json`派生，仅修改导引律和候选映射；
配置SHA256为`a1f70cb447014791bae7bb19f9a58006b0fdd69a4a5f2d4e85cef3e938076497`。

## 3. 安全与运行完整性

|指标|结果|
|---|---:|
|运行时长/CSV行数|119.990 s / 5963|
|停止原因|`duration_complete`|
|安全状态|100% `LOG_ONLY`|
|ARM最大值|0|
|MSP_SET_RAW_RC attempts/success|0 / 0|
|审计|通过，0 violations，0 warnings|
|MSP worker/request/checksum/parser错误|0 / 0 / 0 / 0|
|RKNN worker错误|0|
|有效导引行非有限数值|0|
|无效导引时非零Roll/Pitch候选|0|
|循环周期P95/max|20.21 / 83.08 ms|
|RKNN单帧最大耗时|17.303 ms|

`publish_deadline_miss_count=4`，约占5963个主循环行的0.067%，没有形成MSP错误或控制输出。

## 4. 感知、LOS与时延

全程产生3152个新感知结果，其中2089个含有效目标。该66.28%不能当作漏检率，因为测试明确包含
启动时无目标、边缘出框和完全移除目标。应按动作段统计：

|区间|时长|有效/新结果|confirmed|LOS有效|连续ID|用途|
|---|---:|---:|---:|---:|---|---|
|35--55 s|20 s|494/494，100%|100%|100%|2|静止基线|
|58.5--72.45 s|13.95 s|375/393，95.42%|100%|100%|2|连续水平往返|
|93.68--98.85 s|5.17 s|123/137，89.78%|100%|99.19%|32|一次纵向往返及交叉运动|
|106.63--119.97 s|13.34 s|334/360，92.78%|100%|99.70%|36|末段大范围水平运动|

连续水平段使用同一个`track_id=2`完成3次带`+/-0.12`滞回阈值的中心反转；全程共记录8次水平
反转。纵向段使用同一个`track_id=32`完成2次阈值反转，即一个完整往返。目标位置覆盖归一化
图像范围`x=-0.708..+0.714`、`y=-0.566..+0.139`。边缘/移除动作产生多个大于0.3 s的无检测
间隔，重新出现后允许建立新ID；连续水平和纵向段内部没有ID切换。

|时延指标|P50|P95|P99|max|
|---|---:|---:|---:|---:|
|有效目标结果年龄|56.87 ms|84.56 ms|95.67 ms|113.44 ms|
|姿态融合等待|20.06 ms|40.39 ms|60.19 ms|80.24 ms|

本结果说明50 Hz主循环、20 Hz姿态轮询和隔离RKNN进程可满足当前软件链路P95小于100 ms的门槛。
它仍使用`capture_return_monotonic`而非硬件曝光时刻，实际仿真和参数选择必须保留已测相机时间戳
不确定度，不把84.56 ms解释成曝光到执行器响应的完整延迟。

## 5. 比例导引与候选指令

有效导引共2083个新感知结果：

```text
lambda_dot_I,x = -0.3069 .. +0.2640 1/s
lambda_dot_I,y = -0.2826 .. +0.7069 1/s
Roll rate       = -24.714 .. +16.865 deg/s
Pitch rate      = -18.741 .. +21.018 deg/s
```

两轴均产生正负LOS角速度和正负候选Rate，最大值明显低于60 deg/s限制。导引加速度模长P50/P95/max
为`0.157/0.668/1.000 m/s2`，只有0.43%的有效行达到1.0 m/s2上限。映射关系为：

|关系|相关系数|预期|
|---|---:|---|
|`g_body_y -> desired_roll`|+0.99995|同号|
|`g_body_x -> desired_pitch`|-0.99998|反号|
|`g_body_y -> roll_rate`|+0.99995|同号|
|`g_body_x -> pitch_rate`|-0.99803|反号|

这证明生产链路把三维PNG加速度按当前FRD姿态约定映射为正确的Roll/Pitch候选，并保持限幅和
失效归零。机体在本轮没有响应候选指令，所以不能从该相关性推出闭环增益或稳定性。

## 6. 后续门槛

1. 不再重复本轮LOG_ONLY动态目标测试，也不为纵向次数单独增加人工测试。
2. 使用本轮实测时延、漏检间隔和LOS动态更新离线回放及Monte Carlo；当前候选命中率尚未通过。
3. 新控制器代码和参数通过离线测试后，仅补一次短时无桨MSP输出测试，验证实际RC发布、退出和
   目标丢失联锁。
4. 室外条件具备后，再做真实天空双机LOG_ONLY与低权限非碰撞闭环。本报告不批准直接拦截。

## 7. 证据索引

完整归档：

```text
logs/indoor_vm_final_chain/VM_FINAL_CHAIN_INDOOR_120S_20260902_214423.tar.gz
SHA256 aac68d2c28e46a597ee82ea65c14b576a2fa4ef8652379e2e4820596ab2af261
```

归档包含实际配置、CSV、事件JSONL、meta、控制台日志、安全审计和逐文件SHA256。机器可读结论见
`doc/evidence/BETAFLIGHT_VM_FINAL_CHAIN_INDOOR_20260902.json`。
