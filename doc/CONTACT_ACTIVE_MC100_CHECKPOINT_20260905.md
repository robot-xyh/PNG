# 接触主动配置与 MC100 断点（2026-09-05）

## 2026-09-06 schema v5 续跑

本节覆盖下文的历史断点状态。schema v5 配置与批准逻辑提交后，已重新绑定当前运行配置并
完成两套正式 MC100：

- 非碰撞配置 SHA256：`450917fea6359fb75c72276c9e21b4c40bc3a356bdc4f4f17869ae9ade74c12d`
- 接触配置 SHA256：`8ba8a4871eed7c328c9d223dbd8a3a7b6407c147221c39fa20f8fd35b6919def`
- 非碰撞结果：`logs/betaflight_intercept_schema5_20260906/noncollision_short_mc100.json`
  - `27,000` 行，`release_passed=true`
  - 三个场景及时 ABORT 率均为 `100%`，误接触率均为 `0`
  - 最小 ABORT 提前量为 `0.76 s`
  - SHA256：`256cfe0cf73b9036c935207033edc2dd5997b6971eae4316ba00b2d60ad0f359`
- 接触结果：`logs/betaflight_intercept_schema5_20260906/contact_short_mc100.json`
  - `18,000` 行，`release_passed=true`
  - 三个场景命中率为 `99.88%-99.96%`，FOV 命中率为 `99.77%-99.85%`
  - 三个场景目标陈旧失败率均为 `0`
  - SHA256：`75ae9de85550ef00a1e0522d858d415d60f1e7250167b63ccda8bfbbb81dfe9a`

批准器的 MC 覆盖门槛已按策略区分：非碰撞需要基线、接触性能和非碰撞安全三组评估，
最少 `27,000` 行；接触只需要基线和接触性能两组评估，最少 `18,000` 行。两者仍都要求
每工况至少 `100` 次、至少 `30` 个镜像工况及三个规定电压/延迟场景。

Betaflight Gazebo SIL 已在干净提交
`6bf11d70c5b136164ad30a6d028d385748746e52` 上完成四组最终验证。批准器的
`validate_sitl_evidence` 已分别接受两种策略的 projected/rendered 证据组合：

- 非碰撞 projected：
  `logs/betaflight_sitl/noncollision_projected_20260906_084752/betaflight_gazebo_sil_audit.json`
  - audit SHA256：`1b1007fe41dc30b1a741a222938ffc240d872949ed3cf60e1d5abf925344b9b2`
  - MSP `50.000 Hz`，最大写间隔 `21.624 ms`，零错误；由 TTC 触发 ABORT。
- 非碰撞 rendered：
  `logs/betaflight_sitl/noncollision_rendered_20260906_084932/betaflight_gazebo_sil_audit.json`
  - audit SHA256：`ca4e0e81329f06916c0f28f8905402cbbce879ca8b640e19cbbf2a2c4a7caf4e`
  - MSP `49.995 Hz`，最大写间隔 `22.676 ms`，零错误；由面积 TTC 触发 ABORT。
- 接触 projected：
  `logs/betaflight_sitl/contact_projected_20260906_084836/betaflight_gazebo_sil_audit.json`
  - audit SHA256：`a368d7913b70003413d47e86332d1958ab016aebb3a01eb0642365f512c149ee`
  - MSP `50.000 Hz`，最大写间隔 `21.902 ms`，零错误；进入 TERMINAL_VISUAL 后 COMPLETE。
- 接触 rendered：
  `logs/betaflight_sitl/contact_rendered_20260906_085203/betaflight_gazebo_sil_audit.json`
  - audit SHA256：`5435af1eb3344c492f93e37394277eb21465ba23454faab8df561c8530d9eecc`
  - MSP `50.000 Hz`，最大写间隔 `22.252 ms`，零错误；完整经过
    TRACKING、TERMINAL_VISUAL、BLIND_HOLD、COMPLETE。

四组 NED 真值校验、命令到 gyro 方向和电机差速方向均通过。SIL 场景按策略和检测模式
生成目标逼近参数，实际生成的模型文件已写入运行 manifest 并参与 SHA256 绑定。仍未完成
的真机项目是：当前提交的无桨主动 50 Hz 时序、当天飞控快照，以及分别生成非碰撞和接触
的 schema v5 批准文件。完整单元测试为 `549/549` 通过，`git diff --check` 通过。SIL
本身不构成真机控制批准。

## 当前状态

本轮工作曾因外场飞行准备暂停，随后已恢复并完成配置、单元测试和两套正式 MC100。改动仍仅存在于本机工作树，尚未提交、推送或部署到 RK3588：

- 新 LUT：`config/betaflight.thrust_lut.6s_22v_1500.physics_v1.json`
  - calibration ID：`6S_2412KG_3115_900KV_1050R_HIST15_PHYSICS_V1_20260905`
  - SHA256：`c89babfd2458626e4e26e3ec82a11ae05ae799490e9797b67c9a1e46dc6579be`
  - 电压范围：`22.0-25.2 V`
  - 油门范围：`1200-1500 us`
- 新接触主动候选：`config/betaflight.rk3588.velocity_png.flight_contact_supervised.json`
  - 独立 scope：`flight_contact_short_supervised_v2`
  - 接触终端策略已启用。
  - 使用独立批准文件路径，缺少批准时仍拒绝主动输出。
  - 每个 ARM 周期仅允许一次最长 `0.9 s` 的算法发布，并锁存至 DISARM。
  - 批准 schema v5 必须绑定“MSP 失效时由飞手降低 RC7”的配置哈希风险确认。
- 新 MC100 配置：`config/betaflight.intercept_eval.flight_contact_mc100_20260905.json`
  - 已绑定上述接触配置和 LUT。
  - 三个电压/延迟场景及镜像 30 个工况，每工况 100 次。
- 非碰撞 MC100 配置：`config/betaflight.intercept_eval.flight_supervised_final_mc100_20260903.json`
  - 绑定非碰撞运行配置 SHA256 `cf169b38940e050ac723aa9789fca08c6bc64d4e15c3be5979e0771bd767a93d`。
  - 三个电压/延迟场景及镜像 30 个工况，每工况 100 次。
- 活动配置禁止以 LOG_ONLY 启动；LOG_ONLY 只允许使用无桨专用配置和显式无桨确认。
- 运行时已增加接触 scope 的 LUT、油门、电压、终端参数、三帧采集和风险确认校验。
- MC runner 已增加接触专用性能判据，明确 MC 通过不等于主动飞行批准。
- 检测时序使用两个独立时钟：`detection_result_age_limit_s=0.20` 限制曝光到结果交付延迟，
  `detection_timeout_s` 限制最后一个新结果后的中断时长。非碰撞配置为 `0.15 s`，接触配置为 `0.25 s`。

## 已完成

1. 已补齐接触配置、运行时风险确认、独立待机/活动控制器和接触/非碰撞 MC 判据的单元测试。
2. 接触配置正式 MC100 共 `18,000` 次仿真，三个必选场景全部通过：
   - `final_chain_software_p95`：命中率 `99.923%`，FOV 命中率 `99.808%`，陈旧失败率 `0`。
   - `observed_active_flight_p95`：命中率 `99.962%`，FOV 命中率 `99.769%`，陈旧失败率 `0`。
   - `conservative_physical_p95_budget`：命中率 `99.885%`，FOV 命中率 `99.846%`，陈旧失败率 `0`。
   - 三个场景总加速度饱和均值为 `0.57%-0.69%`，速度建立项饱和均值为 `5.75%-6.10%`。
3. 非碰撞配置正式 MC100 共 `27,000` 次仿真，三个必选场景的专用安全判据全部通过：
   - 每个场景 `2,600/2,600` 个初始可见工况均及时进入 ABORT，及时中止率 `100%`。
   - 三个场景误接触率均为 `0`。
   - 最小 ABORT 提前量分别为 `0.86 s`、`0.80 s`、`0.76 s`，均高于 `0.75 s` 门槛。
   - 目标陈旧失败率分别为 `0.0385%`、`0`、`0`，均低于 `1%` 门槛。
4. 正式结果及绑定：
   - 接触：`logs/betaflight_intercept_short_supervised_20260905/contact_short_mc100.json`，
     SHA256 `8557fd66dea401d79470f9d803b6aa72daef522955650b98cf06ac4d83e0c1bb`。
   - 非碰撞：`logs/betaflight_intercept_short_supervised_20260905/noncollision_short_mc100.json`，
     SHA256 `05b8cd6e2f564c6b43d8a7868a7eb4bdcbd8c1a9ff8515884603fe32b197ed7`。
   - 两份结果均为 `release_passed=true`。
5. 历史结果见 `doc/BETAFLIGHT_CONTACT_MC100_20260905.md`，其配置哈希和 scope 已失效，不得作为当前批准证据。

## 尚未完成

1. 本机工作树完整测试已通过，但尚未提交并固化软件版本哈希。
2. 尚未用当前提交、当前配置和正式 MC100 结果完成最终无桨主动时序验证。
3. 尚未采集与最终软件及飞控配置匹配的新快照。
4. 尚未生成 `flight_contact_short_supervised_v2` 或 `flight_noncollision_short_supervised_v3` 的 schema v5 主动批准文件。
5. 尚未提交、推送或部署本轮候选。

## 本机验证

- `python3 -m unittest discover -s tests -v`：`505` 项通过。
- 修改及新增 JSON：`jq empty` 通过。
- 修改的 Python 文件：`python3 -m py_compile` 通过。
- 两份 MC 配置中的运行配置 SHA256 与当前文件一致，LUT SHA256 一致。
- `git diff --check` 通过。

## MC100 解释

非碰撞策略的通用命中率/FOV 命中率检查会显示失败，这是因为物理仿真在记录 ABORT 后
仍继续计算未由飞手接管的轨迹，而非碰撞策略本身要求提前退出而不是命中。该配置的发布
结论由 `policy_results.noncollision_safety` 决定，判据为及时 ABORT、误接触率和 ABORT
提前量；不能使用通用命中率字段否定或批准非碰撞策略。

接触配置的发布结论由 `policy_results.contact_performance` 决定，要求三个场景的命中率和
FOV 命中率均不低于 `80%`。当前正式 MC100 达到该仿真性能目标，但仿真场景仅供参考，
不能证明真实环境命中率，也不能替代无桨验证、飞控快照和哈希绑定批准。

## 外场隔离要求

在新 MC100、无桨主动时序证据、配置哈希绑定批准全部完成前，本断点中的接触配置不得用于带桨主动输出。短时监督实验仍依赖飞手主动降低 RC7；Orange Pi、进程或 UART 失效时没有独立自动回退。
