# Stage 00：论文与本地实现适用性核对

本报告只判断输入适配与实现能力。没有安装依赖、编译上游、运行优化或验证标签精度。数据证据见 [观测审计](00_observation_audit.md)。论文采用 `docs/papers/` 中的完整 PDF，文本提取保存在本地 run；原始 PDF、阅读的源文件、仓库提交均有 SHA。官方入口核对：[GPTR](https://github.com/brytsknguyen/gptr)、[gnssFGO](https://github.com/hz658832/gnssFGO)、[GPTR 论文](https://arxiv.org/abs/2410.22931)、[gnssFGO 论文](https://arxiv.org/abs/2309.11134)。结论依据本地固定版本，不把网页当前版本替代本地实现。

## 固定版本与阅读位置

| 对象 | 固定版本 | 直接相关内容 |
|---|---|---|
| GPTR | `2c0285a4f3141b131cc7b3aa77b3c6d979c0a2c2` | `gptr/include/GaussianProcess.hpp`；`factor/GPMotionPriorTwoKnotsFactor.h`；`factor/GPIMUFactor.h`；`src/GPUI.hpp`、`GPUI.cpp`、`GPVICalib.cpp`；依赖声明 |
| gnssFGO | `16030fb6a24263a4b927d2ec5d1da266340ddb9b` | `online_fgo/include/model/gp_interpolator/GPWNOJInterpolator{,Full}.h`；`factor/motion/GPWNOJPrior{,Full}.h`；`factor/gnss/{GPInterpolatedPVTFactor,PVTFactor,GPInterpolatedGPSFactor}.h`；`src/graph/GraphTimeCentric.cpp`；`src/integrator/GNSSLCIntegrator.cpp`；`src/gnss_fgo/GNSSFGOLocalizationBase.cpp` |
| GPTR PDF | IEEE TRO 2026，题名 A Third-Order Gaussian Process Trajectory Representation Framework With Closed-Form Kinematics for Continuous-Time Motion Estimation | IV-A，式 (23)–(25)；IV-B，式 (27)–(32)；V；VI-A，式 (44)–(45)；附录导数 |
| gnssFGO PDF | IEEE TRO 40 (2024)，GNSS/Multisensor Fusion Using Continuous-Time Factor Graph Optimization for Robust Localization | III 时间组织；IV-D WNOJ；V-A Remark 2；V-B IMU 与 GNSS-PVT 因子，式 (37)–(39)；GP prior 式 (45)–(47) |

## 方法要求与示例配置不能混为一谈

| 问题 | GPTR | gnssFGO |
|---|---|---|
| 轨迹表示 | SO(3)×R³ 或 SE(3)，三阶 WNOJ；`GPState` 显式保存 R、O、S、P、V、A，分别为旋转、角速度、角加速度、位置、速度、线加速度 | 连续时间 SE(3)，WNOA/WNOJ 等；论文常用 WNOJ 把测得的六维加速度作为输入，未将其全部设为优化变量；本地 **Full** 类另有显式六维加速度状态，不能说此框架没有加速度能力 |
| 最低观测种类 | 表示本身没有“必须有 IMU/UWB/LiDAR”条件；需要足够约束 gauge、偏置及所需导数的实际观测。已有 IMU 因子接受带时间的比力和角速度 | 框架可接多种异步观测；已发布 GNSS/IMU 定位流水线使用 IMU 预积分与传播。松耦合接 PVT；紧耦合另需每颗卫星的伪距、多普勒及相关卫星/时钟信息 |
| 必须记录连续增量吗 | 不要求记录 `vehicle_imu` 连续增量；直接在各测量时刻求 R、ω、a 的残差 | **不要求 ULog 自带增量字段**：`integrateMeasurement(accLin, gyro, dt)` 可从采样的比力/陀螺计算预积分。但是必须有可合理积分、校准并对齐的时间序列；不能把稀疏短积分伪装成完整积分 |
| 频率、异步和缺测 | 观测可在 knot 内任意时刻；核心用固定 knot 间距，不要求传感器同频。没有适用于所有场景的最低 Hz。缺测区间仅靠先验与端点连接，不会创造观测 | 时间中心节点与观测频率解耦，GNSSLC 按修正测量时间连接相邻状态。不同频率和缺测可处理；IMU 大缺口仍破坏正常预积分/传播假设，需分段或另建缺测模型 |
| 噪声与权重 | GP jerk 协方差控制先验；测量残差权重与偏置先验需提供。`GPUI.hpp` IMU 示例把 robust loss 关闭（阈值 -1），不能宣称示例已经自动排除异常 | GP Qc 与 GNSS 协方差/可配置噪声模型、IMU 偏置/噪声。GNSSLC 有多种 noise model 配置；示例的 fixed variance 和接收机专用解析不是本数据的已验证权重 |
| 线加速度 | R³：`A = dV/dt = d²P/dt²`；IMU 比力残差约束它。`GPIMUFactor` 用 `Rᵀ(A+g)`，移植到 NED 必须确认此处 g 的符号约定 | Full 插值可返回六维导数；须区分 SE(3) 的 twist 导数与惯性系线加速度，正确处理旋转项与重力，不能把六维尾部直接命名为 NED a |
| 角加速度 | `ComputeXtAndJacobiansSO3xR3` 使用 `ω=Jr(θ) θ̇`，`α=Jr(θ) θ̈ + D(Jr(θ)θ̇)/Dθ · θ̇`；有闭式 Jacobian，不是四元数分量二阶差分 | 论文常用实现输入测得加速度；`GNSSFGOLocalizationBase.cpp` 由相邻 gyro 差分生成 `accRot`。Full 插值确有 acceleration 函数，但 SE(3) 近似/Jacobian 设置和坐标转换需单独验证 |
| 当前直接读取 ULog | 没有本项目适配器；现成 UWB/视觉/LiDAR 示例不能原样读取这些日志 | 没有本项目适配器；ROS/NovAtel/u-blox 输入不等于 ULog。`NavSatFix` 示例会丢掉本日志已有的速度/航向信息，宜直接适配 PVT 数据结构 |
| 快速扑翼动态 | 显式 α/a 与闭式旋转导数更贴合目标；约 100 Hz 测量对实测扑翼基频有支持，但 knot 太疏、jerk 先验过强仍可削峰 | GP 原理并不排斥扑翼动态；车辆示例的节点间距、预积分、速度约束和测得角加速度处理不能照搬。Full 路线有潜力但额外验证较多 |

两种表示的“运动学一致”指同一轨迹及其导数关系一致，并不证明该轨迹是真实飞行。区间内 GP 条件均值可以严格按其解析公式求导；运动先验因子仍是有权重的随机约束。还需检查 knot 边界连续性、Lie-group 近似、离散数值积分误差和实现 Jacobian，不能只依据公式就给导出标签验收。

## 当前数据带来的硬限制与可适配项

- 主 IMU 是经过驱动坐标旋转、标度、短 FIFO 平均、日志抽取的约 100 Hz 观测；不是芯片全部原始采样。它可用于 GPTR 的直接测量残差，也可在明确带宽限制后用于 gnssFGO 数值预积分。两种方法都不能追回未记录的样本或超过可观测带宽的运动。
- `vehicle_imu` 约 2 Hz、约 5 ms 的增量，仅覆盖约 1% 时间；**直接依赖该流形成连续预积分的方案 BLOCKED**。这不是 gnssFGO 算法的普遍禁用条件；改用校准后的主 accel/gyro 是有条件的替代。
- 90/91 份有 GNSS PVT；缺 GNSS 的日志没有独立全局平移锚点。它仍保留惯性与融合状态观测价值，但不可声称与 RTK 航次同等绝对位置可信度。
- 无已验证的逐星伪距/多普勒/载波相位观测链，**gnssFGO 紧耦合示例不可直接运行**。RTK fixed 标签是接收机解算结果，不能替代原始卫星数据。
- 绝对航向的元数据不充分；`sensor_gnss_relative` 是不完整 CAN 投影，PVT 航向虽常有数值但含异常和不可信精度字段。现阶段不能当作已验证的双天线航向因子。
- 校准、传感器设备 ID、board→body、IMU/GNSS lever arm、GNSS 时延与精度字段语义必须显式适配。零 lever-arm 参数不是物理测量证据。GPS 与 PX4 融合状态不能被假定为独立观测。

## 优先级与复用边界

| 维度 | GPTR | gnssFGO |
|---|---|---|
| 观测适配性 | 原生直接 IMU 残差适合当前离散测量；需加 GNSS PVT 因子及单独陀螺/加速度时间适配 | GNSS 松耦合模块更现成；端到端流水线的连续惯性传播、ROS 接口适配更多 |
| 运动学导数能力 | **优先**：显式 R/ω/α 与 p/v/a，闭式 SO(3) 导数符合标签目标 | Full 实现可作为对照；不能用论文简化配置代替独立加速度估计；需核对 SE(3) 运动学转换 |
| 代码可复用性 | GP 核心和运动先验复用明确；Ceres/Sophus，示例还带 ROS/消息依赖，尚未构建 | PVT/坐标转换/时间连接逻辑有直接参考价值；GTSAM 适配版与 ROS 2 等依赖较重，尚未构建 |
| 科学可信度 | **CONDITIONAL，首选原型**；适配理由来自实测主 IMU/GNSS 与显式导数需求，没有测得的重建精度优势 | **CONDITIONAL，松耦合备选与 GNSS 因子参考**；不能宣称被数据“证伪”，也不能宣称 Full 路线已经验证 |

下一阶段应优先复用 **GPTR 固定版本的 `GaussianProcess.hpp`、`GPMotionPriorTwoKnotsFactor.h`、SO(3) manifold/Jacobian 和 `GPIMUFactor.h` 的观测模型**。用 `GPUI.hpp`/`GPVICalib.cpp` 学习建图与批处理结构，不搬 UWB、相机、LiDAR 数据假设。约束两个惯性流各自的时间；现有 IMU 因子把 accel/gyro 作为同一时刻输入，若不同步应在本仓库提供分开的测量适配，不能偷偷对齐为同一行。

GNSS 部分参考 **gnssFGO 的 `GNSSLCIntegrator.cpp`、`GPInterpolatedPVTFactor.h`/`PVTFactor.h` 与 GNSS 坐标转换工具**，实现本项目的 GNSS PVT 和 lever-arm 残差。先复用数学和接口设计，**不把 GTSAM 因子直接塞进 Ceres**。本地 `GPInterpolatedPVTFactor.h` 的解析路径包含速度 lever arm，而 `evaluateError_` 路径把这项注释掉；这是静态发现的路径差异，未执行数值验证，需在复用前用非零 lever arm 用例检查，不能直接信任所有路径等价。

许可证也不能只看仓库首页：GPTR 根 LICENSE 是 CC BY-NC-SA 4.0，但 `GPIMUFactor.h` 等文件保留 GPLv3+ 头；gnssFGO 根 LICENSE 是 BSD-3-Clause，部分核心头为 Apache-2.0。这里只记录已观察到的声明，未复制上游代码入 Git；将来分发/修改前应逐模块确认适用许可和保留声明。此次选择不以未经验证的构建成功、许可兼容或重建精度为前提。

## 下一阶段进入条件（尚未授权执行）

1. 固定一小组开发航次及原始 SHA/时间段；参数试验与保留评估数据隔离；不得按历史模型测试表现选段。
2. 明确惯性校准与坐标变换，按 device ID 取 bias/校准信息；明确 GNSS source、时间和单位；未知 lever arm 保留未知或有依据地做敏感性分析。
3. 建立非均匀时间戳下 p/v/a 与 R/ω/α 的合成导数/Jacobian/积分检查，另设观测留出与实测扑翼频段保留检查。
4. 小型适配验证后再评估完整优化的必要性；不要把 Stage 00 的可行性判断写成正式标签或物理真值证明。
