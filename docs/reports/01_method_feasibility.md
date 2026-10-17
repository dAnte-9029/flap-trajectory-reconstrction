# Stage 01：GPTR 实现可行性与适配方案

日期：2026-10-17；分支：`reconstruction-audit`。**Stage 01 已完成：GPTR 固定版本的核心已构建，六种运动量、Jacobian、异步惯性残差及合成联合优化已实际验证；查询和参数化适配、一行 PSI 修正也已验证。GPTR 仍为条件优先，不能将原始公共 API 直接作为正式标签导出器。** 小角度导数近似、真实校准/时延/杆臂和结构振动仍有限制。

用户已明确授权 Stage 01，并选择创建 `flap-recon` Conda 环境；首次下载中断后，本轮恢复创建并连续完成构建和验证。未修改系统/全局 Conda 配置或上游 Fork，未读取原始 ULog、重算九月统计、运行真实航迹优化、生成正式标签或训练网络。没有进入 Stage 02。初始缺依赖的失败记录保留，不代表最终状态。

## 1. 证据范围和当前判断

已阅读并复用 [Stage 00 观测报告](00_observation_audit.md)、[方法报告](00_method_applicability.md) 和 `docs/STATUS.md`。本阶段的选段输入仅为 `runs/stage00_20261017/{audit_final,diagnostics_final}` 的现有 CSV；不读取模型性能。下列数值均为 Stage 00 结果的引用，不是新一轮原始统计：91 个独立 SHA、主 accel/gyro 约 100 Hz、GNSS PVT 90 航次约 5 Hz、稀疏惯性增量仅约 1% 时间覆盖、扑翼基频约 3.17–4.43 Hz（P5–P95）、主 IMU 不总是 EKF 选中的设备。

**结论：优先复用 GPTR 核心，加本仓库已验证的查询/参数化适配和显式 PSI 构建补丁。** 优势是直接比力/角速度残差、异步查询及显式 p/v/a、R/ω/α；本次验证提供了实现证据，但不是实际飞行精度证明。gnssFGO 保持 GNSS 松耦合设计参考，未编译其完整流水线或比较真实重建误差。

证据标记：**实际运行**表示本地命令执行结果；**源码事实**表示固定版本可直接确认的实现；**推导/方案**表示数学分析或拟采用设计；**未知**表示尚无测量或验证支持。

## 2. 固定版本、模块与构建边界

| 对象 | 固定版本与实际用途 |
|---|---|
| GPTR | `/home/zn/gptr`，`2c0285a4f3141b131cc7b3aa77b3c6d979c0a2c2`；本地 Fork `https://github.com/dAnte-9029/gptr.git`；原项目 `https://github.com/brytsknguyen/gptr`；外部源码只读，核心已由本仓库 target 编译/执行 |
| gnssFGO | `/home/zn/gnssFGO`，`16030fb6a24263a4b927d2ec5d1da266340ddb9b`；Fork `https://github.com/dAnte-9029/gnssFGO.git`；原项目 `https://github.com/hz658832/gnssFGO`；仅核对 GNSS 因子/时间/噪声设计 |
| PX4 | 使用 Stage 00 已匹配的固件快照；另用 `git show a644ca7…` 阅读传感器校准代码，没有使用当前 PX4 HEAD 代替日志版本 |

GPTR 没有已登记子模块；gnssFGO 的 LIOSAM、irt_gnss_preprocessing、irt_nav_common、novatel_oem7_utilities、ublox_msgs 均显示未初始化，具体 gitlink SHA 存于 Stage 01 manifest。未执行 submodule update。

| 实际复用或参考的文件/接口 | 作用 | 本次状态与限制 |
|---|---|---|
| `gptr/include/GaussianProcess.hpp`: `GPState`, `GPMixer::ComputeXtAndJacobiansSO3xR3` | SO(3)×R³ 状态及两端条件插值、各运动量和切空间 Jacobian | 原始核心与 build-local PSI 修正版均已编译/执行 |
| 同文件 `ComputeMotionPriorFactorSO3xR3`、`GPMotionPriorTwoKnotsFactor.h` | WNOJ jerk 先验与 Ceres 残差封装 | 原始实现已调用，数值 Jacobian 和合成优化已检验；协方差需正定 |
| `factor/GPIMUFactor.h` | 直接比力和 gyro 残差 | 原始因子已执行异步权重分离、重力/偏置及 Jacobian 检查 |
| `GPSO3LocalParameterization` | 右乘 SO(3) 更新 | 原有配套 Jacobian 是特殊约定；见下一节，不可直接与普通 AutoDiff 混用 |
| `GaussianProcess` knot 容器、`getKnot`, `setKnot`, `getGPMixerPtr` | 保存 knot 和提供接口 | 可复用；公共 `getStateAt` 不能未经修正直接作为标签导出入口 |
| `SE3JQ.hpp/.cpp` | 当前头文件和运行时分派依赖的 SE(3) 实现 | 最小 target 仍链接其 `.cpp`；本阶段只计划验证 SO(3)×R³，链接不等于已验证 SE(3) |
| `GPUI.hpp::AddIMUFactors`、`GPVICalib.cpp` | 建图、参数块和批处理参考 | 源码参考；不复制 UWB/视觉数据假设，不构建 ROS 应用 |

**实际构建环境：**Ubuntu 22.04.5，`/home/zn/anaconda3/envs/flap-recon`；Conda GCC 15.3.0、CMake 3.31.8、Eigen 3.4.0、Ceres 2.2.0 CPU build `cpugplhc142d66_210`、Sophus `1!1.24.6` build `he02047a_2`（源码版本 1.24.6）、glog 0.7.1、fmt 12.2.0；Python 3.11、NumPy 1.26.4、SciPy 1.11.4。90 个实际包及 URL/MD5 见 `environment_review/conda-explicit-md5.txt`，包级 SHA/版本也存于最终 manifest。没有安装 ROS/GTSAM/CUDA；CPU Ceres 包自动带入了其 SuiteSparse 等依赖。

上游 README 建议 Ubuntu 24.04、ROS Jazzy、SFUISE2/cf_msgs、Sophus 1.24.6、Ceres 2.2；`gptr/CMakeLists.txt` 还要求 ament、ROS 消息、PCL、OpenCV、OpenMP、fmt。这是**完整示例的构建要求**。数学核心需要 Eigen、Sophus、Ceres 及其传递依赖；不能把“必须有 ROS bag/cf_msgs”写成 GPTR 数学方法的输入条件。

不过核心并非已完全解耦：`GaussianProcess.hpp → SE3JQ.hpp → utility.h` 和两个因子头都会引入 ROS/PCL。仓库内 `experiments/stage01/utility_bridge.hpp` 显式使用上游现有 include guard 跳过 middleware utility，只补充其所需的稀疏块赋值工具；上游文件保持原字节。此构建适配**已编译执行**，不等于“原生 ROS 完整工程构建成功”。本仓库 CMake 直接引用外部 pinned 源码、拒绝不同 GPTR HEAD，不下载任何东西。启用 PSI 修正时只在忽略的构建目录生成有哈希校验的头文件副本。

实际构建使用 `-O1 -DNDEBUG`、Ninja、每个 build 最多 2 个编译任务。激活环境后的复现命令（新 run 路径及原始/修正版两套命令详见 `experiments/stage01/README.md`）：

```sh
cmake -S experiments/stage01 -B runs/stage01_replay/adapted -G Ninja \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG' -DSTAGE01_FIX_PSI_CUTOFF=ON
cmake --build runs/stage01_replay/adapted --parallel 2
ctest --test-dir runs/stage01_replay/adapted --output-on-failure
```

**结果：两个 target 均成功编译、运行；修正版 CTest 1/1 通过。** 原始版本记录 45 个 `PASS`、5 个 `KNOWN_ISSUE`、2 个 `APPROXIMATION_LIMIT` 和 21 个误差测量值；修正版记录 47 个 `PASS`、3 个原 API 问题复现、同样 2 个近似限制和 21 个误差测量值。保留原 API 的失败现象是诊断，不能将退出码 0 解释为原 API 已可直接导出。

早期失败均留存：`build_initial.log` 是系统环境缺 Eigen3；`configure_original.log` 是本仓库 target 仅启用 CXX 导致 SuiteSparse 找不到 OpenMP C component，已通过启用 C/CXX 修正；用户首次 Conda 下载出现 IncompleteRead，使用仅本进程的超时/重试/下载线程配置成功恢复，日志为 `environment_review/create_retry.log`。没有删掉全部缓存、替换系统库或静默更换 GPTR 固定版本。

许可证：GPTR 根 LICENSE 声明 CC BY-NC-SA 4.0，`GPIMUFactor.h`、`utility.h` 保留 GPLv3+ 头；gnssFGO 根 BSD-3-Clause，部分相关头 Apache-2.0。此处记录声明，不推定这些声明自动兼容。没有将上游实现复制进受版本控制的文件，也没有分发链接二进制；将来再分发前仍需逐模块厘清许可。

## 3. 运动学和实现风险

### 3.1 正确的目标关系

以固定局部 NED 为 N，目标刚体 FRD 为 B，R 将 B 向量转入 N：

\[
\dot p^N=v^N,\quad \dot v^N=a^N,\quad
\dot R=R[\omega^B]_\times,\quad \dot\omega^B=\alpha^B.
\]

这里 α 是角速度在随体坐标下分量的时间导数；角加速度向量换到 N 为 Rα，因为 ω×ω=0。`a^B=R^T a^N`，而 `d(R^T v^N)/dt=a^B−ω×v^B`。不能把比力或 body velocity 分量的导数直接称为惯性加速度。

源码的 SO(3)×R³ 路径使用 θ 的 GP 条件均值及 `ω=Jr(θ)θ̇`、`α=Jr(θ)θ̈+D(Jr(θ)θ̇)/Dθ·θ̇`。固定两端 p/v/a 的平移条件均值可由五次多项式验证；旋转先用固定轴多项式验证精确性，再用非交换 `Rx(a(t))Ry(b(t))` 区分插值误差与导数公式误差。4 Hz 合成运动用于测试与实测基频同量级的动态，**不是为真实观测选定优化参数**。

### 3.2 固定版本中已实际复现的发现与处置

1. `GaussianProcess.hpp:2466` 的右端 `Xb` 使用 `S[ua]`，不是 `S[ub]`。合成多项式例中公共查询与正确端点插值的 α 差 **0.03585 rad/s²**；同一批 knot 被优化和导出成不同轨迹。`getStateAt` 还在归一化 s 靠近端点时直接返回 knot，实测该例位置时间吸附误差 **7.57e−5 m**。`query_adapter.hpp::auditedQuery` 从 `getKnot` 取正确两端、检查边界并直接调用原插值器，已验证绕开两种查询层问题；非 cutoff 测点的六种运动量合并误差 1.37e−15，域外/NaN 查询拒绝检查通过。
2. `GPMixer::PSI` 在 `Dtau < DOUBLE_EPSILON`（0.001 秒）时返回零。每个 knot 后 1 ms 的右端条件影响被删除；在合成例中实测 cutoff 左右相差 2 ns 的 a 差 **0.0014695 m/s²**，左端后 0.125 ms 的 a 真值误差 **0.00018385 m/s²**。20 ms knot 下该分支占每段 5% 时间，50 ms 下占 2%；不是实际日志命中率。查询 wrapper 本身不能修复。现已在 build-local 副本中将唯一条件改为 `Dtau == 0.0`，保留精确端点快捷路径；不改变 GP 公式或时间单位。修正后上述差分别降为 **2.94e−9** 和 **1.27e−16 m/s²**，前者包含 2 ns 的真实变化。完整一行 diff 与副本哈希留存，上游 Fork 未改动。
3. `GPSO3LocalParameterization::Plus` 为 `q·Exp(δ)`，但 `PlusJacobian` 返回 `[I;0]`，`MinusJacobian` 返回 `[I,0]`。Sophus quaternion 存储为 xyzw 时真实 Plus 导数为 `0.5*[wI+[q_xyz]×;−q_xyzᵀ]`。原手写因子也把局部 Jacobian 填入四列的前三列，因此二者组合有意返回局部导数；不能把因子的四列解释成真实 ambient 导数，也不能把普通 AutoDiff 四元数因子直接与此 manifold 混接。

原 manifold 的 ambient 导数与数值差分实测最大元素差 **0.5087**；但原手写因子配合原 manifold 的**组合局部导数**检查通过，不能把二者混为同一个判定。本仓库 `AuditedSO3Manifold` 和 `AmbientFactorAdapter` 使用真实 Plus/Minus 导数，将原因子的局部 Jacobian 乘以 MinusJacobian 后交给 Ceres，只用于已确认此约定的手写因子。Plus/Minus 数值误差分别 **3.83e−10 / 2.96e−10**，因子转换后的组合 Jacobian 与直接局部检查一致。与普通 AutoDiff 合成状态因子混合的联合优化已通过。适配没有修改残差或 GP 求解器。

**另有保留的近似限制：**`lie_epsilon=1e−3` 下，`DJrUV_DU` 等函数的小角度分支使用低阶截断，并非全角度的机器精度导数。单独检查 `H=D(Jr(U)V)/DU`，在 `|U|=0.0005` 时最大元素误差 **9.45e−5**；跨 `|U|=0.001` 的 H 分支差约 **1.89e−4**。状态插值 Jacobian 的对应归一化误差最大 **8.74e−5**，满足本探针 5e−4 容差，但达不到 1e−6。两项记作 `APPROXIMATION_LIMIT`，没有伪装成精确等式。PSI 修正不改变此行为；严格全域可微性/更紧导数误差要求仍需检验更高阶稳定展开，不能把本阶段结果外推到所有分支、角度和运动。

上述结果支持**有条件的复用**，不支持原 API 无修改导出或全频段真值承诺。后续必须统一使用已测试的 query、manifold、因子转换和明确启用的 PSI 修正；小角度截断继续进入数值误差预算。当前只验证 SO(3)×R³，未验证 SE(3) 路径。

### 3.3 实际运行与未运行的数学检查

`python scripts/stage01_convention_checks.py` **实际运行 10 项，全部通过**，仅依赖已存在 NumPy/SciPy。它验证独立解析运动和拟采用的观测约定，**不调用 GPTR 或 gnssFGO**：

| 检查 | 误差量级（范数） |
|---|---:|
| p 的差分与 v；v 的差分与 a | 7.74e−9；1.40e−7 |
| 非交换 R 的差分与 body ω；ω 的差分与 α | 2.10e−8；4.17e−7 |
| 非零天线杆臂的位置导数与预测速度 | 8.05e−9 m/s |
| 刚体点二阶差分与含切向/向心项的加速度 | 2.99e−7 m/s² |
| 比力/重力/零偏正反换算 | 6.40e−15 m/s² |
| 静止 NED 比力、PVT 协方差白化、速度方差单位 | 0（给定合成值下） |

同一合成例中，忽略非零杆臂速度项会差 0.1594 m/s；将相差 2.05 ms 的 gyro 强行用同一时间解释会差 0.08077 rad/s。它们只是构造例证，**不是本飞机的实际杆臂或误差估计**。

`experiments/stage01/probe.cpp` 已实际编译并执行，包含：

- 五次平移/固定轴旋转的六种运动量真值比较；非交换 4 Hz 运动在 100/50/20 ms knot 下的插值误差测量。
- 时域 `ṗ=v, v̇=a, Ṙ=Rω̂, ω̇=α` 检查；两端 36 个局部参数到 18 维查询状态的中心差分 Jacobian，包括零/小相对旋转及两种扰动尺度。
- 原始 `GPIMUFactor` 和 motion prior 的组合局部 Jacobian、部分空 Jacobian 指针、静止 NED、非零 bias、分开加速度/gyro 时间检查。
- 特殊 manifold 与真实 ambient 导数的差异；适配 manifold 的 Plus/Minus、因子转换检查；公共查询错误及 PSI 时间分支的显式回归探针。
- 固定解析 trajectory 的 Ceres bias-only 求解，以及两个 knot 的所有六种状态联合优化（使用解析状态锚点、原 motion prior、IMU 因子、实际 manifold 与 AutoDiff）；后者状态合并误差 **4.75e−13**，bias-only 误差 **8.88e−10**。这是已知无噪声可解问题的验证，不是从真实日志推得的可观测性或任意初值收敛保证。
- 相邻 knot 左右六种运动量检查，以及独立中点积分检查。积分步数 50/100/200 的合并误差约 **4.456e−5 / 1.114e−5 / 2.785e−6**，步长减半误差比约 0.25；它验证积分收敛，而非用积分误差修改测量运动。

五次多项式用例中，p/v/a 误差最大 **1.96e−14**，R/ω/α 最大 **6.84e−15**（各自物理单位）；非交换周期运动在测试的区间内部，运动学导数关系差最大约 **1.17e−8**。IMU、运动先验组合局部 Jacobian 归一化误差分别 **5.83e−9 / 6.05e−8**。这里 Jacobian 指标为逐元素 `|J_numeric−J_analytic|/(1+|J_numeric|)`，旋转扰动为右乘指数、弧度制，不能当作 ambient 四元数导数误差。

解析 4 Hz 非交换旋转/平移轨迹的**插值近似误差**如下。两端状态使用解析真值，无测量噪声，也没有优化平滑；每个 dt 检查 5 个连续区间，每区间取 0.1/0.37/0.8 三个相对时刻，因此是有限样点的最大值，不是全域上界：

| knot dt | p m | v m/s | a m/s² | R rad | ω rad/s | α rad/s² |
|---|---:|---:|---:|---:|---:|---:|
| 100 ms | 1.27e−4 | 4.66e−3 | 0.2939 | 3.59e−4 | 0.01340 | 0.8256 |
| 50 ms | 2.12e−6 | 1.55e−4 | 0.01977 | 6.54e−6 | 4.82e−4 | 0.06089 |
| 20 ms | 8.84e−9 | 1.61e−6 | 0.000517 | 2.84e−8 | 5.17e−6 | 0.001666 |

这为 Stage 02 优先比较 20/50 ms 提供合成依据，不能当作真实扑翼数据上的精度承诺，也不能据此选定未知噪声和 jerk 权重。合成试验用单位 jerk 协方差；平移/旋转连续白 jerk 谱密度的单位分别是 m²/s⁵、rad²/s⁵，实际日志权重未拟合。`KNOWN_ISSUE`、`APPROXIMATION_LIMIT` 与 `PASS` 分开保留；误差容差是本有限数值检查的标准，不是物理真值验收标准。

## 4. GPIMUFactor 的物理含义与异步适配

源码残差（12 维，头部“6-element”的注释不准确）为：

\[
r_f=w_f\{R^T(a^N+g_{param})-f_m^B+b_a\},\quad
r_\omega=w_\omega(\omega^B-\omega_m^B+b_g),
\]
\[
r_{bg}=w_{bg}(b_g-b_{g0}),\quad r_{ba}=w_{ba}(b_a-b_{a0}).
\]

因此传入原始加速度必须是与轨迹 body frame 同轴、校准后的**比力**。若物理重力 `g^N=(0,0,+g0)`，此因子需 `g_param=−g^N`。静止、R=I、a=0 时无偏比力为 `(0,0,−g0)`。重力幅值 9.80665 在合成检查中是已知常量；真实场地重力参数仍需注明假设，不能把未建模姿态误差都交给自由重力和 bias 吸收。

测量模型为 `f_m=f_true+b_a+noise`、`ω_m=ω_true+b_g+noise`，所以残差里加 bias；这与“预先减掉 bias”是不同接口。若输入已减偏，应固定该项为零或仅估计剩余偏置，不能重复减。权重是残差乘数，独立高斯轴向标准差 σ 对应 1/σ，不是 1/σ²；原始因子仅支持每类标量权重，完整各轴协方差需在薄因子中白化。

**从实际日志到该模型的必要路径：**

1. 读取 `sensor_accel[0]`、`sensor_gyro[0]` 各自的 device ID、sample/publication 时间、参数变更与校准计数；不以 instance 相同推出两个测量已有同一时刻。
2. 匹配固件的 `calibration::Accelerometer::Correct` 为 `R_cal * ((data−thermal_offset−offset) ⊙ scale)`；内部传感器遵循 board rotation，并有 board level adjustment。按 `CAL_*_ID` 关联校准，明确驱动已施加的安装旋转，避免再次重复。`SENS_BOARD_ROT=8` 的整数本身不是可跳过的物理外参说明。
3. 本阶段没有恢复每个样本的 thermal correction 或验证实际安装外参。Stage 02 需检查所选小段元数据和 `sensor_correction` 链，缺失项标 unknown；不能声称目前已经有可直接送入因子的 body 比力。
4. EKF bias 必须匹配传感器 ID 和有效时间。B 段特意选主 IMU 未被 EKF 选中的例子：此时不能拿选中传感器的 bias 当主 IMU bias。可用已知校准后剩余常 bias 的短段模型，独立初始化/先验并做敏感性分析，不拟合随翼相位变化的“bias”把真实动态吸掉。

**异步方案（合成残差与 Jacobian 已验证；真实日志适配未运行）：**保持两个原生时间序列，分别在 `t_acc` 和 `t_gyro` 找 knot pair。先最少改动复用原 `GPIMUFactor` 两次：加速度时刻令 `wGyro=0`，gyro 时刻令 `wAcce=0`，两次均令两个 bias-prior 权重为零，未使用的测量填有限零。bias prior 每个 bias 状态单独添加一次。合成检查的两个时间相差 2.05 ms，残差约为机器零、Jacobian 检查通过。其代价是保留一些零行、无用参数和插值开销；后续可封装成独立三维因子，数学仍调用原 GPMixer。不要 nearest-neighbor 拼一行，不用 2 Hz、5 ms delta 流做连续预积分。

原 `GPUI::AddIMUFactors` 每条 IMU 记录重复加 bias 先验，若直接照搬，累计约束强度会随记录数变化；权重不能原样当本数据的独立先验。其 robust loss 示例关闭，不代表真实 IMU 异常已经自动处理。日志约 2.5 ms 平均窗口与约 10 ms 记录间隔不同；实际时刻/平均窗口的误差以及未记录采样的混叠仍需保留，异步插值不解决硬件延迟。

## 5. GNSS PVT 适配方案

主入口为 `vehicle_gps_position[0]`。Stage 00 已验证它与 `sensor_gps` 同接收机、同 epoch、相同 PVT，是选择后的接收机解，不是 EKF global state；同 epoch 重复应去重，不能将两 topic 或 `vehicle_global_position` 作为独立因子叠加。

| 事项 | 拟采用方案与尚未知信息 |
|---|---|
| 位置 | `latitude_deg/longitude_deg` 单位度；优先 `altitude_ellipsoid_m` 做 WGS84 LLA→ECEF→固定原点 NED。`altitude_msl_m` 为另一高度基准，不能混用；若椭球高无效，先解决基准或只用可验证分量。原点固定于所选段有效 PVT，并记录其源 epoch，不跟随 EKF origin reset |
| 速度 | `vel_n_m_s/vel_e_m_s/vel_d_m_s` 是接收机当地 NED 速度；若严格使用固定原点 NED，先从测点 NED 旋到 ECEF，再旋到原点 NED；短段近似相等也需量化，而非悄悄假定 |
| 时间 | 保留 HRT publication/sample、GNSS UTC、源 device。当前 sample=publication、原 sensor sample=0，只能采用明确标注的接收时间代理；UTC 去重不能自动给出 HRT 硬件映射。以 `t_meas=t_receive−delay` 显式建模，正 delay 表示接收滞后；真实 delay 未知，先做预注册敏感性，不把 0 当测得结果 |
| 位置协方差 | 日志只保留 `eph=sqrt(max(Cnn,Cee))`、`epv=sqrt(Cdd)`。候选 `diag(eph²,eph²,epv²)` 是丢弃相关项后的代理，不是真实完整协方差，亦不保证 PSD 意义上的保守上界；随坐标变换旋转它 |
| 速度协方差 | 匹配 CAN 驱动直接把最大速度对角**方差**写入 `s_variance_m_s`。候选 `s_variance_m_s * I`，绝不能再平方或按通用字段注释当 σ；同样只是代理 |
| 有效性/噪声 | finite、位置范围、fix、velocity-valid、时间递增与 epoch 唯一分别检查。非正精度表示不可用，不能作零方差强约束；可拒绝相应残差或启用明确标注的预设 fallback。固定/浮点/普通定位的权重政策在开发段冻结，receiver precision 不等于真实误差；保留拒绝原因。GNSS yaw 不启用 |
| 融合状态 | `vehicle_attitude/local_position` 可做初始化或明确相关性的软约束/对照，不能与原始 IMU/PVT一起当独立真值验收 |

目标点 O 到天线 A 的 body 固定杆臂 ℓ，预测：

\[
p_A^N=p_O^N+R\ell,\qquad v_A^N=v_O^N+R(\omega^B\times\ell).
\]

新建 **Ceres** PVT 残差，对六维 `[p_A−p_GNSS, v_A−v_GNSS]` 白化。通过原 GP 查询的 p/v/R/ω 及其 Jacobian 链式连接 knot；若采用真正 manifold，须使用对应 ambient Jacobian。固定杆臂不需要额外运动学求解器。`EKF2_GPS_POS_*`、`EKF2_IMU_POS_*` 为零不证明实际共点，暂不能宣称已有物理正确的天线→质心转换。

参考 gnssFGO 的 `GNSSLCIntegrator.cpp`：`timestamp−delay`、quality/noise-model/onlyRTKFixed 选项；参考 `PVTFactor.h` 和 `GPInterpolatedPVTFactor.h` 的天线位置与速度杆臂设计、`GNSSUtils` 坐标变换。**不链接 GTSAM 因子到 Ceres，不照搬车辆场景的非完整速度约束。** 已复核 `GPInterpolatedPVTFactor::evaluateError_` 注释掉速度杆臂项，而另一求值路径包含该项；此差异尚未运行该二进制验证。独立合成杆臂检查已显示遗漏该项可以产生非零误差，但不等于对 gnssFGO 的数值复现。

## 6. 扑翼振动、参考点和可辨识性

刚性固定安装、IMU 到参考点的 body 杆臂 r 已知时：

\[
a_I^N=a_O^N+R\{\alpha\times r+\omega\times(\omega\times r)\},\quad
f_I^B=R^T(a_I^N-g^N).
\]

若安装柔性使 r 随时间变化，还增加 `R{2ω×ṙ+r̈}`；局部安装姿态变化也会污染 gyro 对整机 body 角速度的解释。扑翼机构施力可引起真实刚体周期加速度，也可产生机架弯曲、安装振动和局部旋转。与翼相位高度相干只能证明周期结构，不能把这几项分离。

建议 Stage 02 首先把重建参考点定义在**主 IMU 安装点**，明确假设该处与 FRD 机体系刚性相连。此举减少未知 IMU 平移杆臂，不会自动证明安装刚性，也不会把结果变成整机质心轨迹。扑翼改变瞬时质量分布时，“整机质心”相对机架也可能不是固定点；静态天线/IMU 坐标和质心假设均需实物证据。

| 运动量 | 当前有观测支持的内容 | 未知或主要靠先验的内容 |
|---|---|---|
| p/v | 优质 RTK 段的天线位置和低频速度、主 IMU 连接的短时变化 | 无可靠杆臂时的质心位置；5 Hz GNSS不能独立验证约 4 Hz 周期内平移波形 |
| R/ω | 主 gyro 的相对旋转和约 100 Hz 记录内的扑翼结构 | 绝对 yaw/慢变 bias、安装局部转动与整体姿态的分离；EKF 姿态不是独立证据 |
| a | 若 R/bias/frame 正确，比力约束 IMU 点的运动学加速度 | 质心 a、局部柔性振动和刚体加速度的分解；重力泄漏、时间和抽取混叠 |
| α | gyro 可解析频带内的导数和周期结构 | 未记录高频/缺口内变化、噪声放大的尖峰；没有独立 α 真值传感器 |

主 IMU 约 26 样本/翼周期足以让**基频结构**成为模型必须解释的观测；它不保证每个谐波、冲击峰和相位精度。记录 Nyquist 50 Hz 只是形式上限，前端平均、日志抽取和未知抗混叠决定更严格的可信带宽。不能为获得好看的 a/α 把强相干波形全部低通掉，也不能强制将每个原始尖峰解释为刚体运动。未知的结构模态需实验/多点 IMU/外部运动测量约束，GPTR 和 gnssFGO 都不能凭算法补齐。

## 7. 已固定的 Stage 02 航段提案

`configs/stage01_feasibility.json` 锁定完整 SHA、绝对 HRT 秒和理由。`scripts/stage01_select_segments.py` **实际执行通过**：三份均为 Stage 00 注记的历史 train；16 s 段以及前后 2 s 上下文处于原有 airborne 主 IMU+PVT 连续支持区间；重点 instance 0 的已有 gap/reset/origin 事件不与其重叠；每段对应两条既有 gyro-y 8 s 频谱窗。没有读取 ULog 或重新生成谱。

| 段 | 原始文件 / SHA 前缀 | HRT 秒 `[start,end)` | 选择理由（来自既有审计） |
|---|---|---|---|
| A | `9.16数据/log_2_2026-9-16-18-08-46.ulg` / `1894725285ba…` | `[101.704719,117.704719)` | 有效 airborne PVT fixed=155.232942/155.232942 s；主 IMU 选择匹配率 1；基频约 4.055/3.736 Hz；作为正常适配基线 |
| B | `9.15数据/log_12_2026-9-15-18-03-58.ulg` / `5abe5e80b029…` | `[117.707089,133.707089)` | fixed=152.823352/152.823352 s；主 IMU 选择匹配率 0；基频约 4.036/3.961 Hz；专门暴露设备/bias误配问题 |
| C | `9.13数据/log_7_2026-9-13-18-12-52.ulg` / `3746db93c8eb…` | `[94.456471,110.456471)` | 全航次 valid airborne PVT仅 2.188918/89.801744 s fixed；eph/epv 中位约 3.984/7.233 m；主 IMU匹配；基频约 4.035/3.906 Hz；用于弱 GNSS 诊断 |

表中 fixed 比例、精度和设备匹配率是**逐航次已有统计**；基频来自两条指定窗口。不能将 C 的全航次质量自动写成段内每一时刻的 fix 状态，也不能把“无已有事件”扩大为原始硬件绝无异常。未来读取这三小段时要重新核对实际行有效性、参数/设备变化、epoch 和校准；若不满足预注册条件，记录拒绝，不依据重建误差另挑更好看的片段。

完整选段证据及 7 张 Stage 00 输入表的 SHA 存 `runs/stage01_20261017/prototype_segments.json`。历史 train 注记不是新注册 split，也不是全局未污染评估集；这三段只用于开发/可行性，不作为最终泛化证明。

## 8. 适配工作量和 Stage 02 实施条件

工作量以下按修改面估计，不是已经完成的工时承诺：

| 工作包 | 修改面/规模估计 | 当前状态 |
|---|---|---|
| 核心构建、数值探针 | 1 个 CMake target、构建 utility bridge、解析测试 | 已在 Conda 环境编译执行；无需 ROS 示例栈 |
| 查询/参数化兼容 | 查询 wrapper、真实 manifold/因子转换、显式一行 PSI 补丁 | 已验证；小角度导数截断仍保留为误差限制 |
| ULog 小段读取与校准 | 各原生时序、device 校准/热补偿/参数事件、source hash | Stage 02 中等工作量；无需重建全量审计器 |
| 异步惯性 | 原始 factor 零权重分离可最小复用；独立 bias prior | 原始因子在分离时刻的残差/Jacobian 已检验；各轴协方差/健壮化需薄适配 |
| GNSS PVT | 新增 Ceres 六维因子、坐标/协方差/时间/杆臂转换 | 中等工作量；本轮仅完成模型与独立约定检查 |
| 科学边界 | 实物外参、安装刚性、yaw/时延/bias可辨识性、频段与不确定度 | 部分需要外部测量，不能仅按代码工作量解决 |

进入真实航段前的门槛：

1. 沿用本次依赖闭包、源码 SHA、编译参数和已验证的 query/manifold/PSI 适配。进入新角度/角速度范围前补查近零导数误差；若要求 1e−6 级 Jacobian 或严格跨分支可微，当前小角度截断不满足，需先完成稳定高阶展开的数值验证。此次适配不是原始 API 的生产放行，更不证明所有轨迹/参数范围。
2. 冻结 frame、时间代理、reference point、各 device 校准和 bias 处理；未测杆臂/延迟明确标注。零参数与默认值不能被当作实测外参。
3. 对 GNSS Ceres 因子执行非零杆臂、位置/速度坐标转换、非对角协方差、延迟符号、epoch 去重和 Jacobian 检查。不混接 GTSAM 类型。

**单独获得 Stage 02 授权后**，只读 A/B/C 及各前后 2 s 上下文；先 A，再 B，C 只诊断失效/不确定性。依据本次 4 Hz 合成误差先比较 20/50 ms knot，在真实重建前冻结候选及预设噪声/jerk敏感性矩阵；不依据网络预测误差调参。留出原生 GNSS epochs、gyro/accel 的稀疏时刻用于观测一致性检查，同时注明同传感器自相关使其不等同独立真值。PX4 状态仅作相关参考。

验收分别检查：原生时间观测残差/质量分层；连续 p/v/a 与 R/ω/α 及 knot 边界；独立离散积分与步长收敛；扑翼基频/可支持谐波的幅值、相位、机动峰保留；bias/重力/杆臂/延迟与 jerk 权重敏感性。输出条件性有效区间与不确定性限制，不以平滑、较小残差或 solver success 替代准确性。不做全航次、正式训练标签或神经网络。

## 9. 交付与停止状态

本仓库新增 Conda/试验配置、C++ 探针与已测适配、独立约定/固定选段/溯源脚本及复现说明。实际输出在忽略目录 `runs/stage01_20261017/`：

- `probe_original_final.csv`、`probe_adapted_final.csv`：两套最终 C++ 数值结果；各有 stderr 和可执行文件哈希。
- `ctest_adapted.log`：1/1 通过；47 个通过的内部数值检查之外，明确保留原 API 问题与小角度近似限制。
- `convention_checks_conda.json`：在新环境执行的 10 项独立解析约定检查，全部通过；它与上游 C++ 检查分别统计。
- `prototype_segments.json`：三段固定航段的既有 Stage 00 证据，未重读原始日志。
- `upstream_overlay.diff`：完整一行 PSI 修改；源头文件 SHA `c87455374511b707aa30741efd39c4baee8e4505ecbf346a8cc5efb86b737680`。
- `environment_review/conda-explicit-md5.txt`、`manifest_completed.json`：实际依赖闭包、源/适配/配置/文档/输出哈希、环境、上游提交和检查结果。原 `manifest.json` 是初始构建阻塞时的历史快照；`probe_original.csv`、`probe_original_v2.csv` 等较早结果保留但不是最终入口。

Python 编译、限定变更的 whitespace 检查和最终哈希检查完成。上游 GPTR/gnssFGO worktree 保持 clean。没有运行全量测试仪式、真实飞行优化或正式标签验收；本阶段结果只支持报告中列明的合成范围。

优先级：**GPTR 核心 + 本仓库已验证适配，条件优先；gnssFGO 的 GNSS 设计参考；暂不引入其完整优化框架。** 依赖和核心构建问题已解决。剩余的小角度导数精度、未知杆臂/结构振动/yaw/绝对时间链及真实观测噪声，需要在后续明确条件下处理；不能把合成数值成功写成实际六自由度物理真值。

Stage 01 为 **COMPLETE — synthetic feasibility with documented limitations**。下一阶段实施方案已给出，但 Stage 02 尚未授权或执行。
