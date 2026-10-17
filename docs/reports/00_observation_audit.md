# Stage 00：全量九月原始 ULog 观测审计

审计日期：2026-10-17。分支：`reconstruction-audit`。结论：**观测支持开展有条件的、分段的运动学一致六自由度轨迹重建；优先验证 GPTR。当前不能承诺全部航次、全部频段或六种运动量具有相同精度。** 本阶段没有重建正式标签、运行完整轨迹优化、训练网络或进入 Stage 01。

## 1. 实际执行范围与身份

对 `/home/zn/QgcLogs` 递归遍历，排除 `.git` 内部对象，检查文件头；日志扩展名或 ULog magic 命中的文件都进入发现表。九月判定使用 `2026-9-*`/`2026.9.*` 名称及 `9.N数据` 等目录，未按历史建模资格筛选。年份省略的目录按本九月归档上下文纳入，GNSS UTC 另行核对；日期不是从模型数据集推断。

- 全库发现 489 个日志命名或 ULog magic 文件；九月候选 **208 个**。
- 九月 **116 个有效 ULog 路径**、**92 个 AppleDouble 元数据文件**（`._*`，非 ULog）。没有把元数据解析失败记成缺失航次。
- SHA-256 去重后 **91 份独立 ULog 内容**，25 个重复路径；约 2.420 GB 独立原始字节，实际解析 **13,645 个 topic/instance、280,551 个字段实例**。这里“航次”统计单位是去重后的记录，不额外声称每份文件都是独立实验或完全不同起降。
- 涉及 17 个顶层九月目录和 `9.12数据/归档`；实际日期为 9/6、7、8、11–19。没有在其他九月日期发现对应原始 ULog。
- 旧清单的 90 个 SHA 均已覆盖；新增 `2026.9.8/log_8_2026-9-8-07-00-42.ulg`，SHA `ac3158c8d122…`。它不在旧历史身份表中，仍纳入审计。
- 历史身份仅作注释：41 train、17 validation、6 Sep8 admitted、22 Sep19 admitted、1 Sep19 excluded_by_frozen_gate、3 not_admitted、1 旧表未列出。未将旧表认作新的权威 split，未修改任何历史划分。所有这些文件现在均有本次原始观测访问记录，不能再称未接触。
- 总记录 **18,506.34 s（5.141 h）**；记录标志支持的离地时间 **14,274.18 s（3.965 h）**。离地口径为 `vehicle_land_detected.landed==false` 且 `actuator_armed.armed==true`，不是独立视频验证的飞行真值。

历史清单 CSV 仅用于 SHA 身份核对，原有训练数据、50 Hz Parquet/CSV、模型预测和测试性能都没有作为输入。原始文件解析前后均核对内容 SHA。全部源库只读，初始 `AGENTS.md` 的既有未提交修改保留。

本次修订了旧 Stage 00 合同的“9/6–9/19、默认 DEVELOPMENT”范围，以及 DATA_CONTRACT、METHOD_DECISION、VALIDATION 中冲突的默认访问/选择措辞；保留测试性能隔离、参数拟合隔离、数据完整性和未来因果输入约束。没有新增 AGENTS 文件。

## 2. 输出入口与复现

最终机器可读输出在 `runs/stage00_20261017/audit_final/`；补充诊断在 `diagnostics_final/`；汇总与检查在 `validation/`；总交付清单在 `delivery_manifest.json`。这些大文件由 `.gitignore` 排除，均保留在本地，不能仅凭 Git 检出声称已获得原始数据或运行结果。

| 文件 | 内容 |
|---|---|
| `audit_final/discovery.csv`、`inventory.csv` | 全库发现范围、九月路径、magic 类型、字节数、完整 SHA、去重代表路径、历史身份及本次曝光 |
| `audit_final/topic_coverage.csv` | 每个 SHA/topic/instance 的全部字段、行数、时间基准、实际频率、P50/P95/max gap、重复/倒序/零时间、缺失槽估计、有效规则与时长 |
| `audit_final/field_statistics.csv` | 每个实际字段的数据类型与有限值比例；重点观测的分位数、离散计数和重置变化数 |
| `audit_final/timing.csv` | 原始 sample/publication 时钟质量及差值、跨 topic 最近采样偏差，保留零 sample stamp 证据 |
| `audit_final/quality.csv` | GNSS fix/float/fixed、精度元数据、有效时间；惯性真实积分时长与覆盖；相位有效性 |
| `audit_final/per_log.csv`、`logs/<SHA>.json` | 逐份汇总、固件 hash、观测缺口、关键参数/变更、logger dropout、组合覆盖 |
| `audit_final/date_topic_summary.csv`、`date_summary.csv` | 日期与全部日志的逐类/逐 instance 频率、有效秒数、支持记录数 |
| `audit_final/continuous_segments.csv` | 主惯性、主惯性+GNSS、状态链的实际支持区间 |
| `diagnostics_final/diagnostics.csv` | 同源 GNSS 核对、UTC epoch、航向异常、静止代理散布、传感器选择、扑翼周期采样、模式变化 |
| `diagnostics_final/boundary_events.csv` | 缺口/时间异常、EKF reset 和参考原点变化的发生区间 |
| `diagnostics_final/spectral_windows.csv` | 逐 SHA、逐连续 8 s 窗、逐轴的实测翼频、采样/周期比例、频谱能量和相位相干性 |
| `audit_final/observability_matrix.csv`、`method_requirements.csv` | 六种运动量的条件判断和两种方法的输入/复用/阻断条件 |
| `validation/checks.json` | 内容哈希、完整性、覆盖范围和两次独立原始读取结果的一致性检查 |
| `provenance_final/` | 7 个实际日志固件版本的相关消息/发布链与 SHA；两个上游固定提交和所读模块哈希 |

代码：`scripts/stage00_audit.py`、`stage00_diagnostics.py`、`stage00_source_provenance.py`、`stage00_validate.py`；配置：`configs/stage00_audit.json`。使用已存在的 Python/pyulog/numpy/scipy/pandas 环境，未安装依赖。运行顺序为 audit → diagnostics → source provenance → validate；精确版本和命令存各 manifest。重新运行须修改配置的输出目录（辅助脚本使用配置中的 run 父目录），使用新的 run 目录；脚本拒绝覆盖已有审计目录。

`audit/`、`audit_v2/`、`diagnostics/`、`diagnostics_v2/` 及最初 parser API 失败目录是保留的探索过程，**不是最终统计入口**。探索中修正了全零 `timestamp_sample` 的回退、Hall 相位每周期复位和初始 NaN 对 unwrap 的传播；最终从原始文件重新生成。当前报告只使用 `*_final`。另保留 `diagnostics_before_heading_range_fix`（relative heading 的范围应允许 0–2π）和 `validation_initial_bounds_check`（RTL 非运动 topic 的尾端零时间导致首尾 span 为负；最终检查分开验证单调流与异常流）作为调试记录；未忽略异常原始行。

## 3. 统计口径：覆盖不等于独立观测真值

每条流分别计算实际时间差，不统一降采样到 50 Hz。`median_hz=1/median(positive dt)` 与整段平均记录率都输出；存在非均匀抽取时两者并不相同。例如主 accel 的间隔倒数约 102 Hz，而整段记录约 100 Hz。

时间顺序优先使用有实际值的 `timestamp_sample`；如果全部为零，明确回退到 `timestamp`，而不伪造测量时刻。相对 GNSS 的 sample 字段来自 CAN 消息远端时间，未验证到 PX4 HRT 的映射，所以覆盖/记录率用 publication clock，原始 sample clock 的异常仍输出。publication–sample 差不代表全部传感器延迟或滤波相位延迟。

“支持时间”是相邻有效样本之间的区间并集：时间递增、两端有效、间隔不超过该流正间隔中位数的 3 倍；不向首尾外推，不跨重置/参考原点变化。另输出 2/3/5 倍敏感性。这是检查日志连续性的描述规则，**不是运动频率的可观测门槛**。主 accel 支持时间在三种口径下为 18,471.98 / 18,472.02 / 18,472.13 s；GNSS PVT 为 17,668.32 / 17,674.15 / 17,674.75 s，主要结论不依赖 3 倍这一选择。

“估计缺失槽”由间隔相对中位周期估算；没有硬件序号时不能声称是真实丢包率。Hall、sensor_selection、模式/事件等异步流不应把长间隔解释成丢包。未知语义 topic 的默认支持仅代表时间戳记录覆盖，不代表物理有效。重点观测用实际 finite/valid/fix 标志；四元数 norm 0.01 容差仅为异常诊断；GNSS fix 3/4/5/6 且速度有效定义基础 PVT 支持，不把每行都宣称 RTK 精确。

每个 instance 单独汇总，不把两个 IMU、两条同源 GPS 或多个 EKF instance 的秒数相加成“独立信息量”。在 `vehicle_imu` 上，topic 的记录跨度和真实积分区间覆盖是两种不同统计，后者必须看 `quality.csv`。

## 4. 实际记录的有价值观测

| 观测 | 去重日志覆盖 | 实际记录速率 | 支持时间：全部 / 离地（s） | 可以怎样使用 |
|---|---:|---|---:|---|
| `sensor_accel[0]` | 91/91 | 约 100 Hz；dt 中位倒数 101.97–102.09 Hz | 18,472.02 / 14,273.71 | 驱动旋转/标度与短 FIFO 平均后的比力观测；需校准到目标 body frame |
| `sensor_gyro[0]` | 91/91 | 约 100 Hz；dt 中位倒数 100.19–100.22 Hz | 18,472.06 / 14,273.78 | 主要角速度测量；需要 device ID、校准与偏置处理 |
| 第二套 accel/gyro instance 1 | 91/91 | 约 1 Hz | 各约 18,367 / 14,258 | 健康/慢变对照；不能代替主流追踪扑翼周期 |
| `vehicle_angular_velocity` | 91/91 | 约 100 Hz | 18,472.08 / 14,273.74 | 已校准、减偏置和滤波的 body rate；含 `xyz_derivative`，但该导数是滤波生成，不是独立角加速度传感器 |
| `vehicle_attitude` | 91/91 | 约 50 Hz | 18,469.65 / 14,272.82 | body→NED 四元数融合状态；可初始化/软约束/对照，不能与输入 IMU 当作独立证据 |
| `vehicle_local_position` | 91/91 | 约 50 Hz | 18,228.47 / 14,166.26 | NED p/v 及融合 ax/ay/az；有效标志、参考原点和 reset 必须保留 |
| `vehicle_gps_position` | 90/91 | 约 5 Hz | 17,674.15 / 13,859.66 | **主要 GNSS PVT 入口**；接收机选择/时间处理后，非 EKF 位置状态 |
| `sensor_gps` | 90/91 | 约 1 Hz | 17,562.69 / 13,835.68 | 同一接收机更稀疏记录；全零 sample stamp，使用明确标注的 publication time 回退 |
| `vehicle_global_position` | 90/91 | 约 5 Hz | 17,851.62 / 13,958.28 | 已融合 GNSS/其他信息的 EKF global state，不当独立 GPS |
| `sensor_combined`、`vehicle_acceleration` | 91/91 | 约 1 Hz | 各约 18,346 / 14,258 | 前者校准后的短积分平均量，后者校准/减偏置/低通后的比力；记录带宽不支持直接恢复扑翼波形 |
| `vehicle_imu[0,1]` | 91/91 | 各约 2 Hz | 记录跨度支持约 18,438 / 14,270 | 每条增量仅约 4.9–5.0 ms；**真实积分时间约占 1%**，不是连续增量流 |
| `wing_phase` | 91 有记录，90 有有效相位 | 约 100 Hz | 17,809.71 / 14,099.74 | 有效相位可定位扑翼周期，但 Hall 重置、齿比与相位时基需要处理 |
| `flap_frequency`、`encoder_count`、`rpm[0]` | 91/91 | 约 100 Hz | 记录支持各约 18,472 / 14,274 | 100 Hz 发布不是 100 Hz 独立频率估计；频率/相位/转速有共享来源 |
| `hall_event` | 43/91 | 事件驱动；有事件时中位约 3.62–4.24 Hz | 事件间隔支持 6,442 / 6,305 | 原始脉冲时间线，有助于相位校核；未记录 Hall 不等于翼相位从未使用 Hall |

还有 `sensor_mag`、`sensor_baro`、`vehicle_air_data`、`differential_pressure`、airspeed 原始/校正链、EKF bias/innovations/status/selector、actuator/control/mission 状态等。全部已枚举，重点字段统计可检索；磁力计/气压计主要约 1 Hz，压差/空速用于工况或气动辅助，不能直接等同地速、惯性加速度或独立轨迹真值。没有发现记录下来的 accel/gyro FIFO，也没有可供紧耦合 GNSS 使用的已验证逐星原始观测。

### 惯性观测的来源与带宽限制

匹配固件的 `PX4Accelerometer.cpp`/`PX4Gyroscope.cpp` 显示 sensor_* 不是 ADC counts：已做安装旋转和标度，FIFO 路径做短时间梯形平均；主流每条 `samples=5`。`vehicle_imu_status` 报告主芯片 raw rate 约 2 kHz、驱动消息约 400 Hz，而 ULog 仅保留约 100 Hz；第二传感器约 8.1 kHz raw、每条 20 samples、ULog 约 1 Hz。**原始传感器速率、驱动消息速率、日志记录速率不能混用。** 约 2.5 ms 的短平均也不等于覆盖整个约 10 ms 日志间隔；抽取混叠仍需评估。

主流未记录到 clipping 行，也未记录 error_count 增长；这只涵盖实际保存的消息，不证明未记录时段从不饱和。87 份日志有“at_rest 且 flap_frequency=0”的代理片段：逐航次静止代理标准差的中位值，accel 三轴约 0.035/0.041/0.042 m/s²，gyro 约 0.0068/0.0080/0.0087 rad/s。静止代理仍含搬动/振动/偏置变化，不能当作已标定白噪声密度；长时间真实静止、Allan 分析和可靠传感器协方差仍未验证。

日志包含多个传感器与估计器。`sensor_selection` 表明高频 instance 0 **并非一直是 PX4 当前选中的传感器**；45 份记录的可追溯时间全部匹配高频 instance 0，18 份全部不匹配，19 份中途变化，9 份没有可追溯 selection 事件。不能把 selected EKF 的偏置随手减到高频 sensor 0 上。记录中的 `SENS_BOARD_ROT=8`、各设备校准、参数变化均在逐 SHA 元数据中，后续必须据匹配固件计算坐标变换。

### GNSS/RTK：位置很有价值，但时钟和精度不可想当然

`vehicle_gps_position` 与 `sensor_gps` 在 90 份日志中 device ID 都相同；稀疏 sensor_gps 的 UTC epoch 全部能匹配到 vehicle_gps_position。它们的对齐位置/速度字段也完全相同，是同源观测，不能给两套独立权重。vehicle_gps_position 的 90,335 行中有 7 次相邻 UTC epoch 重复、90,328 个逐日志不同 epoch；这些重复不是额外独立 GNSS 更新，下一阶段应合并。匹配 `VehicleGPSPosition.cpp` 是选择/可选融合 GPS 与 PPS 时间处理，并非 EKF 状态输出；当前日志的 sample/publication 时间相等，不能据此认定接收机绝对延迟为零。CAN 驱动给 `sensor_gps.timestamp` 写 HRT 接收时间，`timestamp_sample` 保留零；UTC epoch 有助于更新检查，但缺少经验证的硬件时钟/延迟映射。

PVT 有效观测的离地支持 13,859.66 s，其中 RTK fixed 13,064.50 s（94.26%），float 67.25 s；其余为基础 3D/差分解等，不能合称 fixed。逐航次离地有效 PVT 中 fixed 比例从 **1.33% 到 100%**，中位 100%，航次差异明显。例：9/8 `log_3_…06-03-36` 仅 0.80/59.95 s fixed；9/13 `log_7_…18-12-52` 仅 2.19/89.80 s fixed。

有效 PVT 的逐航次 `eph` 中位范围约 0.013–6.543 m、`epv` 约 0.023–7.925 m。它们是接收机报告的精度，不是实测误差，也不能只凭 fixed 标志宣称厘米级真值。CAN 代码将 `eph/epv` 由位置协方差开方，而 **`s_variance_m_s` 直接写入速度协方差最大对角元**，与消息注释“m/s accuracy estimate”的通用语义不同；后续须依据这个具体来源处理，不能盲目再平方或当作标准差。

`EKF2_GPS_POS_*`、`EKF2_IMU_POS_*` 的零参数不证明实际天线与 IMU/质心共点。真实 lever arm、机体参考点和外部实测精度仍未知。

### 航向和相对 GNSS：有字段不等于有有效基线

71 份日志记录 `sensor_gnss_relative`；其 `gnss_fix_ok`、`relative_position_valid`、carrier fixed 标志均没有有效置位。匹配的 `gnss_relative.cpp` 只填写 CAN heading、accuracy、baseline length、down 分量等，**根本没有填写完整相对位置和上述标志**。因此不能解释为“全部真实 RTK 相对定位失败”，也不能把默认零 position 当完整 NED baseline。该 topic 的 sample 时间有零值/回退且来源为远端消息，需要另做时基核验。

PVT 中存在大量有限 heading（有效 PVT 下约 58,119 行），但 heading_accuracy 常为零，存在约 `2.56e11` 量级异常值；relative 仅 3 行 heading_valid 置位也不足以建立可靠航向流。结合驱动的 RelPosHeading 与 ECEF 字段兼容传输路径，PVT 中没有一行同时满足有限正常范围的 heading 与正值、正常范围的 heading_accuracy。因此目前结论是**航向候选线索存在，独立 GNSS yaw 尚未验证**，而不是宣称完全没记录航向。

## 5. 连续性、时间关系与工况

90 份有 GNSS 的日志都具有连续共同支持的主 accel/gyro/GNSS 区间；离地联合覆盖 **13,859.19 s**，约占全部记录离地时间的 **97.09%**。每份有 GNSS 日志的最长离地共同支持段约 8.06–229.19 s；这不意味着整个航次无缺口，更不意味着完成了预积分数据要求。没有 GNSS 的 `9.19-2数据/log_6_2026-9-19-15-28-00.ulg` 仍完整保留在惯性和状态审计内。

全体有 232 条 logger dropout 记录，累计 7.869 s；topic gap 与 logger dropout 不是同一统计。最大主 gyro 间隔 0.636 s，发生于 `9.19-3数据/log_4_2026-9-19-17-44-06.ulg`（SHA `329162d9984d…`）；此时 accel/姿态/位置也有约 0.63 s 缺口，已跨多个扑翼周期，不能按完整周期观测处理。GNSS 最大记录间隔约 1.198 s。

38 份日志出现姿态 reset counter 变化，合计 48 次；局部位置 xy/z/vxy/vz reset 变化分别 62/74/70/42 次。计数仅是日志中相邻保存值的变化，不宣称捕获了所有内部 reset。原点变更和每个边界时刻均输出，后续必须分段或正确变换，不能跨跳变拟合一条“平滑真值”。

跨 topic 的采样偏差并非零：accel→最近 gyro 的逐航次中位约 0.319–0.375 ms，P95 可达约 2.05 ms；姿态→最近 body rate 的逐航次中位 0–2.495 ms，P95 可达约 4.99 ms；local position 与 attitude 的中位偏差可到约 -4.99 ms。wing phase 与最近 gyro 的中位偏差可约 ±3.45 ms。GNSS 对 local state 的最近采样偏差 P95 可约 13 ms。这些只是记录的采样关系，不识别未知的滤波/接收机绝对延迟。

离地、nav_state、命令/执行器、airspeed 和扑翼有效区间用于解释工况，没有根据控制激励或历史建模资格删掉任何日志。没有动力学模型性能参与质量排序。

### 按日期汇总（按 SHA 去重后计算）

| 日期 | 去重记录 | GNSS 记录 | 离地记录时间 s | 惯性+GNSS 联合离地 s | GNSS 有效离地内 fixed 比例 |
|---|---:|---:|---:|---:|---:|
| 2026-09-06 | 7 | 7 | 1087.57 | 1087.56 | 98.86% |
| 2026-09-07 | 10 | 10 | 1799.00 | 1787.55 | 83.61% |
| 2026-09-08 | 8 | 8 | 1113.69 | 1090.05 | 93.35% |
| 2026-09-11 | 3 | 3 | 345.41 | 316.64 | 83.06% |
| 2026-09-12 | 8 | 8 | 1421.31 | 1344.65 | 91.14% |
| 2026-09-13 | 6 | 6 | 905.30 | 850.89 | 85.94% |
| 2026-09-14 | 4 | 4 | 698.68 | 691.62 | 87.27% |
| 2026-09-15 | 6 | 6 | 893.68 | 893.68 | 99.82% |
| 2026-09-16 | 5 | 5 | 700.55 | 700.25 | 100.00% |
| 2026-09-17 | 8 | 8 | 1176.74 | 1176.71 | 100.00% |
| 2026-09-18 | 3 | 3 | 516.11 | 515.98 | 99.07% |
| 2026-09-19 | 23 | 22 | 3616.15 | 3403.63 | 99.10% |

日期比例为该日期有效时间池化比例；逐航次分布另见上文，不能把池化比例当作每航次的保证。完整的日期×topic×instance 表在 `date_topic_summary.csv`。

## 6. 扑翼动态究竟有多少观测支持

`phase_unwrapped_rad` 在 Hall 重定零时会返回周期起点，所以频谱诊断使用 `phase_rad` 的圆周展开，在有限值连续段内独立处理，再按各惯性流原生时间重采样作诊断。没有把无效初始 NaN 延续到后面的有效相位，也没有跨无效相位或大缺口补周期。

连续有效离地/扑翼区间中的非重叠 8 s 窗，使用 2 s Welch 子窗（约 0.5 Hz 频率分辨率），检查基于**本窗实际翼频**的 0.8–1.2 倍频带及与 sin(phase) 的相干性。8 s 与 ±20% 是描述性频谱窗口，不是优化参数或物理成功阈值；没有用保留测试模型表现调窗口。不同传感器的窗口起点略不同，逐轴行数不能当成独立窗口数。

89 份日志具有可用 8 s 频谱窗口；缺少窗口的两份为 9/6 `log_24_…06-28-38`（相位全无效）和 9/19 `log_2_…14-45-56`（有效扑翼区间不足 8 s）；仍保留在其他统计。窗口翼频中位约 **3.867 Hz**，P5–P95 为 **3.173–4.429 Hz**，窗口中位值全范围约 **1.724–4.807 Hz**。主 accel/gyro 每周期约 26 个样本，典型窗口最大间隔约 0.055–0.057 个周期；GNSS 5 Hz 仅约 1.3 样本/周期，1 Hz 校正加速度更不足，不能单靠它们恢复每次扑翼加速度。

全部有效窗逐轴合并后的描述性中位：主 accel z 的基频带功率占比约 **0.838**、相位相干性约 **0.993**；主 gyro y 约 **0.396 / 0.976**。其他轴支持程度不同（例如 accel y 基频带占比约 0.048、相干性约 0.265）。这些是 raw board-frame 信号的周期结构证据，不能直接称为质心加速度真值，也不能把共享机械振动/安装柔性排除掉。相干性与信噪比不是同一个量。

相对而言，扑翼基频、部分低阶谐波与机动趋势有数据支持；接近/超过日志 Nyquist 的变化、缺口内尖峰、精细高阶导数、传感器与质心之间的振动分量主要受先验及未验证机械假设支配。不能用“曲线可微”来掩盖这一缺口。

## 7. 六自由度重建能力判断

| 运动量 | 当前观测可支持的内容 | 可信度不足或待验证内容 |
|---|---|---|
| 位置 p | 90 份的 GNSS PVT/RTK 可锚定位置；融合 local state 提供初始化及时间连续参考 | 非 fixed 区间、GNSS 缺口、天线到目标点的位移；无 GNSS 航次的全局漂移不独立可观测 |
| 速度 v | 约 5 Hz GNSS NED velocity，加上约 100 Hz 惯性和状态链，支持平移趋势与动态约束 | 不能仅由 5 Hz GNSS 分辨扑翼周期内全部速度波形；CAN 速度精度单位必须适配 |
| 姿态 R | 约 50 Hz 融合四元数与约 100 Hz gyro，支持相对旋转及融合姿态附近的估计 | 绝对 yaw、gyro bias 与低频航向漂移；独立双天线 heading 尚不可靠；EKF 与 IMU 相关 |
| 角速度 ω | 高频 gyro 与校正 body rate 有直接支持，扑翼基频明确 | device/board-body 校准、不同选中传感器、低通延迟、抽取混叠、饱和未记录区间 |
| 线加速度 a | 100 Hz accel **比力**直接约束快速平移/振动；结合 R、bias、gravity 可约束运动学 a | 不可直接把 accel 当 a；外参、重力/姿态误差、质心 lever arm、结构振动与高次谐波仍限制可信度 |
| 角加速度 α | gyro 的时间结构和 PX4 `xyz_derivative` 提供带宽受限线索；连续 SO(3) 轨迹可给一致导数 | 没有独立 α 传感器，求导放大噪声；滤波导数不能当独立真值，缺测与高频部分更依赖轨迹先验 |

整体是 **CONDITIONAL**：足以启动观测约束下的一致轨迹原型，但不是全频段或所有航次的六自由度物理真值保证。相对更有直接支持的是 RTK 良好段的位置、GNSS 速度趋势、角速度与相对姿态；a/α 必须连同观测频带、有效性和条件性不确定度输出。未知噪声不能凭空变成准确协方差。

## 8. 方法决定、验证与停止点

详见 [GPTR / gnssFGO 适用性与模块清单](00_method_applicability.md)。优先级：

1. **GPTR：CONDITIONAL，首选小型适配原型。** 直接比力/角速度观测因子不要求已记录完整 IMU 增量；显式 p/v/a 与 R/ω/α、闭式 SO(3) 导数最贴近目标。需增加 GNSS PVT、校准/异步时间/重力/外参适配。
2. **gnssFGO 松耦合：CONDITIONAL，备选及 GNSS 因子参考。** 可从校准后的采样 IMU 数值预积分，并非必须已有 delta 字段；但当前抽取带宽、gap 和时延必须进入误差模型。Full WNOJ 的导数能力存在，需要额外验证，未被排除。
3. **直接用稀疏 vehicle_imu 拼连续 GNSS-INS / gnssFGO 紧耦合示例：当前输入 BLOCKED。** 前者缺连续增量，后者缺已验证逐星原始观测。不能用先验把缺失输入伪装成已测量。

**11 项聚焦单元测试和 18 项交付完整性检查全部通过**；Python 编译检查与限定变更范围的 `git diff --check` 通过。实际验证包括：每个独立 ULog 全 topic 解析、原始 SHA 前后核对、全部字段清点、时间/覆盖边界和敏感性检查、单独原始读取的谱与联合覆盖复核、7 个固件版本消息/发布源代码核对、两个上游固定版本及两篇论文阅读。聚焦单元测试覆盖去重与异名目录、AppleDouble、时间倒序、全零 sample clock、缺口/重置、不重叠覆盖、短积分、NaN 相位展开。没有做优化收敛、导出标签精度、独立真值或构建可行性验证。

下一阶段尚未授权。需要先锁定开发用小段、校准/时基/外参与相关性约定，验证 GPTR 核心运动学和 GNSS 因子，制定观测留出及扑翼动态保留检查；不得把本次全部日志的观测访问授权延伸为保留评估集上的参数调优。
