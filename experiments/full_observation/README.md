# 全量观测连续轨迹重建

独立于历史 Stage 编号。主结果是所有实际参与优化的有效观测的原生时刻一致性，留出预测仅为辅助诊断。完整运行报告见 `docs/reports/full_observation_reconstruction_v1.md`；大文件保留于被 Git 忽略的 `runs/full_observation_reconstruction_v1/`。

## 实现与边界

- `solve.cpp` 是仓库 Stage02.6 H0/G0 入口的局部适配，复用 GPTR 连续查询、WNOJ、IMU 和 PVT 白化因子。`full_observation` 必须显式传入，任何残留 `holdout!=0` 均报错；H0 不因机械相位缺少而删除 IMU/GNSS。
- `recover.cpp` 复用 Stage02.4 固定慢变轨迹/偏置、拟合 36 个物理周期系数的顺序恢复。`prepare.py` 使用全部有效样本重拟合 H1–H3 模板；不载入历史训练周期模板。派生投影目标和原始传感器残差分别保存在 ledger，不能解释为独立双重似然。
- `query.cpp` 使用已有 SO(3) 查询与运动学组合，在连续支持外拒绝查询。姿态为 xyzw、机体 FRD 到 NED；a 为 NED 运动学加速度，不能直接与加速度计相减。
- 主参考点是主 IMU，刚性安装为假设。GNSS 零杆臂/110ms 延迟未标定；IMU 杆臂为零是参考点定义，不是质心标定。未知 COM/弹性形变不可由本次 RMSE 排除。
- 原生异步时间保留。GNSS 使用 `vehicle_gps_position` 的真实接收机 PVT，不加入同源融合状态因子。位姿初始化使用 GNSS、gyro 和非独立 PX4 姿态，不将融合姿态当真值。
- 上游 GPTR `2c0285a4f3141b131cc7b3aa77b3c6d979c0a2c2`，沿用 Stage01 经审计的 build-local PSI overlay，不修改上游。GPTR root CC BY-NC-SA 4.0、因子 GPLv3+ 的既有许可说明见 `docs/UPSTREAM.md`；新目录只复用本仓库适配，不复制上游源码到 Git。

## 构建（现有环境）

```bash
PREFIX=/home/zn/anaconda3/envs/flap-recon
$PREFIX/bin/cmake -S experiments/full_observation -B runs/full_observation_reconstruction_v1/build -G Ninja -DCMAKE_PREFIX_PATH=$PREFIX -DCMAKE_CXX_COMPILER=$PREFIX/bin/c++ -DCMAKE_C_COMPILER=$PREFIX/bin/cc -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'
$PREFIX/bin/cmake --build runs/full_observation_reconstruction_v1/build --target full_solve full_recover full_query --parallel 2
```

使用新的输出目录重跑，不覆盖现有结果。构建目录可通过 `--build` 指定。

## 通用单 ULog 入口

```bash
python experiments/full_observation/run.py \
  --ulog '/home/zn/QgcLogs/9.16数据/log_2_2026-9-16-18-08-46.ulg' \
  --output runs/full_observation_new_run \
  --build runs/full_observation_reconstruction_v1/build
```

入口顺序执行全日志事件/支持审计、有效连续区间提取、标定、全部有效样本周期拟合、方案 A、方案 B 慢变求解与物理恢复、原生残差/运动学检查、轨迹导出和哈希 manifest。`--segments selected.json` 可指定已审计连续支持内的目标区间，格式为 `[ {"start_s":101.704719,"end_s":117.704719} ]`。自动留出两端各 2s 上下文。当前严格提取器拒绝不能解释的标定、坐标复位和时间间隙，不保证所有 ULog 固件/硬件适配；不跨缺口插值，不拼接独立解。

A/B 及固定敏感性入口：

```bash
python experiments/full_observation/run.py --ab --output runs/full_observation_reconstruction_v1
python experiments/full_observation/run.py --sensitivity --output runs/full_observation_reconstruction_v1
```

它们依赖先运行 `prepare_ab` 生成的全量输入；精确命令、输入 SHA 和配置已保存在此次运行的 `*_execution.json`、`frozen_config.json`、各解 `run_config.json`。不要对现有目录重复运行。

## 任意时刻查询

准备带 `t` 表头的 HRT 秒 CSV，然后：

```bash
runs/full_observation_reconstruction_v1/build/full_query \
  runs/full_observation_reconstruction_v1/full_log/full_0_direct \
  times.csv new_query.csv
```

`full_0_phase` 为方案 B。`query_settings.csv` 记录目标可信区间；knot 覆盖更宽上下文，端部可信程度较低。输出 p/v/a/R/omega/alpha 来自同一连续曲线。相位模型与 knot 均保留；`trajectory_export.csv` 增加机体系运动学加速度、相位/相位速度、邻近原生观测质量与来源时间差。

## 评价与产物

- `analyze.py <run...>`：原生物理公式、逐轴/向量/1s/飞行模式/GNSS质量统计，白化及实际鲁棒权重，非独立姿态，有限差分、边界和独立积分；拒绝覆盖查询。
- `diagnostics.py <run...> --sensitivity`：原生 Lomb 图、相位谐波、非周期和观测固定的瞬态子集、慢变周期泄漏及固定参数差异。
- `audit_delivery.py`：独立预测与 C++ 一致、优化目标鲁棒成本闭合、全日志逐样本覆盖。
- `atlas.py`：本次 A/B/完整航次的离线中文 `index.html`，所有 JS 内嵌、不需要网络；42 MB，保留图中原生样本。
- `seal.py <output> --verify-history`：输入/方法/上游/配置/每个产物哈希和历史完整性。主 manifest 在报告与检查完成后写入。

`*_ledger.csv` 保留所有输入行、原始时间、参与/排除原因、观测/预测/未加权物理残差/白化残差/实际目标白化/实际 Huber 权重。`observation_metrics.json` 的 GNSS position/velocity 行共享同一个六维 Huber 块成本，不能把两行成本再相加。有效样本之外不产生貌似可信的残差。

数学检查阈值只验证数值一致性；receiver uncertainty 与固定 floors/IMU sigma 都不等于已标定噪声。未估计可靠后验，不使用拟合残差制造高频导数真值，不授权 Stage03、神经网络训练或外部仓库修改。
