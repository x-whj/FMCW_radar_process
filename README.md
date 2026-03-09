# FMCW 雷达 GPU 处理工程（`radar_app`）

## 1. 项目简介

本项目实现了一套基于 CUDA 的 FMCW 雷达处理链，支持离线回放与在线处理场景。

当前流程（对应你课题中的 S1-S6）：

1. 高速数据流传入主机。
2. CPU 完成组帧/解包与双缓冲调度。
3. GPU 对多通道 IQ 数据解包并转换为复数形式。
4. GPU 执行 RD 相关处理、CFAR 检测与候选点提取。
5. 对候选点做后处理（DBSCAN + 可选单目标时序选择）。
6. 检测结果回传 CPU 打印/上报。

可执行程序：`radar_app`

## 2. 目录结构

```text
app/                    # 主程序入口、离线回放循环
gpu/
  RadarPipeline.cu      # 总流程调度
  kernels/              # unpack / power / cfar / monopulse 内核
io/                     # OfflineReplay、UDP 接收、组帧
model/                  # RadarConfig、目标结构、通道映射
runtime/                # Logger、Metrics
CMakeLists.txt
```

## 3. 环境要求

- Linux / WSL2（可访问 NVIDIA GPU）
- 已安装 NVIDIA 驱动
- CUDA Toolkit（建议位于 `/usr/local/cuda`）
- `cmake >= 3.16`
- 支持 C++17 的 `g++`

建议先检查：

```bash
which nvcc
nvcc --version
/usr/bin/cmake --version
nvidia-smi
```

注意：`which nvcc` 必须指向有效 CUDA 工具链（例如 `/usr/local/cuda/bin/nvcc`），避免误用旧版本工具链。

## 4. 编译方法

在仓库根目录执行：

```bash
rm -rf build
/usr/bin/cmake -S . -B build \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=86
/usr/bin/cmake --build build -j$(nproc)
```

当前 `CMakeLists.txt` 默认架构为 `86`（适配 RTX 30 系列常见环境）。

## 5. 离线回放运行

`app/main.cpp` 默认从**当前工作目录**读取三路离线数据：

- `output_he.dat`：和通道（sum）
- `output_cha1.dat`：方位差通道（az diff）
- `output_cha2.dat`：俯仰差通道（el diff）

常用运行方式：

```bash
cd build
./radar_app
```

终端会打印：

- `[CFAR] hits=... peaks=...`
- `[DBSCAN] in=... clusters=... out=...`
- `[Offline Frame k] targets=n`
- `[Primary] ...`（当前帧主目标）

## 6. 输出文件说明

### 6.1 主轨迹 CSV

离线运行会生成：

- `offline_primary_track.csv`

字段如下：

- `frame`
- `valid`（`1` 表示有主目标，`0` 表示无主目标）
- `targets`（本帧目标数）
- `primary_idx`（主目标在本帧目标列表中的索引）
- `rbin`
- `dbin_c`（中心化 Doppler bin）
- `dbin_u`（未中心化 Doppler bin）
- `range_m`
- `vel_mps`
- `snr_db`
- `az_deg`
- `el_deg`

### 6.2 可选 RD 功率图输出

部分调试流程会生成 `power_map_frame_XXX.bin`，用于离线可视化分析。

## 7. 核心参数与调参建议

参数定义在 `model/RadarConfig.h`，可在 `app/main.cpp` 中覆盖。

### 7.1 CFAR / NMS

| 参数 | 调大影响 | 调小影响 |
|---|---|---|
| `cfar_peak_half_window` | 抑制更强，近邻峰更少 | 峰值更密集，杂点更容易出现 |
| `cfar_peak_min_snr_db` | 误检减少 | 检测更敏感但误检增加 |
| `cfar_zero_doppler_suppress_bins` | 更强抑制零速杂波 | 保留低速/零速目标 |
| `cfar_near_range_suppress_bins` | 近距杂波减少 | 可保留更近距离目标 |

### 7.2 后处理

| 参数 | 作用 |
|---|---|
| `post_min_snr_db` | 过滤弱点 |
| `post_max_abs_vel_mps` | 过滤不合理高速点 |
| `post_top_k` | 仅保留最强 K 个目标（`0` 为全保留） |

### 7.3 DBSCAN（二维：range-doppler）

| 参数 | 调大影响 | 调小影响 |
|---|---|---|
| `dbscan_eps_range_m` | 更容易在距离维合并 | 聚类更严格，易分裂 |
| `dbscan_eps_velocity_mps` | 更容易在速度维合并 | 速度维更严格分离 |
| `dbscan_min_points` | 小簇更难保留 | 小簇更容易被保留 |
| `dbscan_keep_noise` | 孤立点可保留 | 孤立点直接剔除 |

### 7.4 单目标时序模式

| 参数 | 作用 |
|---|---|
| `single_target_mode` | 启用基于连续性的主目标选择 |
| `single_track_gate_range_m` | 帧间距离门限 |
| `single_track_gate_velocity_mps` | 帧间速度门限 |
| `single_track_snr_near_max_db` | 仅在“接近最大 SNR”的候选中择优 |

## 8. MATLAB 轨迹验证示例

用于验证主轨迹是否与 RD 图中亮点运动一致：

```matlab
clc; clear; close all;

T = readtable('offline_primary_track.csv');
Tv = T(T.valid == 1, :);   % 只看有效主目标

figure('Name','Primary Track');
subplot(2,1,1);
plot(Tv.frame, Tv.rbin, 'o-'); grid on;
xlabel('frame'); ylabel('rbin');
title('距离 bin 轨迹');

subplot(2,1,2);
plot(Tv.frame, Tv.dbin_c, 'o-'); grid on;
xlabel('frame'); ylabel('dbin\_c');
title('中心化多普勒 bin 轨迹');
```

解读建议：

- `dbin_c` 长时间保持常数是正常的 bin 量化现象。
- `dbin_c` 正负切换通常对应径向运动方向变化。
- 起始/末尾少量离群帧常见，可通过门限与时序参数进一步稳住。

## 9. 常见问题排查

### 9.1 `ptxas fatal: Value 'sm_30' is not defined`

原因：使用了过旧或不匹配的 CUDA 工具链。

处理：

- 明确指定 `CMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc`
- 重新配置并构建

### 9.2 CMake 来自错误路径（例如其他工具软件自带 cmake）

检查并固定使用：

```bash
which cmake
/usr/bin/cmake --version
```

### 9.3 链接错误 `undefined reference to launch_cfar_and_peak_extract...`

原因：函数声明和定义签名不一致。

处理：保持 `RadarPipeline` 调用签名与 `cfar.cu` launcher 完全一致。

### 9.4 目标数过多或前几帧不稳定

可尝试：

- 增大 `cfar_peak_min_snr_db`
- 增大 `cfar_peak_half_window`
- 设置 `dbscan_min_points = 2~3`
- 单目标数据集开启 `single_target_mode = true`

### 9.5 `CUDA error: OS call failed or operation not supported on this OS`

通常是运行环境无法访问 GPU（例如当前 WSL/容器未正确透传）。
请先确认 `nvidia-smi` 与 CUDA 运行时可用。

## 10. Git 使用

本地已有提交时推送到远端：

```bash
git remote add origin <repo-url>
git push -u origin main
```

建议打版本标签：

```bash
git tag -a v0.1.0 -m "first working offline pipeline"
git push origin v0.1.0
```
