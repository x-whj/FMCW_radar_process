# FMCW 雷达 GPU 处理流水线（多目标追踪）

## 1. 项目概述

本项目实现了一套基于 CUDA 的 FMCW 雷达处理链，支持离线回放与后续在线集成。

当前端到端流程：

1. 读取多通道 IQ 原始数据。
2. GPU 执行 unpack + 慢时间加窗。
3. 各通道执行 Doppler FFT。
4. 生成和通道 RD 功率图。
5. 在 RD 图上执行 CFAR + NMS 峰值提取。
6. 对候选点做单脉冲测角与距离/速度估计。
7. 主机侧后处理：
   - 基础阈值过滤
   - DBSCAN 候选合并
   - 可选的 Doppler 线杂波抑制
8. 多目标追踪（Kalman + 门控关联 + 轨迹生命周期）。
9. 将轨迹输出到 CSV。

可执行文件：`radar_app`

## 2. 目录结构

```text
app/                    # 主程序入口与离线回放循环
gpu/
  RadarPipeline.cu      # GPU主流程 + 主机侧后处理
  kernels/              # unpack / power / cfar / monopulse 内核
tracking/
  MultiTargetTracker.*  # 多目标追踪模块
io/                     # OfflineReplay、UDP接收、组帧
model/                  # RadarConfig、目标结构、通道映射
runtime/                # 日志、统计
matlab/
  plot_multi_track.m    # 轨迹可视化脚本
CMakeLists.txt
```

## 3. 编译

```bash
rm -rf build
/usr/bin/cmake -S . -B build \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=86
/usr/bin/cmake --build build -j$(nproc)
```

## 4. 离线回放运行

`radar_app` 会从当前工作目录读取以下文件：

- `output_he.dat`（和通道）
- `output_cha1.dat`（方位差通道）
- `output_cha2.dat`（俯仰差通道）

### 4.1 运行配置档（profile）

支持两套运行参数：

- `single`（默认）：更保守，适合单目标/低杂波场景。
- `multi`：更宽松，适合多目标场景。

```bash
cd build
./radar_app --profile single
./radar_app --profile multi
```

帮助：

```bash
./radar_app --help
```

终端日志包含：

- `[CFAR] hits=... peaks=...`
- `[DBSCAN] in=... clusters=... out=...`
- `[DOPPLER] in=... suppressed_bins=... out=...`（启用时）
- `[Offline Frame k] detections=n tracks=m`
- `[Det] ...` 与 `[Track] ...`

## 5. 输出文件

### 5.1 多轨迹 CSV

离线运行输出：

- `offline_multi_track.csv`

字段说明：

- `frame`
- `track_id`（`-1` 表示该帧无输出轨迹）
- `confirmed`（`1` 已确认，`0` 未确认）
- `age`（轨迹生命周期，单位帧）
- `hits`（成功关联次数）
- `missed`（连续丢失帧数）
- `rbin`
- `dbin_c`（中心化 Doppler bin）
- `dbin_u`（未中心化 Doppler bin）
- `range_m`
- `vel_mps`
- `snr_db`
- `az_deg`
- `el_deg`
- `power`

### 5.2 RD 功率图导出（调试）

调试时可能输出 `power_map_frame_XXX.bin`，用于离线比对。

## 6. Track 字段含义

日志示例：

```text
[Track] id=98 conf=1 age=18 hits=18 miss=0 rbin=24 dbin_c=-15 range=3.6 m vel=-3.0 m/s snr=27.54 dB
```

含义：

- `id`：轨迹编号
- `conf`：是否已确认轨迹
- `age`：轨迹已存在帧数
- `hits`：累计命中次数
- `miss`：连续丢失次数
- `range/vel`：滤波后的轨迹状态

## 7. 关键参数

参数定义在 `model/RadarConfig.h`，默认 profile 在 `app/main.cpp` 设置。

### 7.1 检测与后处理

- `cfar_peak_min_snr_db`
- `cfar_peak_half_window`
- `post_min_snr_db`
- `post_top_k`
- `dbscan_*`
- `post_doppler_line_suppress_enable`
- `post_doppler_line_min_points`
- `post_doppler_line_keep_per_bin`

### 7.2 追踪

- `tracking_gate_range_m`
- `tracking_gate_velocity_mps`
- `tracking_confirm_hits`
- `tracking_max_missed_frames`
- `tracking_spawn_min_snr_db`
- `tracking_spawn_exclusion_range_m`
- `tracking_spawn_exclusion_velocity_mps`
- `tracking_output_only_updated_tracks`
- `tracking_output_min_age`
- `tracking_output_min_hits`

若要抑制短命假轨迹，可调大：

- `tracking_output_min_age`
- `tracking_output_min_hits`

## 8. MATLAB 轨迹可视化

脚本：

- `matlab/plot_multi_track.m`

运行方式：

```matlab
cd('/home/whj/cuda_workplace/radar_app/matlab');
plot_multi_track;
```

脚本功能：

- 读取 `offline_multi_track.csv`
- 保留 confirmed 轨迹
- 用 `min_track_rows`（默认 `6`）过滤短轨迹
- 绘制：
  - frame-range
  - frame-velocity
  - frame-SNR
  - range-velocity 相图
  - bin 域总览（`rbin`、`dbin_c`）

## 9. 常见问题

### 9.1 `CUDA error: OS call failed or operation not supported on this OS`

通常表示当前环境无法访问 GPU（常见于 WSL/容器配置问题）。

先检查：

```bash
nvidia-smi
```

### 9.2 目标/轨迹数量过多

可尝试：

- 调高 `cfar_peak_min_snr_db`
- 调高 `post_min_snr_db`
- 调高 `dbscan_min_points`
- 开启并收紧 Doppler 线抑制
- 调高 `tracking_output_min_hits` / `tracking_output_min_age`

### 9.3 轨迹容易断

可尝试：

- 降低 `tracking_confirm_hits`
- 增大 `tracking_max_missed_frames`
- 略微放宽 `tracking_gate_*`

## 10. Git 工作流

当前多目标开发分支：

- `feature/multi-target-tracking`

常用命令：

```bash
git status
git add .
git commit -m "feat: multi-target tracking pipeline and tuning profiles"
git push -u origin feature/multi-target-tracking
```

