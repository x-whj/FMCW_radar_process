# 雷达离线处理工程说明

## 1. 项目概述

本项目是一套基于 C++ / CUDA 的雷达离线处理工程，当前重点支持两类输入：

- 三个通道的离线 IQ 文件回放
- 原始抓包 `.dat` 文件回放

当前主流程是：

1. 组帧得到一个完整 CPI
2. GPU 解包并施加慢时间窗
3. 对每个通道做 Doppler FFT
4. 用和通道生成功率图
5. 在功率图上做 2D CA-CFAR + 峰值提取
6. 用差通道做比幅单脉冲测角
7. 主机侧做基础过滤、DBSCAN、Doppler 线抑制等后处理
8. 可选做多目标跟踪
9. 输出检测点 CSV、轨迹 CSV、功率图 dump

注意：

- 当前输入数据已经确认是脉压后的三通道数据
- 因此当前工程不做 Range FFT
- 当前 RD 图实际上是“距离门 x Doppler”图

## 2. 可执行程序

当前 CMake 会构建以下三个程序：

- `radar_app`
- `raw_packet_replay`
- `udp_offline_verify`

### 2.1 `radar_app`

用途：

- 旧的离线回放入口
- 从三个 float IQ 文件读取数据
- 适合快速验证算法主链是否通畅

输入文件固定为当前工作目录下：

- `output_he.dat`
- `output_cha1.dat`
- `output_cha2.dat`

其中：

- `output_he.dat`：和通道
- `output_cha1.dat`：方位差通道
- `output_cha2.dat`：俯仰差通道

### 2.2 `raw_packet_replay`

用途：

- 当前主要使用的原始抓包离线入口
- 从 `.dat` 抓包文件中恢复 PRT / CPI
- 跑完整 GPU 检测链
- 可导出功率图、检测点 CSV、轨迹 CSV

它比 `radar_app` 更接近真实协议链路。

### 2.3 `udp_offline_verify`

用途：

- 协议/PRT/CPI 级诊断工具
- 不跑完整 GPU 算法
- 用于确认：
  - PRT 头
  - PRT 跨度
  - CPI 长度
  - 波位统计
  - padding 情况

## 3. 目录结构

```text
app/
  main.cpp                  # 三文件离线回放入口 radar_app
  raw_packet_replay.cpp     # 原始抓包离线回放入口
  udp_offline_verify.cpp    # 协议诊断工具
  TuneProfiles.h            # single / multi 两套 profile

gpu/
  RadarPipeline.cu          # GPU主流程 + 主机侧后处理
  RadarPipeline.h
  kernels/
    unpack.cu               # [chirp][channel][sample][Q,I] -> [channel][chirp][sample]
    power.cu                # 和通道功率图
    cfar.cu                 # 2D CA-CFAR + 峰值提取
    monopulse.cu            # 距离/速度/比幅测角

io/
  FrameAssembler.h          # 原始抓包字节流 -> PRT -> CPI
  FrameRuntimeConfigBuilder.h
  OfflineReplay.h           # 三文件离线回放拼帧
  PrtProtocol.h             # PRT头解析、波位解码
  UdpReceiver.*             # 预留在线接收路径

tracking/
  MultiTargetTracker.*      # 多目标跟踪

model/
  RadarConfig.h
  FrameMetadata.h
  Calibration.h
  ChannelMap.h
  TargetTypes.h

runtime/
  Logger.h
  Metrics.h

matlab/
  README.md
  compare_raw_packet_power_map.m
  inspect_dumped_power_map.m
  plot_detection_csv.m
  inspect_detection_frame.m
  ...
```

## 4. 协议与数据格式

### 4.1 原始抓包协议

当前已经确认的协议事实：

- 单个 UDP payload：`1464 bytes`
- 单个 PRT 总跨度：`8784 bytes = 6 x 1464`
- 单个 PRT 头：`256 bytes`
- 单个 PRT 数据区：`7968 bytes`
- 一个 CPI 的协议 PRT 数：`261`
- 当前算法实际处理的有效 PRT 数：`256`
- 最后 5 个 PRT 用于切波位，不进入算法主处理

PRT 编号范围：

- 协议范围：`1 .. 261`
- 算法处理范围：`1 .. 256`

### 4.2 原始数据布局

单个 PRT 数据区布局：

```text
[channel][sample][Q,I]
```

整个 CPI 组装后的原始帧缓冲布局：

```text
[chirp/PRT][channel][sample][Q,I]
```

通道顺序固定为：

- `ch0`：和通道 `Sigma`
- `ch1`：方位差通道 `Delta_az`
- `ch2`：俯仰差通道 `Delta_el`

线路上的每个复数样点是 `int16`，按接收端 16-bit 小端读取后可视为：

```text
[Q, I]
```

GPU `unpack` 阶段会把它恢复为算法内部统一使用的 `(I, Q)`。

### 4.3 GPU 内部布局

`unpack.cu` 之后，GPU 立方体布局为：

```text
[channel][chirp][sample]
```

之后沿 `chirp` 维做 Doppler FFT。

和通道功率图可理解为：

```text
[doppler][range]
```

## 5. 波位与角度

PRT 头中包含波位角信息。

当前波位角解码公式为：

```text
(raw_angle - 1000) * 0.05  degree
```

例如：

- `raw = 1000` 对应 `0 deg`

当前工程里，比幅测角得到的是“相对波束的角误差”，最终输出角度时会把当前波位角加进去，因此导出的：

- `azimuth_deg`
- `elevation_deg`

表示的是目标相对雷达的最终角度。

## 6. 编译

```bash
rm -rf build
/usr/bin/cmake -S . -B build \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=86
/usr/bin/cmake --build build -j$(nproc)
```

默认构建目标：

- `build/radar_app`
- `build/raw_packet_replay`
- `build/udp_offline_verify`

## 7. 运行方式

### 7.1 `radar_app`

帮助：

```bash
./build/radar_app --help
```

支持选项：

- `--profile single|multi`

示例：

```bash
./build/radar_app --profile single
./build/radar_app --profile multi
```

说明：

- `radar_app` 使用固定输入文件名
- 当前不支持通过 CLI 指定输入文件路径

### 7.2 `raw_packet_replay`

帮助：

```bash
./build/raw_packet_replay --help
```

当前支持选项：

- `--file PATH`
- `--profile single|multi`
- `--max-frames N`
- `--payload-bytes N`
- `--track-csv PATH`
- `--det-csv PATH`
- `--quiet`
- `--power-stats`
- `--dump-power-map`
- `--only-zero-beam`
- `--beam-az-raw N`
- `--beam-el-raw N`
- `--cfar-peak-snr-db X`
- `--post-min-snr-db X`

示例：全量原始抓包回放

```bash
./build/raw_packet_replay \
  --file build/output_first_900MB.dat \
  --profile multi
```

示例：只跑前 100 帧并导出功率图

```bash
./build/raw_packet_replay \
  --file build/26_03_19_11_37_37.dat \
  --profile multi \
  --max-frames 100 \
  --power-stats \
  --dump-power-map
```

示例：只分析 `0/0` 波位

```bash
./build/raw_packet_replay \
  --file build/26_03_19_11_37_37.dat \
  --profile multi \
  --only-zero-beam \
  --det-csv zero_beam_det.csv \
  --track-csv zero_beam_track.csv
```

示例：显式指定原始波位

```bash
./build/raw_packet_replay \
  --file build/26_03_19_11_37_37.dat \
  --profile multi \
  --beam-az-raw 1000 \
  --beam-el-raw 1000
```

### 7.3 `udp_offline_verify`

帮助：

```bash
./build/udp_offline_verify --help
```

当前支持选项：

- `--file PATH`
- `--payload-bytes N`
- `--scan-bytes N`
- `--max-cpi N`
- `--preview-cpi N`
- `--inspect-prt N`
- `--inspect-bytes N`

典型用途：

- 推断 PRT 跨度
- 统计 CPI
- 检查波位分布
- 核对 padding

## 8. `--profile` 说明

工程目前统一使用 `app/TuneProfiles.h` 中的两套 profile：

- `single`
- `multi`

### 8.1 `single`

用途：

- 更保守
- 适合单目标、低杂波、先求稳的场景

主要特点：

- `cfar_peak_min_snr_db` 更高
- `post_min_snr_db` 更高
- 跟踪门控更紧
- 输出轨迹确认条件更严格

### 8.2 `multi`

用途：

- 更宽松
- 适合多目标或需要先把点“放出来”的场景

主要特点：

- `cfar_peak_min_snr_db` 更低
- `post_min_snr_db` 更低
- 速度范围更宽
- DBSCAN 半径更紧，但检测保留更积极

### 8.3 一个很重要的细节

`raw_packet_replay` 在 `multi` profile 上还会额外做一次“原始抓包 bring-up 放宽”：

- `cfar_peak_min_snr_db = 8.0`
- `post_min_snr_db = 6.0`
- `tracking_spawn_min_snr_db = 8.0`

这一步只发生在 `raw_packet_replay`，目的是：

- 先确认原始协议接入后能否正常出点
- 不让抓包接入初期因为门限过严而误判“算法没工作”

所以：

- `radar_app --profile multi`
- `raw_packet_replay --profile multi`

两者不是完全同一套最终门限。

## 9. 当前 CFAR 语义

当前已经明确收口为：

```text
threshold = alpha * noise
```

也就是说：

- 当前真正影响 CFAR 的核心参数是 `cfar_pfa`
- CLI 层主要暴露的是 `--cfar-peak-snr-db`
- 旧的 `cfar_threshold_scale` / `--cfar-scale` 语义已经删除，不再使用

这样做是为了避免“配置里看起来可调、实际算法里不生效”的语义断裂。

## 10. 输出文件

### 10.1 检测点 CSV

`raw_packet_replay` 默认输出：

- `raw_packet_det.csv`

也可通过 `--det-csv PATH` 指定文件名。

字段包括：

- `det_uid`
- `frame_idx`
- `frame_id`
- `det_idx`
- `beam_az_raw`
- `beam_el_raw`
- `beam_az_deg`
- `beam_el_deg`
- `rel_az_deg`
- `rel_el_deg`
- `abs_az_deg`
- `abs_el_deg`
- `valid_az`
- `valid_el`
- `rbin`
- `dbin_c`
- `dbin_u`
- `range_m`
- `vel_mps`
- `snr_db`
- `power`
- `az_err`
- `el_err`

说明：

- 这是当前最推荐的离线分析输出
- 更适合 Matlab / 上位机做散点图、轨迹趋势观察和跨波位分析

### 10.2 轨迹 CSV

`radar_app` 默认输出：

- `offline_multi_track.csv`

`raw_packet_replay` 默认输出：

- `raw_packet_track.csv`

也可通过 `--track-csv PATH` 指定文件名。

说明：

- 当前 tracker 仍可用于参考
- 但在按波位抽帧分析时，track 不一定稳定
- 现阶段更建议优先看 detection CSV

### 10.3 功率图 dump

启用 `--dump-power-map` 后，会在仓库根目录生成：

- `power_map_frame_000.bin`
- `power_map_frame_001.bin`
- `...`

它们可被 Matlab 脚本直接读取。

## 11. MATLAB 脚本

当前常用脚本包括：

- `matlab/inspect_dumped_power_map.m`
- `matlab/compare_raw_packet_power_map.m`
- `matlab/plot_detection_csv.m`
- `matlab/inspect_detection_frame.m`
- `matlab/plot_multi_track.m`
- `matlab/plot_primary_track.m`
- `matlab/rd_quicklook.m`
- `matlab/fit_angle_calibration.m`

推荐看法：

1. 先用 `compare_raw_packet_power_map.m` 对齐 GPU 与 Matlab 功率图
2. 再用 `plot_detection_csv.m` 看检测点整体分布
3. 对异常帧用 `inspect_detection_frame.m` 单帧展开

更详细的 Matlab 说明见：

- [matlab/README.md](/home/whj/cuda_workplace/radar_app/matlab/README.md)

## 12. 常见日志

常见日志项包括：

- `[Power] min=... max=... mean=... max_bin=(d,r)`
- `[CFAR] hits=... peaks=...`
- `[DBSCAN] in=... clusters=... out=...`
- `[DOPPLER] in=... suppressed_bins=... out=...`
- `[Raw Frame k] frame_id=... det=... tracks=... beam=(az,el)`
- `[Offline Frame k] detections=n tracks=m`

说明：

- `hits`：CFAR 过门限单元数
- `peaks`：NMS 后峰值数
- `det`：主机侧后处理后的最终检测点数
- `tracks`：当前帧输出的轨迹数

## 13. 当前工程状态建议

当前更推荐的分析方式是：

1. 先把所有检测点保存到 `det csv`
2. 在 Matlab 或上位机上看全量检测点分布
3. 再决定是否需要跟踪

原因：

- 检测点更直接反映 RD / CFAR / 测角是否正常
- 轨迹会叠加额外的时间基准、门控和确认策略
- 在波位扫描、抽帧分析场景下，track 可能不稳定

## 14. 常见问题

### 14.1 `CUDA driver version is insufficient for CUDA runtime version`

说明当前环境 CUDA 驱动与运行时不匹配。先检查：

```bash
nvidia-smi
```

### 14.2 原始抓包接入后完全不出点

建议依次检查：

- `compare_raw_packet_power_map.m` 是否与 GPU dump 对齐
- `--profile multi` 是否已经使用
- `--cfar-peak-snr-db` 和 `--post-min-snr-db` 是否过高

### 14.3 某一帧检测点突然很多

建议：

- 用 `inspect_detection_frame.m` 单独展开该帧
- 先确认是 RD 图本身异常，还是 CFAR / 峰值提取过宽

