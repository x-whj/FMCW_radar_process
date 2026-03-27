# MATLAB 脚本说明

本目录用于配合 `radar_app` / `raw_packet_replay` 做离线验证和结果分析。

当前更推荐的使用方式是：

1. 先导出检测点 CSV
2. 用 `plot_detection_csv.m` 看检测点整体分布和轨迹趋势
3. 如果需要，再用 `compare_raw_packet_power_map.m` 对齐 GPU / Matlab 功率图
4. 对异常帧用 `inspect_detection_frame.m` 单帧展开诊断

注意：

- 当前输入数据已经确认是脉压后的三通道数据
- 因此 Matlab 侧也不需要再做 Range FFT
- 当前主要验证的是：
  - 原始抓包组帧是否正确
  - `Q/I` 顺序是否正确
  - RD 功率图是否与 GPU 一致
  - 检测点在时间/距离/速度上的分布是否合理

## 1. 当前主推荐脚本

### 1.1 `plot_detection_csv.m`

用途：

- 读取 `raw_packet_replay` 导出的检测点 CSV
- 画出检测点散点图，而不是轨迹确认后的 tracker 图
- 当前这是最核心的结果分析入口

当前图包括：

- `Range vs Detection ID`
- `Range vs Frame`
- `Velocity vs Frame`
- `Azimuth vs Detection ID`

适用场景：

- 先看所有检测点整体分布
- 判断是否存在明显轨迹带
- 看目标在时间轴上的距离/速度变化趋势
- 不依赖 tracker

当前脚本开头直接指定要看的文件名：

```matlab
csv_name = 'zero_beam_det_s15.csv';
```

常用参数：

- `csv_name`
  - 要分析的检测点 CSV 文件名
- `min_snr_db`
  - 最小 SNR 过滤门限，例如 `15` / `20`
- `only_valid_angle`
  - 是否只保留至少有一个有效角度的点

推荐用法：

1. 先运行 `raw_packet_replay` 导出检测点 CSV，例如：

```bash
./build/raw_packet_replay \
  --file build/26_03_19_11_37_37.dat \
  --profile multi \
  --det-csv zero_beam_det_s15.csv
```

2. 在脚本开头设置：

```matlab
csv_name = 'zero_beam_det_s15.csv';
min_snr_db = -inf;
only_valid_angle = false;
```

3. 在 Matlab 中运行：

```matlab
cd('/home/whj/cuda_workplace/radar_app');
run('matlab/plot_detection_csv.m');
```

怎么看图：

- `Range vs Frame`
  - 最适合看轨迹趋势
  - 横轴是帧序，能看出目标距离随时间怎么变化
- `Velocity vs Frame`
  - 最适合看目标速度趋势
- `Range vs Detection ID`
  - 只是检测点分布图，不是时间轨迹图
- `Azimuth vs Detection ID`
  - 适合快速看角度分布是否集中

### 1.2 `compare_raw_packet_power_map.m`

用途：

- 直接从原始抓包文件里还原指定帧
- 在 Matlab 里复现当前 C++ / CUDA 的主链：
  - `raw packet -> PRT -> CPI payload`
  - `[chirp][channel][sample][Q,I]`
  - `Q/I -> complex(I,Q)`
  - 慢时间窗
  - Doppler FFT
  - 和通道功率图
- 再与 `raw_packet_replay` 导出的 `power_map_frame_XXX.bin` 逐帧对比

适用场景：

- 验证前端数据格式和 RD 图生成是否正确
- 排查“是协议/解包问题，还是检测问题”

主要输入：

- `output_first_900MB.dat` 或仓库根目录/`build/` 下同名文件
- `power_map_frame_XXX.bin`
- `ChebWindow256.h`

建议先看这些输出：

- `Matlab peak`
- `GPU peak`
- `Peak delta`
- `MAE`
- `MaxAbsErr`

### 1.3 `inspect_dumped_power_map.m`

用途：

- 浏览 `raw_packet_replay` 导出的功率图 dump
- 同时看：
  - 原始 Doppler bin 视图
  - `fftshift` 后速度视图
  - 行能量分布

适用场景：

- 快速浏览整批帧
- 检查零多普勒亮线
- 看最强点是否总卡在固定 Doppler / Range 上

主要输入：

- 仓库根目录下的 `power_map_frame_XXX.bin`

常用开关：

- `play_all_frames = true/false`
- `frame_idx`
- `pause_s`

### 1.4 `inspect_detection_frame.m`

用途：

- 严格诊断某一帧为什么会出现大量检测点
- 不做“修饰性筛选”，只把该帧的 RD 图和检测点原样摊开

当前图包括：

- 原始 RD 功率图
- `fftshift` 后 RD 图
- 当前帧检测点的距离-速度散点
- `range bin` 检测计数
- `centered doppler bin` 检测计数
- 帧内检测序号与速度分布

适用场景：

- 某一帧突然爆出很多点
- 想分清是 RD 图本身异常，还是 CFAR/峰值提取过宽

当前主要配置：

```matlab
frame_idx = 51;
```

也可以改成别的异常帧，比如 `52`。

## 2. 历史/辅助脚本

### 2.1 `plot_multi_track.m`

用途：

- 读取 `offline_multi_track.csv`
- 画 tracker 输出轨迹

说明：

- 这是轨迹视角，不是检测点视角
- 当前阶段更推荐先看 `plot_detection_csv.m`

### 2.2 `plot_primary_track.m`

用途：

- 查看主目标轨迹 CSV

说明：

- 更偏向旧流程或单目标分析
- 现在不是主推荐入口

### 2.3 `fit_angle_calibration.m`

用途：

- 根据轨迹和真值做角度标定

说明：

- 属于后续标定脚本
- 不是当前 RD / CFAR / det 验证主链的一部分

### 2.4 `rd_quicklook.m`

用途：

- 快速浏览单通道 RD 图

说明：

- 更偏快速观察
- 如果要严谨对齐，优先用 `compare_raw_packet_power_map.m`

### 2.5 `compare_power_map_quantized.m`

用途：

- 做量化前后功率图对比

说明：

- 属于补充验证脚本
- 当前最核心的是原始抓包到 GPU dump 的一致性验证

### 2.6 `data_pross_2.m`

用途：

- 历史动态回放脚本

说明：

- 名字和当前主线不够一致
- 可以保留参考，但不是当前主推荐脚本

## 3. 推荐工作流

### 3.1 先看检测点轨迹趋势

先运行 `raw_packet_replay` 导出检测点 CSV，例如：

```bash
./build/raw_packet_replay \
  --file build/26_03_19_11_37_37.dat \
  --profile multi \
  --det-csv raw_packet_det.csv
```

然后在 Matlab 中运行：

```matlab
cd('/home/whj/cuda_workplace/radar_app');
run('matlab/plot_detection_csv.m');
```

这是当前最推荐的第一步，因为它能最快回答：

- 有没有明显轨迹带
- 距离和速度趋势是否连贯
- 杂点大致集中在哪些距离段

### 3.2 验证原始抓包前端是否正确

1. 先用 `raw_packet_replay` 导出功率图：

```bash
./build/raw_packet_replay \
  --file build/output_first_900MB.dat \
  --profile multi \
  --max-frames 1 \
  --dump-power-map \
  --power-stats
```

2. 再在 Matlab 中运行：

```matlab
cd('/home/whj/cuda_workplace/radar_app');
run('matlab/compare_raw_packet_power_map.m');
```

如果：

- 峰值位置一致
- `Peak delta = 0`
- `MAE` 很小

那么基本可以说明前端：

- 组帧正确
- `Q/I` 顺序正确
- RD 功率图生成正确

### 3.3 浏览整批功率图

```matlab
cd('/home/whj/cuda_workplace/radar_app');
run('matlab/inspect_dumped_power_map.m');
```

适合看：

- 最强点是否总在零多普勒
- 某批数据有没有明显运动目标
- 功率图随帧怎么变化

### 3.4 查异常帧

当你发现某一帧检测点突然异常多时：

1. 先在 `plot_detection_csv.m` 里看是哪个 `frame_idx`
2. 再到 `inspect_detection_frame.m` 里设置：

```matlab
frame_idx = 51;
```

3. 运行：

```matlab
cd('/home/whj/cuda_workplace/radar_app');
run('matlab/inspect_detection_frame.m');
```

## 4. 常见输入文件

Matlab 脚本当前主要会用到这些文件：

- `power_map_frame_XXX.bin`
- `raw_packet_det.csv`
- `zero_beam_det.csv`
- `zero_beam_det_s15.csv`
- `offline_multi_track.csv`
- `output_first_900MB.dat`
- `ChebWindow256.h`

部分脚本会自动在：

- 当前目录
- 仓库根目录
- `build/`

这几个位置查找文件。

## 5. 使用建议

### 5.1 先看 `plot_detection_csv.m`，不要先迷信 track

当前阶段更推荐：

- 先导出所有检测点
- 用 Matlab 看 `plot_detection_csv.m`
- 先确认 RD / CFAR / 测角是否正常

原因：

- 检测点更直接
- 轨迹会叠加额外的确认和关联逻辑
- 在按波位抽帧分析时，track 不一定稳定

### 5.2 `Range vs Detection ID` 不是轨迹图

它表示：

- 横轴：检测点写入 CSV 的流水号 `det_uid`
- 纵轴：该检测点距离

它适合看“距离分布”，不适合直接看时间轨迹。

真正更适合看轨迹的是：

- `Range vs Frame`
- `Velocity vs Frame`

### 5.3 `fftshift` 只影响显示坐标，不改变频谱本身

所以：

- `inspect_dumped_power_map.m` 里看 `fftshift` 前后主要是为了更直观解释速度轴
- 如果某一帧同一距离上铺满很多速度点，通常不是单纯 `fftshift` 问题

## 6. 后续建议

如果后面继续扩展 Matlab 侧分析，优先建议加：

1. 按波位分别统计检测点分布
2. 每帧只保留一个候选目标的可视化脚本
3. 单个 `range bin` 慢时间序列与 Doppler 频谱对比脚本
