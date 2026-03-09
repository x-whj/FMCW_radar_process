# MATLAB 验证脚本说明

本目录收纳与 `radar_app` 离线处理链配套的 MATLAB 验证脚本，用于：

- RD 图可视化检查
- 主轨迹结果核对
- Matlab 与 GPU 功率图一致性对比（含量化影响）

## 数据约定

离线 `.dat` 文件按以下格式存储：

- 数据类型：`float32`
- 排列方式：`I0, Q0, I1, Q1, ...`（IQ 交织）
- 典型维度：`rows = 256`（chirp），`cols = 664`（sample）

默认读取路径为仓库根目录下 `build/`。

## 脚本清单

### 1) `compare_power_map_quantized.m`

用途：

- 读取 `output_he.dat` 指定帧，计算 Matlab 版功率图
- 同时构造“量化后”输入（`float -> int16 -> complex`）做对照
- 读取 GPU 导出的 `power_map_frame_XXX.bin`
- 对比峰值位置、误差图、MAE 指标

主要输入：

- `build/output_he.dat`
- `build/power_map_frame_XXX.bin`

### 2) `data_pross_2.m`

用途：

- 读取 `output_cha2.dat`
- 逐帧计算 `fftshift(fft(...))` 形成 RD 视图
- 动态可视化亮点轨迹（可按需导出 GIF）

主要输入：

- `build/output_cha2.dat`

### 3) `plot_primary_track.m`

用途：

- 读取 `offline_primary_track.csv`
- 绘制 `frame-rbin` 与 `frame-dbin_c` 轨迹
- 统计有效/缺失主目标帧数

主要输入：

- `build/offline_primary_track.csv`

### 4) `rd_quicklook.m`

用途：

- 快速浏览单通道 RD 图（默认 `output_cha2.dat`）
- 适合做回放前后的定性检查

## 运行方式

1. 在仓库 `build/` 下先跑离线程序：

```bash
./radar_app
```

2. 在 MATLAB 中将当前目录切到仓库根目录，然后运行：

```matlab
run('matlab/compare_power_map_quantized.m');
run('matlab/data_pross_2.m');
run('matlab/plot_primary_track.m');
run('matlab/rd_quicklook.m');
```

## 注意事项

- `compare_power_map_quantized.m` 中 `frame_idx` 为 0 基（与 C++ dump 命名一致）。
- 若你调整了 `rows/cols`，请同步修改脚本内参数。
- `data_pross_2.m` 默认 `pause(0.15)` 做动态回放，可按机器性能调节。
- MATLAB 临时文件（如 `*.asv`）已在项目 `.gitignore` 中忽略。
