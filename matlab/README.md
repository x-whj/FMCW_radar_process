# MATLAB 测试脚本目录

本目录用于放置离线验证脚本（`.m` 文件）。

## 推荐结构

- `plot_primary_track.m`
  读取 `offline_primary_track.csv`，画主轨迹（`rbin`、`dbin_c`）。
- `rd_quicklook.m`
  快速查看 `output_cha2.dat` 的 RD 图（按帧）。
- 你自己的其他脚本（例如 `cfar_nms_demo.m`、`data_pross_2.m`）也可以直接放这里。

## 使用前准备

通常在仓库 `build/` 下先跑一次：

```bash
./radar_app
```

然后确认至少有这些文件可供 MATLAB 读取：

- `build/offline_primary_track.csv`
- `build/output_he.dat`
- `build/output_cha1.dat`
- `build/output_cha2.dat`

## MATLAB 运行示例

在 MATLAB 当前目录切到仓库根目录后执行：

```matlab
run('matlab/plot_primary_track.m');
run('matlab/rd_quicklook.m');
```

## 说明

- `*.asv` 等 MATLAB 临时文件已在项目 `.gitignore` 中忽略。
- 如果你要上传大体积数据文件（`.dat`/`.bin`），建议使用 Git LFS。
