clc;
clear;
close all;

%% =========================
% 参数配置
% ==========================
rows = 256;          % num_chirps
cols = 664;          % num_samples
frame_idx = 1;       % 要对比的帧号，从0开始
data_dir = 'build';

he_file   = fullfile(data_dir, 'output_he.dat');
dump_file = fullfile(data_dir, sprintf('power_map_frame_%03d.bin', frame_idx));

%% =========================
% 1) 读取和通道 output_he.dat
% 文件格式：
% float, float, float, ...
% 按 [I0, Q0, I1, Q1, ...] 交织
% ==========================
fid = fopen(he_file, 'rb');
if fid < 0
    error('Failed to open %s', he_file);
end

raw = fread(fid, 'float');
fclose(fid);

if mod(length(raw), 2) ~= 0
    error('Float count is not even, IQ pairing failed.');
end

% 组复数
st_tt = raw(1:2:end) + 1i * raw(2:2:end);

samples_per_frame = rows * cols;
if mod(length(st_tt), samples_per_frame) ~= 0
    error('File does not contain integer number of frames.');
end

num_frames = length(st_tt) / samples_per_frame;
fprintf('Total frames in output_he.dat = %d\n', num_frames);

if frame_idx < 0 || frame_idx >= num_frames
    error('frame_idx out of range.');
end

%% =========================
% 2) reshape 成 [total_chirps, cols]
% 然后切出第 frame_idx 帧
% ==========================
data = reshape(st_tt, [cols, length(st_tt)/cols]).';

row_begin = frame_idx * rows + 1;
row_end   = row_begin + rows - 1;
data1 = data(row_begin:row_end, 1:cols);

%% =========================
% 3) Matlab 原始 float 版本
% 这个是“未量化”的参考
% ==========================
mtd_coeff = chebwin(rows);

% 为了和当前 C++ 未fftshift的流程对齐，这里先不做 fftshift
echo_float = fft(data1 .* mtd_coeff, rows, 1);
matlab_power_float = abs(echo_float).^2;

%% =========================
% 4) Matlab 量化对照版本
% 模拟 C++ 离线回放中的：
% float -> int16 -> GPU
%
% 注意：
% C++ 里是对 I/Q 各自做 int16 截断
% 然后再当作复数输入 GPU
% ==========================
data1_q = complex( ...
    single(int16(real(data1))), ...
    single(int16(imag(data1))) ...
);

echo_quant = fft(data1_q .* mtd_coeff, rows, 1);
matlab_power_quant = abs(echo_quant).^2;

%% =========================
% 5) 读取 GPU dump 的功率图
% C++ dump 的是 float，尺寸 [rows][cols]
% 写出顺序是按内存平铺的 [doppler][range]
% Matlab 用 fread([cols, rows]) 再转置回来
% ==========================
fid = fopen(dump_file, 'rb');
if fid < 0
    error('Failed to open %s', dump_file);
end

gpu_power = fread(fid, [cols, rows], 'float');
fclose(fid);

gpu_power = gpu_power.';   % 转成 [rows x cols]

%% =========================
% 6) 计算误差
% ==========================
diff_float_vs_gpu = abs(matlab_power_float - gpu_power);
diff_quant_vs_gpu = abs(matlab_power_quant - gpu_power);

%% =========================
% 7) 主峰位置比较
% ==========================
[max_float, idx_float] = max(matlab_power_float(:));
[float_doppler, float_range] = ind2sub(size(matlab_power_float), idx_float);

[max_quant, idx_quant] = max(matlab_power_quant(:));
[quant_doppler, quant_range] = ind2sub(size(matlab_power_quant), idx_quant);

[max_gpu, idx_gpu] = max(gpu_power(:));
[gpu_doppler, gpu_range] = ind2sub(size(gpu_power), idx_gpu);

fprintf('\n===== Peak Comparison =====\n');
fprintf('Matlab FLOAT peak : value=%g, doppler=%d, range=%d\n', ...
    max_float, float_doppler, float_range);

fprintf('Matlab QUANT peak : value=%g, doppler=%d, range=%d\n', ...
    max_quant, quant_doppler, quant_range);

fprintf('GPU peak          : value=%g, doppler=%d, range=%d\n', ...
    max_gpu, gpu_doppler, gpu_range);

fprintf('Delta(FLOAT vs GPU): doppler=%d, range=%d\n', ...
    gpu_doppler - float_doppler, gpu_range - float_range);

fprintf('Delta(QUANT vs GPU): doppler=%d, range=%d\n', ...
    gpu_doppler - quant_doppler, gpu_range - quant_range);

%% =========================
% 8) 画图对比
% 第一行：float / quant / gpu
% 第二行：float-gpu / quant-gpu / 行切片比较
% ==========================
figure('Name', 'Power Map Comparison', 'Color', 'w', 'Position', [100 100 1500 800]);

subplot(2,3,1);
imagesc(matlab_power_float);
axis xy;
title('Matlab power (float, no fftshift)');
xlabel('Range bin');
ylabel('Doppler bin');
colorbar;

subplot(2,3,2);
imagesc(matlab_power_quant);
axis xy;
title('Matlab power (quantized, no fftshift)');
xlabel('Range bin');
ylabel('Doppler bin');
colorbar;

subplot(2,3,3);
imagesc(gpu_power);
axis xy;
title('GPU dumped power map');
xlabel('Range bin');
ylabel('Doppler bin');
colorbar;

subplot(2,3,4);
imagesc(diff_float_vs_gpu);
axis xy;
title('|Matlab float - GPU|');
xlabel('Range bin');
ylabel('Doppler bin');
colorbar;

subplot(2,3,5);
imagesc(diff_quant_vs_gpu);
axis xy;
title('|Matlab quant - GPU|');
xlabel('Range bin');
ylabel('Doppler bin');
colorbar;

% 取 GPU 主峰所在 doppler 行，比较三者曲线
subplot(2,3,6);
plot(matlab_power_float(gpu_doppler, :), 'LineWidth', 1); hold on;
plot(matlab_power_quant(gpu_doppler, :), 'LineWidth', 1);
plot(gpu_power(gpu_doppler, :), 'LineWidth', 1);
grid on;
title(sprintf('Range cut at Doppler bin = %d', gpu_doppler));
xlabel('Range bin');
ylabel('Power');
legend('Matlab float', 'Matlab quant', 'GPU');

%% =========================
% 9) 可选：对数显示，更容易看弱目标/背景
% ==========================
eps_val = 1e-12;

figure('Name', 'Log Power Comparison', 'Color', 'w', 'Position', [120 120 1500 500]);

subplot(1,3,1);
imagesc(10*log10(matlab_power_quant + eps_val));
axis xy;
title('Matlab quant power (dB)');
xlabel('Range bin');
ylabel('Doppler bin');
colorbar;

subplot(1,3,2);
imagesc(10*log10(gpu_power + eps_val));
axis xy;
title('GPU power (dB)');
xlabel('Range bin');
ylabel('Doppler bin');
colorbar;

subplot(1,3,3);
imagesc(abs(10*log10(matlab_power_quant + eps_val) - 10*log10(gpu_power + eps_val)));
axis xy;
title('|Matlab quant dB - GPU dB|');
xlabel('Range bin');
ylabel('Doppler bin');
colorbar;

%% =========================
% 10) 打印一些整体误差指标
% ==========================
mae_float = mean(diff_float_vs_gpu(:));
mae_quant = mean(diff_quant_vs_gpu(:));

fprintf('\n===== Error Metrics =====\n');
fprintf('Mean abs error (FLOAT vs GPU) = %g\n', mae_float);
fprintf('Mean abs error (QUANT vs GPU) = %g\n', mae_quant);

if mae_quant < mae_float
    fprintf('Result: Quantized Matlab is closer to GPU.\n');
else
    fprintf('Result: Float Matlab is closer to GPU.\n');
end
