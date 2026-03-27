clc;
clear;
close all;

% 直接从原始抓包文件 output_first_900MB.dat 里按当前 C++ 逻辑还原一帧，
% 再生成 Matlab 版和通道功率图，与 raw_packet_replay 导出的 power_map_frame_XXX.bin 对比。
%
% 这份脚本的目标不是“看起来像”，而是尽量复现当前 C++ 这条链路：
% raw packet file -> PRT -> CPI payload -> [chirp][channel][sample][Q,I]
% -> sum channel complex(I,Q) -> slow-time window -> Doppler FFT -> power
%
% 注意：
% 1) 当前 GPU dump 是在“零多普勒首尾各若干行清零”之后导出的。
% 2) 因此这里默认也会在 Matlab 侧做同样的 zero-doppler notch。

%% =========================
% 参数配置
% ==========================
script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(script_dir);

frame_idx = 0;                % 要对比的 GPU 帧号，从 0 开始
raw_packet_file = resolve_existing_file( ...
    {'output_first_900MB.dat', ...
     fullfile('build', 'output_first_900MB.dat'), ...
     fullfile(repo_root, 'output_first_900MB.dat'), ...
     fullfile(repo_root, 'build', 'output_first_900MB.dat')});
gpu_dump_file = resolve_existing_file( ...
    {sprintf('power_map_frame_%03d.bin', frame_idx), ...
     fullfile(repo_root, sprintf('power_map_frame_%03d.bin', frame_idx)), ...
     fullfile(repo_root, 'build', sprintf('power_map_frame_%03d.bin', frame_idx))});
window_header_file = resolve_existing_file( ...
    {'ChebWindow256.h', ...
     fullfile(repo_root, 'ChebWindow256.h')});

num_chirps = 256;
num_samples = 664;
num_channels = 3;

protocol_prt_start_num = 1;
protocol_prts_per_cpi = 261;
udp_payload_bytes = 1464;
prt_header_bytes = 256;

apply_zero_doppler_notch = false;
zero_doppler_suppress_bins = 8;

%% =========================
% 波形参数（用于打印物理量）
% ==========================
fc_hz = 28.2e9;
bw_hz = 20e6;
prt_s = 51e-6;
c_mps = 299792458;
lambda_m = c_mps / fc_hz;
range_resolution_m = c_mps / (2 * bw_hz);
velocity_resolution_mps = lambda_m / (2 * num_chirps * prt_s);

%% =========================
% 协议尺寸
% ==========================
fixed_head_le = uint8([hex2dec('32'), hex2dec('CD'), hex2dec('55'), hex2dec('AA')]);
prt_data_bytes = num_channels * num_samples * 2 * 2;    % 3通道 * sample * [Q,I] * int16
prt_total_bytes = prt_header_bytes + prt_data_bytes;     % 不含尾部补齐
prt_padded_span = ceil(prt_total_bytes / udp_payload_bytes) * udp_payload_bytes;
frame_payload_bytes = num_chirps * prt_data_bytes;

fprintf('frame_idx = %d\n', frame_idx);
fprintf('prt_data_bytes = %d\n', prt_data_bytes);
fprintf('prt_total_bytes = %d\n', prt_total_bytes);
fprintf('prt_padded_span = %d\n', prt_padded_span);
fprintf('frame_payload_bytes = %d\n', frame_payload_bytes);
fprintf('range_resolution_m = %.6f\n', range_resolution_m);
fprintf('velocity_resolution_mps = %.6f\n', velocity_resolution_mps);

%% =========================
% 读取慢时间窗（与 C++ 完全一致）
% ==========================
slow_time_window = load_cheb_window_from_header(window_header_file, num_chirps);

%% =========================
% 从原始抓包文件还原指定 frame_idx 的 CPI payload
% ==========================
[frame_payload_u8, first_header_u8, first_prt_offset] = ...
    extract_frame_payload_from_raw_packets(raw_packet_file, ...
                                           fixed_head_le, ...
                                           frame_idx, ...
                                           num_chirps, ...
                                           num_samples, ...
                                           protocol_prt_start_num, ...
                                           protocol_prts_per_cpi, ...
                                           prt_header_bytes, ...
                                           prt_data_bytes, ...
                                           prt_padded_span);

fprintf('first_prt_offset = %d\n', first_prt_offset);
fprintf('first frame header: global_prt_count = %u, prt_num = %u\n', ...
    read_le_u32(first_header_u8(9:12)), ...
    bitshift(read_le_u32(first_header_u8(13:16)), -16));

%% =========================
% 还原和通道复数矩阵 [chirp x sample]
% 输入原始 payload 布局：[chirp][channel][sample][Q,I]
% ==========================
frame_i16 = typecast(frame_payload_u8, 'int16');
raw_4d = reshape(frame_i16, [2, num_samples, num_channels, num_chirps]);

sum_q = permute(raw_4d(1, :, 1, :), [4, 2, 3, 1]);
sum_i = permute(raw_4d(2, :, 1, :), [4, 2, 3, 1]);
sum_complex = complex(single(sum_i), single(sum_q));

%% =========================
% Matlab 版 RD / power map
% ==========================
sum_windowed = sum_complex .* reshape(single(slow_time_window), [num_chirps, 1]);
rd_matlab = fft(sum_windowed, num_chirps, 1);
power_matlab = abs(rd_matlab).^2;

if apply_zero_doppler_notch && zero_doppler_suppress_bins > 0
    notch = min(zero_doppler_suppress_bins, floor(num_chirps / 2));
    power_matlab(1:notch, :) = 0;
    power_matlab(end-notch+1:end, :) = 0;
end

%% =========================
% 读取 GPU dump
% ==========================
fid = fopen(gpu_dump_file, 'rb');
if fid < 0
    error('无法打开 GPU dump 文件: %s', gpu_dump_file);
end
gpu_raw = fread(fid, [num_samples, num_chirps], 'float32=>single');
fclose(fid);
gpu_power = gpu_raw.';

%% =========================
% 误差统计
% ==========================
abs_diff = abs(power_matlab - gpu_power);
mae = mean(abs_diff(:));
max_abs_err = max(abs_diff(:));

[max_matlab, idx_matlab] = max(power_matlab(:));
[matlab_doppler, matlab_range] = ind2sub(size(power_matlab), idx_matlab);
[max_gpu, idx_gpu] = max(gpu_power(:));
[gpu_doppler, gpu_range] = ind2sub(size(gpu_power), idx_gpu);

fprintf('\n===== Peak Comparison =====\n');
fprintf('Matlab peak : value=%g, doppler=%d, range=%d\n', ...
    max_matlab, matlab_doppler - 1, matlab_range - 1);
fprintf('GPU peak    : value=%g, doppler=%d, range=%d\n', ...
    max_gpu, gpu_doppler - 1, gpu_range - 1);
fprintf('Peak delta  : doppler=%d, range=%d\n', ...
    (gpu_doppler - matlab_doppler), (gpu_range - matlab_range));

fprintf('\n===== Error Metrics =====\n');
fprintf('MAE        = %g\n', mae);
fprintf('MaxAbsErr  = %g\n', max_abs_err);

%% =========================
% 可视化对比
% ==========================
eps_val = 1e-12;

figure('Name', 'Raw Packet Power Map Compare', 'Color', 'w', 'Position', [100 80 1600 900]);

subplot(2, 3, 1);
imagesc(10 * log10(max(power_matlab, eps_val)));
axis xy;
colorbar;
title('Matlab Power (dB)');
xlabel('Range bin');
ylabel('Doppler bin');

subplot(2, 3, 2);
imagesc(10 * log10(max(gpu_power, eps_val)));
axis xy;
colorbar;
title('GPU Dumped Power (dB)');
xlabel('Range bin');
ylabel('Doppler bin');

subplot(2, 3, 3);
imagesc(abs(10 * log10(max(power_matlab, eps_val)) - 10 * log10(max(gpu_power, eps_val))));
axis xy;
colorbar;
title('|Matlab dB - GPU dB|');
xlabel('Range bin');
ylabel('Doppler bin');

subplot(2, 3, 4);
imagesc(abs_diff);
axis xy;
colorbar;
title('|Matlab - GPU|');
xlabel('Range bin');
ylabel('Doppler bin');

subplot(2, 3, 5);
plot(power_matlab(gpu_doppler, :), 'LineWidth', 1.2);
hold on;
plot(gpu_power(gpu_doppler, :), 'LineWidth', 1.2);
grid on;
title(sprintf('Range Cut @ GPU Doppler Bin %d', gpu_doppler - 1));
xlabel('Range bin');
ylabel('Power');
legend('Matlab', 'GPU');

subplot(2, 3, 6);
plot(power_matlab(:, gpu_range), 'LineWidth', 1.2);
hold on;
plot(gpu_power(:, gpu_range), 'LineWidth', 1.2);
grid on;
title(sprintf('Doppler Cut @ GPU Range Bin %d', gpu_range - 1));
xlabel('Doppler bin');
ylabel('Power');
legend('Matlab', 'GPU');

sgtitle(sprintf('Raw Packet -> Power Map Compare (frame %03d)', frame_idx));

%% =========================
% 本地函数
% ==========================
function w = load_cheb_window_from_header(header_path, expected_count)
    text = fileread(header_path);
    block = regexp(text, '\{(.*)\}', 'tokens', 'once');
    if isempty(block)
        error('无法从 %s 提取窗函数数组', header_path);
    end

    tokens = regexp(block{1}, '[-+]?\d*\.?\d+(?:e[-+]?\d+)?', 'match');
    values = str2double(tokens);
    values = values(~isnan(values));

    if numel(values) ~= expected_count
        error('窗函数元素数不对，期望 %d，实际 %d', expected_count, numel(values));
    end

    w = single(values(:));
end

function [frame_payload_u8, first_header_u8, first_prt_offset] = extract_frame_payload_from_raw_packets( ...
    file_path, fixed_head_le, frame_idx, num_chirps, num_samples, protocol_prt_start_num, ...
    protocol_prts_per_cpi, prt_header_bytes, prt_data_bytes, prt_padded_span)

    fid = fopen(file_path, 'rb');
    if fid < 0
        error('无法打开原始抓包文件: %s', file_path);
    end
    cleaner = onCleanup(@() fclose(fid));

    prefix_scan_bytes = 32 * 1024 * 1024;
    prefix = fread(fid, prefix_scan_bytes, '*uint8');
    first_prt_offset = find_first_valid_head_offset(prefix, fixed_head_le, ...
        protocol_prt_start_num, protocol_prts_per_cpi, prt_padded_span, prt_header_bytes);

    fseek(fid, first_prt_offset, 'bof');

    frame_payload_u8 = zeros(num_chirps * prt_data_bytes, 1, 'uint8');
    first_header_u8 = zeros(prt_header_bytes, 1, 'uint8');

    cpi_started = false;
    received_mask = false(num_chirps, 1);
    received_count = 0;
    completed_frames = 0;

    while true
        block = fread(fid, prt_padded_span, '*uint8');
        if numel(block) < prt_padded_span
            error('文件结束前未能组出目标 frame_idx=%d', frame_idx);
        end

        hdr = block(1:prt_header_bytes);
        if ~isequal(hdr(1:4), fixed_head_le(:))
            error('PRT 对齐失效：offset=%d 处不是固定帧头', ftell(fid) - numel(block));
        end

        prt_info_word = read_le_u32(hdr(13:16));
        prt_num = bitshift(prt_info_word, -16);
        global_prt_count = read_le_u32(hdr(9:12)); %#ok<NASGU>

        if ~cpi_started
            if prt_num ~= protocol_prt_start_num
                continue;
            end

            cpi_started = true;
            received_mask(:) = false;
            received_count = 0;
            frame_payload_u8(:) = 0;
            first_header_u8 = hdr;
        elseif prt_num == protocol_prt_start_num
            cpi_started = true;
            received_mask(:) = false;
            received_count = 0;
            frame_payload_u8(:) = 0;
            first_header_u8 = hdr;
        end

        active_last_prt = protocol_prt_start_num + num_chirps - 1;
        if prt_num <= active_last_prt
            prt_idx = prt_num - protocol_prt_start_num;
            if prt_idx < 0 || prt_idx >= num_chirps
                error('PRT 索引越界: prt_num=%d', prt_num);
            end

            if ~received_mask(prt_idx + 1)
                dst_begin = prt_idx * prt_data_bytes + 1;
                dst_end = dst_begin + prt_data_bytes - 1;
                frame_payload_u8(dst_begin:dst_end) = block(prt_header_bytes + 1 : prt_header_bytes + prt_data_bytes);
                received_mask(prt_idx + 1) = true;
                received_count = received_count + 1;
            end
        end

        if received_count == num_chirps
            if completed_frames == frame_idx
                return;
            end

            completed_frames = completed_frames + 1;
            cpi_started = false;
            received_mask(:) = false;
            received_count = 0;
        end
    end
end

function offset0 = find_first_valid_head_offset(prefix, fixed_head_le, protocol_prt_start_num, protocol_prts_per_cpi, prt_padded_span, prt_header_bytes)
    hits = find_subsequence_u8(prefix, fixed_head_le);
    if isempty(hits)
        error('在前缀扫描区里没找到固定帧头');
    end

    for k = 1:numel(hits)
        pos = hits(k);
        if pos + prt_header_bytes - 1 > numel(prefix)
            continue;
        end

        hdr = prefix(pos : pos + prt_header_bytes - 1);
        prt_word = read_le_u32(hdr(13:16));
        prt_num = bitshift(prt_word, -16);
        if prt_num < protocol_prt_start_num || prt_num > protocol_prt_start_num + protocol_prts_per_cpi - 1
            continue;
        end

        next_pos = pos + prt_padded_span;
        if next_pos + 3 <= numel(prefix)
            if ~isequal(prefix(next_pos : next_pos + 3), fixed_head_le(:))
                continue;
            end
        end

        offset0 = pos - 1;
        return;
    end

    error('没有找到合法的首个 PRT 帧头位置');
end

function value = read_le_u32(bytes4)
    b = uint32(bytes4(:));
    value = b(1) + bitshift(b(2), 8) + bitshift(b(3), 16) + bitshift(b(4), 24);
end

function hits = find_subsequence_u8(buffer_u8, pattern_u8)
    buffer_u8 = uint8(buffer_u8(:));
    pattern_u8 = uint8(pattern_u8(:));

    n = numel(buffer_u8);
    m = numel(pattern_u8);
    if m == 0 || n < m
        hits = [];
        return;
    end

    hits = [];
    first_matches = find(buffer_u8(1:n-m+1) == pattern_u8(1));
    for idx = 1:numel(first_matches)
        pos = first_matches(idx);
        if all(buffer_u8(pos:pos+m-1) == pattern_u8)
            hits(end+1) = pos; %#ok<AGROW>
        end
    end
end

function path_out = resolve_existing_file(candidates)
    for i = 1:numel(candidates)
        if isfile(candidates{i})
            path_out = candidates{i};
            return;
        end
    end

    msg = sprintf('找不到输入文件，尝试过这些路径：\n');
    for i = 1:numel(candidates)
        msg = sprintf('%s  - %s\n', msg, candidates{i}); %#ok<AGROW>
    end
    error('%s', msg);
end
