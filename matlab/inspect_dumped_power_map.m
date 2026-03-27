clc;
clear;
close all;

% 读取 raw_packet_replay 导出的功率图二进制，并检查是否存在零多普勒亮线。
% 对应 C++ 布局：[chirp][sample]，float32，未做 fftshift。
%
% 用法：
% 1) 看单帧：play_all_frames = false，并设置 frame_idx
% 2) 看全部帧：play_all_frames = true，脚本会自动遍历 power_map_frame_*.bin

play_all_frames = true;
frame_idx = 0;
pause_s = 0.08;
max_frames_to_play = inf;

num_chirps = 256;
num_samples = 664;

fc_hz = 28.2e9;
bw_hz = 20e6;
prt_s = 51e-6;
c_mps = 299792458;
lambda_m = c_mps / fc_hz;
range_resolution_m = c_mps / (2 * bw_hz);
velocity_resolution_mps = lambda_m / (2 * num_chirps * prt_s);

range_axis_m = (0:num_samples-1) * range_resolution_m;
doppler_axis_raw = 0:num_chirps-1;
doppler_axis_shift = (-num_chirps/2):(num_chirps/2-1);
velocity_axis_shift_mps = doppler_axis_shift * velocity_resolution_mps;

bin_files = dir('power_map_frame_*.bin');
if isempty(bin_files)
    error('当前目录下找不到 power_map_frame_*.bin');
end
bin_names = {bin_files.name};
bin_names = sort(bin_names);

if play_all_frames
    selected_names = bin_names;
    if isfinite(max_frames_to_play)
        selected_names = selected_names(1:min(numel(selected_names), max_frames_to_play));
    end
else
    target_name = sprintf('power_map_frame_%03d.bin', frame_idx);
    if ~isfile(target_name)
        error('找不到功率图文件: %s', target_name);
    end
    selected_names = {target_name};
end

fprintf('range_resolution_m = %.6f\n', range_resolution_m);
fprintf('velocity_resolution_mps = %.6f\n', velocity_resolution_mps);
fprintf('frames_to_play = %d\n', numel(selected_names));

fig = figure('Name', 'Power Map Inspect', 'Color', 'w', 'Position', [80 80 1500 900]);

for file_idx = 1:numel(selected_names)
    bin_path = fullfile(selected_names{file_idx});
    fid = fopen(bin_path, 'rb');
    raw = fread(fid, inf, 'float32=>single');
    fclose(fid);

    expected_count = num_chirps * num_samples;
    if numel(raw) ~= expected_count
        error('文件 %s 元素数不对，期望 %d，实际 %d', bin_path, expected_count, numel(raw));
    end

    power_map = reshape(raw, [num_samples, num_chirps]).';
    power_db = 10 * log10(max(power_map, eps('single')));
    power_db_shift = fftshift(power_db, 1);

    [max_val, max_idx] = max(power_map(:));
    [max_doppler_raw, max_range_raw] = ind2sub(size(power_map), max_idx);

    row_energy = sum(power_map, 2);
    [~, strongest_row_raw] = max(row_energy);
    row_energy_shift = fftshift(row_energy, 1);
    [~, strongest_row_shift] = max(row_energy_shift);

    current_frame_idx = sscanf(selected_names{file_idx}, 'power_map_frame_%d.bin');
    if isempty(current_frame_idx)
        current_frame_idx = file_idx - 1;
    end

    fprintf('frame_idx = %d\n', current_frame_idx);
    fprintf('  raw max power = %.6g, raw max bin = (doppler=%d, range=%d)\n', ...
        max_val, max_doppler_raw - 1, max_range_raw - 1);
    fprintf('  strongest raw doppler row = %d\n', strongest_row_raw - 1);
    fprintf('  strongest shifted doppler row = %d (center row is %d)\n', ...
        strongest_row_shift - 1, floor(num_chirps / 2));
    fprintf('  raw max approx = (velocity=%.6f m/s, range=%.6f m)\n', ...
        (max_doppler_raw - 1) * velocity_resolution_mps, (max_range_raw - 1) * range_resolution_m);

    figure(fig);

    subplot(2, 2, 1);
    cla;
    imagesc(range_axis_m, doppler_axis_raw, power_db);
    axis xy;
    colorbar;
    title('Raw Power Map (dB, no fftshift)');
    xlabel('Range (m)');
    ylabel('Raw Doppler Bin');
    hold on;
    plot(range_axis_m(max_range_raw), doppler_axis_raw(max_doppler_raw), ...
        'rx', 'MarkerSize', 12, 'LineWidth', 2);

    subplot(2, 2, 2);
    cla;
    imagesc(range_axis_m, velocity_axis_shift_mps, power_db_shift);
    axis xy;
    colorbar;
    title('Power Map (dB, fftshift on Doppler)');
    xlabel('Range (m)');
    ylabel('Velocity (m/s)');

    subplot(2, 2, 3);
    cla;
    plot(doppler_axis_raw, 10 * log10(max(row_energy, eps('single'))), 'LineWidth', 1.5);
    grid on;
    title('Row Energy Before fftshift');
    xlabel('Raw Doppler Bin');
    ylabel('Sum Power (dB)');
    xline(0, '--r', 'bin 0');

    subplot(2, 2, 4);
    cla;
    plot(velocity_axis_shift_mps, 10 * log10(max(row_energy_shift, eps('single'))), 'LineWidth', 1.5);
    grid on;
    title('Row Energy After fftshift');
    xlabel('Velocity (m/s)');
    ylabel('Sum Power (dB)');
    xline(0, '--r', 'zero Doppler');

    sgtitle(sprintf('Dumped Power Map Frame %03d (%d / %d)', ...
        current_frame_idx, file_idx, numel(selected_names)));

    drawnow;
    if play_all_frames && file_idx < numel(selected_names)
        pause(pause_s);
    end
end
