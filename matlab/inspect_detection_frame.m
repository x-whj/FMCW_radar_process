clc;
clear;
close all;

% 严格诊断某一帧为什么会出现大量检测点。
% 不做“修饰性筛选”，只把该帧的 RD 图和检测点原样摊开。

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(script_dir);

frame_idx = 51;
num_chirps = 256;
num_samples = 664;

fc_hz = 28.2e9;
bw_hz = 20e6;
prt_s = 51e-6;
c_mps = 299792458;
lambda_m = c_mps / fc_hz;
range_resolution_m = c_mps / (2 * bw_hz);
velocity_resolution_mps = lambda_m / (2 * num_chirps * prt_s);

det_csv = resolve_existing_file({ ...
    'zero_beam_det.csv', ...
    fullfile(repo_root, 'zero_beam_det.csv'), ...
    'raw_packet_det.csv', ...
    fullfile(repo_root, 'raw_packet_det.csv')});

power_map_file = resolve_existing_file({ ...
    sprintf('power_map_frame_%03d.bin', frame_idx), ...
    fullfile(repo_root, sprintf('power_map_frame_%03d.bin', frame_idx)), ...
    fullfile(repo_root, 'build', sprintf('power_map_frame_%03d.bin', frame_idx))});

tbl = readtable(det_csv);
rows = tbl(tbl.frame_idx == frame_idx, :);
if isempty(rows)
    error('CSV 中找不到 frame_idx = %d 的检测点', frame_idx);
end

fprintf('frame_idx = %d\n', frame_idx);
fprintf('detections = %d\n', height(rows));
fprintf('range_m   = [%.3f, %.3f]\n', min(rows.range_m), max(rows.range_m));
fprintf('vel_mps   = [%.3f, %.3f]\n', min(rows.vel_mps), max(rows.vel_mps));
fprintf('snr_db    = [%.3f, %.3f]\n', min(rows.snr_db), max(rows.snr_db));

unique_rbin = unique(rows.rbin);
fprintf('\nRange-bin counts:\n');
for i = 1:numel(unique_rbin)
    rb = unique_rbin(i);
    cnt = sum(rows.rbin == rb);
    rr = mean(rows.range_m(rows.rbin == rb));
    fprintf('  rbin=%d count=%d mean_range=%.3f m\n', rb, cnt, rr);
end

fid = fopen(power_map_file, 'rb');
if fid < 0
    error('无法打开功率图文件: %s', power_map_file);
end
raw = fread(fid, [num_samples, num_chirps], 'float32=>single');
fclose(fid);
power_map = raw.';
power_db = 10 * log10(max(power_map, eps('single')));

range_axis_m = (0:num_samples-1) * range_resolution_m;
doppler_axis_raw = 0:num_chirps-1;
doppler_axis_shift = (-num_chirps/2):(num_chirps/2-1);
velocity_axis_shift_mps = doppler_axis_shift * velocity_resolution_mps;
power_db_shift = fftshift(power_db, 1);

fig = figure('Name', 'Inspect Detection Frame', 'Color', 'w', 'Position', [80 80 1600 920]);

subplot(2, 3, 1);
imagesc(range_axis_m, doppler_axis_raw, power_db);
axis xy;
colorbar;
title(sprintf('Raw RD Power (frame %d)', frame_idx));
xlabel('Range (m)');
ylabel('Raw Doppler Bin');
hold on;
plot(rows.range_m, rows.dbin_u, 'wo', 'MarkerSize', 7, 'LineWidth', 1.2);

subplot(2, 3, 2);
imagesc(range_axis_m, velocity_axis_shift_mps, power_db_shift);
axis xy;
colorbar;
title('RD Power (fftshift Doppler)');
xlabel('Range (m)');
ylabel('Velocity (m/s)');
hold on;
plot(rows.range_m, rows.vel_mps, 'wo', 'MarkerSize', 7, 'LineWidth', 1.2);

subplot(2, 3, 3);
scatter(rows.vel_mps, rows.range_m, 44, rows.snr_db, 'filled');
grid on;
colorbar;
title('Detections in Range-Velocity');
xlabel('Velocity (m/s)');
ylabel('Range (m)');

subplot(2, 3, 4);
histogram(rows.rbin, 'BinMethod', 'integers');
grid on;
title('Detection Count per Range Bin');
xlabel('Range Bin');
ylabel('Count');

subplot(2, 3, 5);
histogram(rows.dbin_c, 'BinMethod', 'integers');
grid on;
title('Detection Count per Centered Doppler Bin');
xlabel('Centered Doppler Bin');
ylabel('Count');

subplot(2, 3, 6);
scatter(rows.det_idx, rows.vel_mps, 44, rows.range_m, 'filled');
grid on;
colorbar;
title('Detection Index vs Velocity');
xlabel('Detection Index in Frame');
ylabel('Velocity (m/s)');

sgtitle(sprintf('Frame %d Detection Diagnosis', frame_idx));

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
