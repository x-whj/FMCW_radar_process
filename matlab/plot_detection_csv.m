clc;
clear;
close all;

% 读取 raw_packet_replay 导出的检测点 CSV，并画出“每个检测点”的散点轨迹图。
% 适合先看检测点整体分布，不依赖 tracker 输出。

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(script_dir);

% 直接在这里改要看的 CSV 文件名，避免被旧文件抢先匹配。
csv_name = 'zero_beam_det_s15.csv';

csv_path = resolve_existing_file({ ...
    csv_name, ...
    fullfile(repo_root, csv_name)});

min_snr_db = -inf;      % 可以改高一点，例如 15 / 20
only_valid_angle = false;

tbl = readtable(csv_path);
if isempty(tbl)
    error('检测 CSV 为空: %s', csv_path);
end

mask = true(height(tbl), 1);
mask = mask & isfinite(tbl.range_m) & isfinite(tbl.vel_mps) & isfinite(tbl.snr_db);
mask = mask & (tbl.snr_db >= min_snr_db);
if only_valid_angle
    mask = mask & (tbl.valid_az ~= 0 | tbl.valid_el ~= 0);
end

tbl = tbl(mask, :);
if isempty(tbl)
    error('筛选后没有检测点，请调低 min_snr_db 或检查 CSV 内容。');
end

fprintf('csv_path = %s\n', csv_path);
fprintf('points = %d\n', height(tbl));
fprintf('frame range = [%d, %d]\n', min(tbl.frame_idx), max(tbl.frame_idx));
fprintf('range_m range = [%.3f, %.3f]\n', min(tbl.range_m), max(tbl.range_m));
fprintf('vel_mps range = [%.3f, %.3f]\n', min(tbl.vel_mps), max(tbl.vel_mps));
fprintf('snr_db range = [%.3f, %.3f]\n', min(tbl.snr_db), max(tbl.snr_db));

fig = figure('Name', 'Detection Scatter Inspect', 'Color', 'w', 'Position', [80 80 1600 900]);

subplot(2, 2, 1);
s1 = scatter(tbl.det_uid, tbl.range_m, 28, tbl.snr_db, 'filled');
grid on;
colorbar;
title('Range vs Detection ID');
xlabel('Detection UID');
ylabel('Range (m)');

subplot(2, 2, 2);
s2 = scatter(tbl.frame_idx, tbl.range_m, 28, tbl.snr_db, 'filled');
grid on;
colorbar;
title('Range vs Frame');
xlabel('Frame Index');
ylabel('Range (m)');

subplot(2, 2, 3);
s3 = scatter(tbl.frame_idx, tbl.vel_mps, 28, tbl.snr_db, 'filled');
grid on;
colorbar;
title('Velocity vs Frame');
xlabel('Frame Index');
ylabel('Velocity (m/s)');

subplot(2, 2, 4);
s4 = scatter(tbl.det_uid, tbl.abs_az_deg, 28, tbl.snr_db, 'filled');
grid on;
colorbar;
title('Azimuth vs Detection ID');
xlabel('Detection UID');
ylabel('Absolute Azimuth (deg)');

sgtitle(sprintf('Detection Scatter: %s', csv_path), 'Interpreter', 'none');

apply_datatips(s1, tbl);
apply_datatips(s2, tbl);
apply_datatips(s3, tbl);
apply_datatips(s4, tbl);

function apply_datatips(sc, tbl)
    sc.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('det_uid', tbl.det_uid);
    sc.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('frame_idx', tbl.frame_idx);
    sc.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('range_m', tbl.range_m);
    sc.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('vel_mps', tbl.vel_mps);
    sc.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('snr_db', tbl.snr_db);
    sc.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('abs_az_deg', tbl.abs_az_deg);
    sc.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('abs_el_deg', tbl.abs_el_deg);
    sc.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('beam_az_deg', tbl.beam_az_deg);
    sc.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('beam_el_deg', tbl.beam_el_deg);
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
