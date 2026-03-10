clc; clear; close all;

% Try common output locations.
csv_candidates = {
    'offline_multi_track.csv'
    fullfile('build', 'offline_multi_track.csv')
    fullfile('..', 'build', 'offline_multi_track.csv')
};

csv_path = '';
for i = 1:numel(csv_candidates)
    if isfile(csv_candidates{i})
        csv_path = csv_candidates{i};
        break;
    end
end

if isempty(csv_path)
    error('offline_multi_track.csv not found. Run radar_app first.');
end

T = readtable(csv_path);

% -------- Plot controls (tunable) --------
min_track_rows = 6;          % Drop short-lived tracks
angle_min_snr_db = 20;       % Use high-SNR points for angle trend view
angle_smooth_window = 3;     % Moving-average window for az/el trend
show_raw_angle = true;       % Overlay raw az/el with dashed lines
% ----------------------------------------

% Keep only valid confirmed track rows.
valid = (T.track_id ~= -1) & (T.confirmed == 1);
Tv = T(valid, :);

if isempty(Tv)
    error('No confirmed tracks found in %s', csv_path);
end

% Drop short-lived tracks (can be adjusted).
track_ids_raw = unique(Tv.track_id);
keep_id = false(size(track_ids_raw));
for i = 1:numel(track_ids_raw)
    id = track_ids_raw(i);
    keep_id(i) = sum(Tv.track_id == id) >= min_track_rows;
end
track_ids = track_ids_raw(keep_id);
Tv = Tv(ismember(Tv.track_id, track_ids), :);

if isempty(Tv)
    error('No tracks left after min_track_rows=%d filtering.', min_track_rows);
end

n_tracks = numel(track_ids);
cmap = lines(max(n_tracks, 1));

fprintf('CSV: %s\n', csv_path);
fprintf('Confirmed tracks: %d\n', n_tracks);
fprintf('min_track_rows filter: %d\n', min_track_rows);
fprintf('Track IDs: ');
fprintf('%d ', track_ids);
fprintf('\n');
fprintf('Note: az/el are trend-only before calibration.\n');
fprintf('Angle view: snr >= %.1f dB, smooth_window = %d\n', ...
    angle_min_snr_db, angle_smooth_window);

figure('Name', 'Track vs Frame', 'Color', 'w');
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile; hold on; grid on;
for k = 1:n_tracks
    id = track_ids(k);
    Tk = sortrows(Tv(Tv.track_id == id, :), 'frame');
    plot(Tk.frame, Tk.range_m, '-o', ...
        'Color', cmap(k, :), 'LineWidth', 1.2, 'MarkerSize', 4, ...
        'DisplayName', sprintf('ID %d', id));
end
xlabel('Frame');
ylabel('Range (m)');
title('Range Trajectory');
legend('Location', 'eastoutside');

nexttile; hold on; grid on;
for k = 1:n_tracks
    id = track_ids(k);
    Tk = sortrows(Tv(Tv.track_id == id, :), 'frame');
    plot(Tk.frame, Tk.vel_mps, '-o', ...
        'Color', cmap(k, :), 'LineWidth', 1.2, 'MarkerSize', 4, ...
        'DisplayName', sprintf('ID %d', id));
end
xlabel('Frame');
ylabel('Velocity (m/s)');
title('Velocity Trajectory');

nexttile; hold on; grid on;
for k = 1:n_tracks
    id = track_ids(k);
    Tk = sortrows(Tv(Tv.track_id == id, :), 'frame');
    plot(Tk.frame, Tk.snr_db, '-o', ...
        'Color', cmap(k, :), 'LineWidth', 1.2, 'MarkerSize', 4, ...
        'DisplayName', sprintf('ID %d', id));
end
xlabel('Frame');
ylabel('SNR (dB)');
title('SNR Trajectory');

figure('Name', 'Range-Velocity Phase Plot', 'Color', 'w');
hold on; grid on;
for k = 1:n_tracks
    id = track_ids(k);
    Tk = sortrows(Tv(Tv.track_id == id, :), 'frame');
    plot(Tk.range_m, Tk.vel_mps, '-o', ...
        'Color', cmap(k, :), 'LineWidth', 1.2, 'MarkerSize', 4, ...
        'DisplayName', sprintf('ID %d', id));
    text(Tk.range_m(1), Tk.vel_mps(1), sprintf('  ID%d', id), ...
        'Color', cmap(k, :), 'FontSize', 9);
end
xlabel('Range (m)');
ylabel('Velocity (m/s)');
title('Range-Velocity Track');
legend('Location', 'best');

figure('Name', 'Bin Domain Overview', 'Color', 'w');
subplot(2,1,1);
scatter(Tv.frame, Tv.rbin, 24, Tv.track_id, 'filled');
grid on;
xlabel('Frame');
ylabel('Range Bin');
title('rbin vs frame');
cb = colorbar;
cb.Label.String = 'track\_id';

subplot(2,1,2);
scatter(Tv.frame, Tv.dbin_c, 24, Tv.track_id, 'filled');
grid on;
xlabel('Frame');
ylabel('Doppler Bin (centered)');
title('dbin\_c vs frame');
cb = colorbar;
cb.Label.String = 'track\_id';

figure('Name', 'Angle vs Frame', 'Color', 'w');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

% Angle trend subset (high-SNR only).
Ta = Tv(Tv.snr_db >= angle_min_snr_db, :);
if isempty(Ta)
    warning('No rows pass angle_min_snr_db=%.1f, fallback to all Tv.', angle_min_snr_db);
    Ta = Tv;
end

nexttile; hold on; grid on;
for k = 1:n_tracks
    id = track_ids(k);
    Tk = sortrows(Ta(Ta.track_id == id, :), 'frame');
    if isempty(Tk)
        continue;
    end
    az_sm = smoothdata(Tk.az_deg, 'movmean', angle_smooth_window);
    if show_raw_angle
        plot(Tk.frame, Tk.az_deg, '--o', ...
            'Color', cmap(k, :), 'LineWidth', 0.8, 'MarkerSize', 3, ...
            'HandleVisibility', 'off');
    end
    plot(Tk.frame, az_sm, '-o', ...
        'Color', cmap(k, :), 'LineWidth', 1.6, 'MarkerSize', 4, ...
        'DisplayName', sprintf('ID %d', id));
end
xlabel('Frame');
ylabel('Azimuth (deg)');
title('Azimuth Trajectory (high-SNR + smoothed)');
legend('Location', 'eastoutside');

nexttile; hold on; grid on;
for k = 1:n_tracks
    id = track_ids(k);
    Tk = sortrows(Ta(Ta.track_id == id, :), 'frame');
    if isempty(Tk)
        continue;
    end
    el_sm = smoothdata(Tk.el_deg, 'movmean', angle_smooth_window);
    if show_raw_angle
        plot(Tk.frame, Tk.el_deg, '--o', ...
            'Color', cmap(k, :), 'LineWidth', 0.8, 'MarkerSize', 3, ...
            'HandleVisibility', 'off');
    end
    plot(Tk.frame, el_sm, '-o', ...
        'Color', cmap(k, :), 'LineWidth', 1.6, 'MarkerSize', 4, ...
        'DisplayName', sprintf('ID %d', id));
end
xlabel('Frame');
ylabel('Elevation (deg)');
title('Elevation Trajectory (high-SNR + smoothed)');

figure('Name', 'Az-El Phase Plot (Scatter)', 'Color', 'w');
hold on; grid on;
all_frames = [];
for k = 1:n_tracks
    id = track_ids(k);
    Tk = sortrows(Ta(Ta.track_id == id, :), 'frame');
    if isempty(Tk)
        continue;
    end
    az_sm = smoothdata(Tk.az_deg, 'movmean', angle_smooth_window);
    el_sm = smoothdata(Tk.el_deg, 'movmean', angle_smooth_window);
    scatter(az_sm, el_sm, 34, Tk.frame, 'filled', ...
        'MarkerEdgeColor', cmap(k, :), ...
        'DisplayName', sprintf('ID %d', id));
    plot(az_sm, el_sm, '-', 'Color', cmap(k, :), ...
        'LineWidth', 0.8, 'HandleVisibility', 'off');
    plot(az_sm(1), el_sm(1), 's', 'Color', cmap(k, :), ...
        'MarkerSize', 6, 'HandleVisibility', 'off');
    plot(az_sm(end), el_sm(end), 'd', 'Color', cmap(k, :), ...
        'MarkerSize', 6, 'HandleVisibility', 'off');
    text(az_sm(1), el_sm(1), sprintf(' start ID%d', id), ...
        'Color', cmap(k, :), 'FontSize', 8);
    all_frames = [all_frames; Tk.frame]; %#ok<AGROW>
end
xlabel('Azimuth (deg)');
ylabel('Elevation (deg)');
title('Azimuth-Elevation Trend (color = frame index)');
if ~isempty(all_frames)
    caxis([min(all_frames), max(all_frames)]);
    cb = colorbar;
    cb.Label.String = 'frame';
end
legend('Location', 'best');
