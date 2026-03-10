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

% Keep only valid confirmed track rows.
valid = (T.track_id ~= -1) & (T.confirmed == 1);
Tv = T(valid, :);

if isempty(Tv)
    error('No confirmed tracks found in %s', csv_path);
end

% Drop short-lived tracks (can be adjusted).
min_track_rows = 6;
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
