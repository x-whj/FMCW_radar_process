clc; clear; close all;

% Fit az/el calibration from raw amplitude-comparison error:
%   az_err -> az_truth_deg
%   el_err -> el_truth_deg
%
% Required inputs:
% 1) offline_multi_track.csv with columns:
%    frame, track_id, confirmed, az_err, el_err
% 2) angle_truth.csv with columns:
%    frame, track_id, az_truth_deg, el_truth_deg

track_csv_candidates = {
    'offline_multi_track.csv'
    fullfile('build', 'offline_multi_track.csv')
    fullfile('..', 'build', 'offline_multi_track.csv')
};

truth_csv_candidates = {
    'angle_truth.csv'
    fullfile('matlab', 'angle_truth.csv')
    fullfile('..', 'matlab', 'angle_truth.csv')
};

track_csv = pick_existing_file(track_csv_candidates);
if isempty(track_csv)
    error('offline_multi_track.csv not found. Run radar_app first.');
end

truth_csv = pick_existing_file(truth_csv_candidates);

T = readtable(track_csv);
required_track_cols = {'frame', 'track_id', 'confirmed', 'az_err', 'el_err'};
assert_has_columns(T, required_track_cols, track_csv);

% Use confirmed track rows only.
T = T((T.track_id ~= -1) & (T.confirmed == 1), :);
if isempty(T)
    error('No confirmed track rows in %s', track_csv);
end

% Optional short-track filtering to remove unstable IDs.
min_track_rows = 6;
track_ids = unique(T.track_id);
keep = false(size(track_ids));
for i = 1:numel(track_ids)
    keep(i) = sum(T.track_id == track_ids(i)) >= min_track_rows;
end
T = T(ismember(T.track_id, track_ids(keep)), :);
if isempty(T)
    error('No rows left after min_track_rows=%d filtering.', min_track_rows);
end

if isempty(truth_csv)
    % Create a template that user can fill with truth angles.
    U = unique(T(:, {'frame', 'track_id'}));
    U.az_truth_deg = nan(height(U), 1);
    U.el_truth_deg = nan(height(U), 1);
    template_out = 'angle_truth_template.csv';
    writetable(U, template_out);
    fprintf('truth file not found.\n');
    fprintf('Template generated: %s\n', template_out);
    fprintf('Fill az_truth_deg/el_truth_deg then rename to angle_truth.csv and rerun.\n');
    return;
end

G = readtable(truth_csv);
required_truth_cols = {'frame', 'track_id', 'az_truth_deg', 'el_truth_deg'};
assert_has_columns(G, required_truth_cols, truth_csv);

J = innerjoin(T, G, 'Keys', {'frame', 'track_id'});
if isempty(J)
    error('No matched rows between %s and %s (keys: frame, track_id).', track_csv, truth_csv);
end

fprintf('Track CSV: %s\n', track_csv);
fprintf('Truth CSV: %s\n', truth_csv);
fprintf('Matched rows: %d\n', height(J));

lut_bins = 81; % odd number recommended

res_az = fit_axis_calibration(J.az_err, J.az_truth_deg, 'Azimuth', lut_bins);
res_el = fit_axis_calibration(J.el_err, J.el_truth_deg, 'Elevation', lut_bins);

fprintf('\n=== Linear fit (theta = k*e + b) ===\n');
fprintf('AZ: k = %.6f, b = %.6f, RMSE = %.4f deg\n', res_az.k, res_az.b, res_az.rmse_deg);
fprintf('EL: k = %.6f, b = %.6f, RMSE = %.4f deg\n', res_el.k, res_el.b, res_el.rmse_deg);

% Export LUT as one CSV.
az_axis = repmat({'az'}, numel(res_az.lut_err), 1);
el_axis = repmat({'el'}, numel(res_el.lut_err), 1);
lut_tbl = [ ...
    table(az_axis, res_az.lut_err, res_az.lut_angle_deg, ...
        'VariableNames', {'axis', 'error', 'angle_deg'}); ...
    table(el_axis, res_el.lut_err, res_el.lut_angle_deg, ...
        'VariableNames', {'axis', 'error', 'angle_deg'})];
writetable(lut_tbl, 'angle_calib_lut.csv');

save('angle_calib_result.mat', 'res_az', 'res_el', 'lut_tbl');
fprintf('Saved: angle_calib_lut.csv\n');
fprintf('Saved: angle_calib_result.mat\n');

function out = fit_axis_calibration(err, truth_deg, axis_name, lut_bins)
    v = isfinite(err) & isfinite(truth_deg);
    err = err(v);
    truth_deg = truth_deg(v);
    if numel(err) < 3
        error('%s: not enough valid samples.', axis_name);
    end

    p = polyfit(err, truth_deg, 1);
    pred = polyval(p, err);
    rmse = sqrt(mean((pred - truth_deg).^2));

    % Build LUT in normalized error domain [-1, 1].
    edges = linspace(-1, 1, lut_bins + 1);
    centers = 0.5 * (edges(1:end-1) + edges(2:end));
    idx = discretize(err, edges);
    lut_angle = nan(size(centers));
    for i = 1:numel(centers)
        m = (idx == i);
        if any(m)
            lut_angle(i) = mean(truth_deg(m));
        end
    end

    valid = isfinite(lut_angle);
    if nnz(valid) < 2
        % Fallback to linear fit.
        lut_angle = polyval(p, centers);
    else
        lut_angle = interp1(centers(valid), lut_angle(valid), centers, 'linear', 'extrap');
    end
    lut_angle = smoothdata(lut_angle, 'movmean', 5);

    figure('Name', sprintf('%s Calibration', axis_name), 'Color', 'w');
    tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile; hold on; grid on;
    scatter(err, truth_deg, 12, 'filled', 'MarkerFaceAlpha', 0.5);
    xline(0, '--k');
    x_fit = linspace(min(err), max(err), 200);
    y_fit = polyval(p, x_fit);
    plot(x_fit, y_fit, 'r-', 'LineWidth', 1.4);
    xlabel('Raw Error');
    ylabel('Truth Angle (deg)');
    title(sprintf('%s Linear Fit', axis_name));
    legend({'Samples', 'e=0', 'Linear Fit'}, 'Location', 'best');

    nexttile; hold on; grid on;
    plot(centers, lut_angle, 'b-', 'LineWidth', 1.4);
    xlabel('Raw Error');
    ylabel('Angle (deg)');
    title(sprintf('%s LUT', axis_name));

    out = struct();
    out.k = p(1);
    out.b = p(2);
    out.rmse_deg = rmse;
    out.lut_err = centers(:);
    out.lut_angle_deg = lut_angle(:);
end

function path = pick_existing_file(candidates)
    path = '';
    for i = 1:numel(candidates)
        if isfile(candidates{i})
            path = candidates{i};
            return;
        end
    end
end

function assert_has_columns(T, cols, source_name)
    names = T.Properties.VariableNames;
    miss = cols(~ismember(cols, names));
    if ~isempty(miss)
        error('Missing column(s) in %s: %s', source_name, strjoin(miss, ', '));
    end
end
