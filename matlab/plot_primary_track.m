clc; clear; close all;

csv_path = fullfile('build', 'offline_primary_track.csv');
if ~isfile(csv_path)
    error('File not found: %s. Run radar_app first.', csv_path);
end

T = readtable(csv_path);

figure('Name', 'Primary Track');

subplot(2,1,1);
plot(T.frame, T.rbin, 'o-', 'LineWidth', 1.2);
grid on;
xlabel('frame');
ylabel('rbin');
title('Primary Range Bin Track');

subplot(2,1,2);
plot(T.frame, T.dbin_c, 'o-', 'LineWidth', 1.2);
grid on;
xlabel('frame');
ylabel('dbin\_c');
title('Primary Centered Doppler Bin Track');

valid_idx = T.valid == 1;
fprintf('Total frames: %d\n', height(T));
fprintf('Valid primary frames: %d\n', nnz(valid_idx));
fprintf('Missing primary frames: %d\n', nnz(~valid_idx));
