clc; clear; close all;

% Quick RD viewer for a single channel binary file.
% Data format: interleaved float32 IQ => I0,Q0,I1,Q1,...

dat_path = fullfile('build', 'output_cha2.dat'); % change to output_he.dat / output_cha1.dat if needed
if ~isfile(dat_path)
    error('File not found: %s', dat_path);
end

num_chirps = 256;
num_samples = 664;

fid = fopen(dat_path, 'rb');
raw = fread(fid, 'float32=>single');
fclose(fid);

if mod(numel(raw), 2) ~= 0
    error('IQ stream length is not even.');
end

iq = raw(1:2:end) + 1i * raw(2:2:end);
num_complex = numel(iq);
frame_size = num_chirps * num_samples;
num_frames = floor(num_complex / frame_size);
if num_frames < 1
    error('No complete frame found.');
end

iq = iq(1:num_frames * frame_size);
cube = reshape(iq, [num_samples, num_chirps, num_frames]); % sample x chirp x frame

win = chebwin(num_chirps);

figure('Name', 'RD Quicklook');
for k = 1:num_frames
    frame = squeeze(cube(:,:,k)).'; % chirp x sample
    rd = fftshift(fft(frame .* win, num_chirps, 1), 1);
    p = abs(rd);

    imagesc(p);
    axis xy;
    colormap turbo;
    colorbar;
    title(sprintf('Frame %d / %d', k-1, num_frames-1));
    xlabel('Range Bin');
    ylabel('Doppler Bin (shifted)');
    drawnow;
    pause(0.08);
end
