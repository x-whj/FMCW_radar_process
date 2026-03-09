clc;clear;
close all;

%% 读取二进制文件
fid = fopen(fullfile('build', 'output_cha2.dat'), 'rb');
if fid < 0
    error('Failed to open build/output_cha2.dat');
end
data = fread(fid, 'float');
fclose(fid);

st_tt = data(1:2:end) + 1i * data(2:2:end);
a = size(st_tt);

rows = 256;
% 344  664
cols = 664;

data = single(reshape(st_tt,[cols,a(1)/cols])).';

%% 加载mat文件
filename_grf = 'radar_echo.gif';   % 输出 GIF 文件名

%%
snr_all = [];
for m = 1:a(1)/cols/rows-1

data1 = data(1+m*rows:rows+m*rows, 1:cols);

mtd_coeff = chebwin(rows);
echo_s1 = fftshift(fft(data1.*mtd_coeff, rows,1),1);

% 计算噪声功率
noise=0;
temp=0;
echo_s1_abs = abs(echo_s1);

sampleNum = cols;
chirpNum = 128;

for i=fix(chirpNum/2)-20:fix(chirpNum/2)+20
    temp=0;
    for j=fix(sampleNum/2)-50:fix(sampleNum/2)+49
        temp=temp+(echo_s1_abs(i,j));
    end
    noise=noise+temp/41;
end
noies=noise/length(sampleNum/2-50:sampleNum/2+49);
noies_db=db(noies);
snr_all = [snr_all; noies_db]; %#ok<AGROW>

figure(2)
mesh(echo_s1_abs)
view(0,90)
% zlim([0 3000])

% % === 将帧保存为 GIF ===
% frame = getframe(gcf);             % 抓取当前图像帧
% im = frame2im(frame);              % 转为图像
% [A,map] = rgb2ind(im,256);         % 转为索引图
% if m == 1
%     imwrite(A,map,filename_grf,'gif','LoopCount',Inf,'DelayTime',0.2); % 首帧
% else
%     imwrite(A,map,filename_grf,'gif','WriteMode','append','DelayTime',0.2);
% end
% unwrap(angle(slow_time))
pause(0.15)

end
