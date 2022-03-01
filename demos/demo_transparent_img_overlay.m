% 
% Demonstration script: 
%     Overlay transparent (reference) image over on-going image stream
% 

% [0] == Script parameter(s)
demo_mat_data = 'data/snakey_scenario_imgs.mat';  % demonstration data

OVERLAY_ALPHA = 0.4;    % alpha channel transparency for overlaid image


% [1] == Script setup
loadedData = load(demo_mat_data);

img_timestamps = loadedData.img_timestamps;   % image stream timestamps
img_seq = loadedData.img_seq;                 % image stream 
overlay_img = loadedData.overlay_img;         % overlay image 


% [2] == Setup visualization
figure;

% Original image stream
subplot(1, 2, 1);
hold on;
img_stream1_hdl = imagesc(zeros(1080, 1920, 3));
hold off;
axis equal;
xlabel('X (px)'); ylabel('Y (px)');
title1_hdl = title('Original Image Seq.');

% Image stream with transparent (reference) image overlaid
subplot(1, 2, 2);
hold on;
img_stream2_hdl = imagesc(zeros(1080, 1920, 3));
ovly_img_hdl = imagesc(overlay_img);              % overlay (reference) image
set(ovly_img_hdl, 'AlphaData', OVERLAY_ALPHA);    % set overlaid image alpha
hold off;
axis equal;
xlabel('X (px)'); ylabel('Y (px)');
title2_hdl = title('With Overlay');


% [3] == 'Play' image stream
for ii = 1:size(img_seq, 4)
  set(img_stream1_hdl, 'CData', img_seq(:, :, :, ii));
  set(img_stream2_hdl, 'CData', img_seq(:, :, :, ii));

  title1_hdl.String = sprintf('Original Image Seq. (%.2f s)', img_timestamps(ii));
  title2_hdl.String = sprintf('With Overlay (%.2f s)', img_timestamps(ii));
  pause(0.5);
end


