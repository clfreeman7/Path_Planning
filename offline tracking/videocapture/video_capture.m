clear all;
clc;
% Camera for tracking
cam = webcam(2);  %Initializing object for camera
cam.Resolution = cam.AvailableResolutions{1};   %Resolution 640x480

%Camera for good video
cam2nd = webcam(1);
cam2nd.Resolution = cam2nd.AvailableResolutions{6}; %Resolution 1920x1080

vwrite = VideoWriter('Phase3_2','MPEG-4');
vwrite_2ndcam = VideoWriter('Phase3_2_Real','MPEG-4');

open(vwrite);
open(vwrite_2ndcam);

preview(cam);
preview(cam2nd);

pause(4);

%% Properties

cam = camera_properties_marker(cam);
pause(4);

% newim = cam.snapshot;
% imwrite(newim,'calib_xaxis.png');

%% Loop to capture the video

while true
    newim = cam.snapshot;
    writeVideo(vwrite,newim);
    
    newim_2ndcam = cam2nd.snapshot;
    writeVideo(vwrite_2ndcam,newim_2ndcam);
end
close(vwrite);
close(vwrite_2ndcam);