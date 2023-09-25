% This is a function to setup the camera properties to capture markers.

function cam = camera_properties_marker_1920_Blue(cam)

%% Properties
cam.Resolution = cam.AvailableResolutions{6}; %Resolution - {1}-640 x 480 {6}-1920x1080
cam.BacklightCompensation = 8;  
cam.Brightness = 75;
cam.Contrast = 31;
cam.ExposureMode = 'auto';
cam.Exposure = -4;
cam.Gain = 16;
cam.Gamma = 140;
cam.Hue = 0;
cam.Iris = 0;
cam.Saturation = 100;
cam.Sharpness = 5;
cam.WhiteBalanceMode = 'auto';
cam.WhiteBalance = 4600;

end