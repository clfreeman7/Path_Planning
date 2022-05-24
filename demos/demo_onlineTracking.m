%===================Script: demo_onlineTracking.m==========================
% 
% Author: Arun Mahendran Niddish anmahendran@crimson.ua.edu
%
% Dependencies: 
%   +onlineanalysis.rigid_transform_3D 
%   +onlineanalysis.createMaskCarpetBlue
%   +onlineanalysis.onlinetracker
%
% Description: 
%    To track marker coordinates, get the rotation and translation matrix
%    in global and body frame for live videos.
%
% Inputs:
% 1) Number of markers - options.numberofmarkers
% 2) File name of the actual video to be written - outputfilename_original 
% 3) File name of the tracked video to be written - outputfilename_tracked
% 4) Visibility of the plot of output video - options.outputvideo
% 5) Visibility of tracked video of markers - options.debug_code
% =========================================================================

clear all;
clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures if you have the Image Processing Toolbox.
clear;  % Erase all existing variables. Or clearvars if you want.
workspace;  % Make sure the workspace panel is showing.
format long g;
format compact;

outputfilename_original = 'onlineVideoTest';
outputfilename_tracked = 'Phase10_2_gcf';
options.numberofmarkers = 8;
options.outputvideo = true;
choose_type = menu("Select the mode",'Run normally','Debug mode');
if choose_type == 1
options.debug_code = false;
end
if choose_type == 2
options.debug_code = true;
end
tracking_data = onlinetracker(outputfilename_original,outputfilename_tracked,options);
