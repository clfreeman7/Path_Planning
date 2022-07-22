%===================Script: demo_offineTracking.m==========================
% 
% Author: Arun Mahendran Niddish anmahendran@crimson.ua.edu
%
% Dependencies: 
%   +offlineanalysis.rigid_transform_3D 
%   +offlineanalysis.createMaskCarpetBlue
%   +offlineanalysis.offinetracker
%
% Description: 
%    To track marker coordinates, get the rotation and translation matrix
%    in global and body frame for recorded video.
%
% Inputs:
% 1) Number of markers - options.numberofmarkers
% 2) Which frame to start from - options.startframe
% 3) Name of the video file to be tracked - inputfilename
% 4) Name of the tracked video output file - outputfilename
% 5) Visibility of the plot of output video - options.outputvideo
%
% =========================================================================
clear all;
clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures if you have the Image Processing Toolbox.
clear;  % Erase all existing variables. Or clearvars if you want.
workspace;  % Make sure the workspace panel is showing.
format long g;
format compact;

% inputfilename = 'Marker Focussed Videos\MTA_4\MTA4_Gait2_Carpet.mp4';  % Video to track
inputfilename = 'video_timestamp.mp4';
outputfilename = 'video_timestamp_gcf';
ts = load('video_timestamp.mat');
options.startframe = 1;
options.numberofmarkers = 8;
options.outputvideo = true;
tracking_data = offlinetracker(inputfilename,outputfilename,options);

tracking_data = [tracking_data ts.time_stamp];
