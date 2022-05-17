clear all;
clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures if you have the Image Processing Toolbox.
clear;  % Erase all existing variables. Or clearvars if you want.
workspace;  % Make sure the workspace panel is showing.
format long g;
format compact;


% Initialize
% Inputs:
% 1) Number of markers - number_of_markers
% 2) Which frame to start from - start_frame
% 3) Name of the video file to be tracked - baseFileName

    inputfilename = 'Marker Focussed Videos\Euler Tours\Euler 7.mp4';  % Video to track
    outputfilename = 'Phase10_2_gcf';
    options.startframe = 1;
    options.numberofmarkers = 8;
    options.outputvideo = true;
    tracking_data = offlinetracker(inputfilename,outputfilename,options);