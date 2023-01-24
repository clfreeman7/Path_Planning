% 
% Script: demo_PathTest.m
%
% Description: 
%   Demonstrate how to process the raw experimental data from the visual
%   tracking of paths (combinations of gaits).


% [0] == Script setup
clear; clc; close all
% Add dependencies to classpath
addpath('../');
addpath('data/visualtracking/')
% 
% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Load data for the path tests and the gait library.
paths{1} = 'B60_G60_32SR_NF_L_1.mat';
path_sequence.gait_names = ['B' 'G']; 
path_sequence.gait_durations = [60 60];
gait_library = load('data/gait_library_4.mat').gait_library_4;
raw_data = load(paths{1}).tracking_data; 

% [2] == Extract and define parameters for PathTest() objects.

% Define experimental parameters after investigating video. 
frame_start_list = [37,2505];      % for orange robot

% Find marker order by investigating first frame.
% Define robot.
path_test.params.robot_name = 'orange';
path_test.params.substrate = 'black mat';
path_test.params.n_markers = 4;
path_test.params.frame_1 = frame_start_list;


path_1 = offlineanalysis.PathTest( raw_data, path_sequence, gait_library,  path_test.params)
path_1.plot
