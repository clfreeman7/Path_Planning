% 
% Script: demo_PrimitivesTest.m
% 
% Dependencies:
%   +demos/data/visualtracking/state_order.mat
%   +demos/data/visualtracking/Euler 1.mat
%   +demos/data/visualtracking/Euler 2.mat
%   +demos/data/visualtracking/Euler 3.mat
%   +demos/data/visualtracking/Euler 4.mat
%   +demos/data/visualtracking/Euler 5.mat
%   +offlineanalysis/PrimtivesTest
%
%
% Description:
%
% Demonstrate the processing of visual tracking data of motion primitives
% after Euler tour (exhaustive exploration) experiments.
%
% This data corresponds to experiment 1 with the blue robot with the old 
% frictional component.
%   


clear; clc; close all

% [0] == Script setup
% Add dependencies to classpath
addpath('../');
load('data/visualtracking/state_order.mat')

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Define experimental parameters after investigating video. 
frame_start_list = [206, 142, 260, 363, 316];

for i = 1:5
    % Define robot.
    exp_1(i).params.robot_name = 'blue';   
    exp_1(i).params.substrate = 'black mat';
    
    % Extract and store first frame information.
    exp_1(i).params.frame_1 = frame_start_list(i);
    
    % Extract and store raw data from each trial.
    filename = ['data/visualtracking/Euler', ' ', num2str(i), '.mat'];
    exp_1(i).raw_data = load(filename).all_pt;     
end


% [2] == Instantiate experimental motion primitive data analysis and plot
% results.

% Set up figure. 
figure(1)
tiledlayout(1, 5);

for i = 1:5
    alltrials(i) = offlineanalysis.PrimitivesTest(exp_1(i).raw_data, exp_1(i).params, state_order(i,:));
    nexttile;
    alltrials(i).plot;
    
    % Extract data to store in a more convenient way for gait synthesis.
    delta_x_data(i, :) = alltrials(i).delta_x;
    delta_y_data(i, :) = alltrials(i).delta_y;
    delta_theta_data(i, :) = alltrials(i).delta_theta;
end

% [3] == Concatenate all motion primitive motion data into a 3D matrix.
motion_primitive_data = cat(3, delta_x_data, delta_y_data, delta_theta_data);

