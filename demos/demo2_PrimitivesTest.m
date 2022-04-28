% 
% Script: demo2_PrimitivesTest.m
% 
% Dependencies:
%   +demos/data/visualtracking/state_order_2.mat
%   +demos/data/visualtracking/Euler 7.mat
%   +demos/data/visualtracking/Euler 8.mat
%   +demos/data/visualtracking/Euler 9.mat
%   +demos/data/visualtracking/Euler 10.mat
%   +demos/data/visualtracking/Euler 11.mat
%   +offlineanalysis/PrimtivesTest
%
% Future Dependencies:
%   +offlineanalaysis/GaitSynthesizer
%
% Description:
%
% Demonstrate the processing of visual tracking data of motion primitives
% after Euler tour (exhaustive exploration) experiments.
%
% This data corresponds to experiment 2 with the orange robot with the new 
% frictional component.
%   
%   


clear; clc; close all

% [0] == Script setup
% Add dependencies to classpath
addpath('../');
load('data/visualtracking/state_order_2.mat')

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Define experimental parameters after investigating video. 
frame_start_list = [76, 286, 257, 404, 808];

% Find marker order by investigating first frame.


for i = 1:5
    % Define robot.
    exp_2(i).params.robot_name = 'orange';
    exp_2(i).params.substrate = 'black mat';
    
    % Define pixel length.
    exp_2(i).params.pixel_length = .23898;
    
    % Extract and store first frame information.
    exp_2(i).params.frame_1 = frame_start_list(i);
    
    % Extract and store raw data from each trial.
    filename = ['data/visualtracking/Euler', ' ', num2str(i+6), '.mat'];
    exp_2(i).raw_data = load(filename).all_pt;  
end


% [2] == Instantiate experimental motion primitive data analysis and plot
% results.

% Set up figure. 
figure(1)
tiledlayout(1, 5);

for i = 1:5
    all_trials(i) = offlineanalysis.PrimitivesTest(exp_2(i).raw_data, exp_2(i).params, state_order_2(i,:));
    nexttile;
    all_trials(i).plot;
    
    % Extract data to store in a more convenient way for gait synthesis.
    delta_x_data(i, :) = all_trials(i).delta_x;
    delta_y_data(i, :) = all_trials(i).delta_y;
    delta_theta_data(i, :) = all_trials(i).delta_theta;
end

% [3] == Concatenate all motion primitive motion data into a 3D matrix.
motion_primitive_data = cat(3, delta_x_data, delta_y_data, delta_theta_data);


