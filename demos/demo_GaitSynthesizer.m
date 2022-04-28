% 
% Script: demo_GaitSynthesizer.m
%  
% Dependencies:
%   +demos/data/visualtracking/state_order_2.mat
%   +demos/data/visualtracking/Euler 7.mat
%   +demos/data/visualtracking/Euler 8.mat
%   +demos/data/visualtracking/Euler 9.mat
%   +demos/data/visualtracking/Euler 10.mat
%   +demos/data/visualtracking/Euler 11.mat
%
%   +offlineanalysis/PrimtivesTest
%   +offlineanalaysis/GaitSynthesizer
%
% Description: 
%   Demonstrate how to populate an instance of gait synthesis after
%   processing the motion primtives data. 
% 
% 

% [0] == Script setup
clear; clc
% Add dependencies to classpath
addpath('../');
load('data/visualtracking/state_order_2.mat')


% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Extract and define parameters for PrimitivesTest() objects.

% Define experimental parameters after investigating video. 
frame_start_list = [76, 287, 254, 405, 814];      % for orange robot

% Extract data.
for i = 1:5
    % Define robot.
    exp_2(i).params.robot_name = 'orange';
    exp_2(i).params.substrate = 'black mat';
    
    % Extract and store first frame information.
    exp_2(i).params.frame_1 = frame_start_list(i);
    
    % Extract and store raw data from each trial.
    filename = ['data/visualtracking/Euler', ' ', num2str(6 + i), '.mat'];
    exp_2(i).raw_data = load(filename).all_pt;  
end


% [2] == Instantiate PrimitivesTest() objects for each experimental trial.
% This analyzes the data from each trial to find motion primitive twist
% information.
figure(1)
tiledlayout(1, 5)
for i = 1:5
    all_trials(i) = offlineanalysis.PrimitivesTest(exp_2(i).raw_data, ...
                                                  exp_2(i).params, ...
                                                  state_order_2(i,:));
    nexttile;
    all_trials(i).plot;                                         
    delta_x_data(i, :) = all_trials(i).delta_x;
    delta_y_data(i, :) = all_trials(i).delta_y;
    delta_theta_data(i, :) = all_trials(i).delta_theta;
end

% Concatenate all motion primitive motion data into a 3D matrix.
motion_primitive_data = cat(3, delta_x_data, delta_y_data, delta_theta_data);

% [4] == Set up parameters for gait synthesis.
params_2.robot_name = 'orange';
params_2.substrate= 'black mat';

% [3] == Instantiate GaitSynthesizer() object to generate gaits from motion
% primitive data.
gait_synthesis = offlineanalysis.GaitSynthesizer( motion_primitive_data )
gaits = gait_synthesis.solutions




