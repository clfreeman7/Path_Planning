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
addpath('../')
addpath('./ivaMatlibs')
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
    % Make PrimitivesTest object using the raw experimental data (i.e., 
    % analyze the data to calculate the motion primitive data).
    alltrials(i) = offlineanalysis.PrimitivesTest(exp_1(i).raw_data, exp_1(i).params, state_order(i,:));
    nexttile;

    % Plot the raw data (continuous motion and keyframes).
    alltrials(i).plot;
    % Initialize pose_check variable with starting position and
    % orientation.
    pose_check(:, 1) = alltrials(i).poses(:, alltrials(i).keyframes(2)) - alltrials(i).poses(:,1);
    % Convert to SE(2) object (from ivaMatlibs group class SE2).
    g_0 = SE2(pose_check(1:2,1), pose_check(3,1));
    g_traj = cell(1,alltrials(i).n_unique_states*(alltrials(i).n_unique_states-1)+1);
    g_traj{1} = g_0;
    % For each motion primitive:
    for j = 1:alltrials(i).n_unique_states*(alltrials(i).n_unique_states-1)
          % Reorder for correct experimental primitive order.
          k = alltrials(i).primitive_labels(j);
          % Perform successive Lie group multiplication for each MP.
          g_i = SE2([alltrials(i).delta_x(k); alltrials(i).delta_y(k)], alltrials(i).delta_theta(k));
          g_traj{j+1} = g_traj{j}*g_i;
          clear g_i;
          % Extract pose.
          pose_check(1:2,j+1) = g_traj{j+1}.getTranslation();
          pose_check(3,j+1) = g_traj{j+1}.getAngle();
    end
    sz = 30;
    c1 = linspace(1,10,length(pose_check));
    c2 = [0.6350 0.0780 0.1840];
    scatter(pose_check(1, :), pose_check(2, :),sz, c2)

    % Extract data to store in a more convenient way for gait synthesis.
    delta_x_data(i, :) = alltrials(i).delta_x;
    delta_y_data(i, :) = alltrials(i).delta_y;
    delta_theta_data(i, :) = alltrials(i).delta_theta;
end
title('Verification of Motion Primitive Calculations')
                %legend('Continuous Robot Position', 'Actual Keyframe Positions', ...
                   %'Keyframe Positions Reconstructed from Motion Primitives in SE(2)')
% [3] == Concatenate all motion primitive motion data into a 3D matrix.
motion_primitive_data = cat(3, delta_x_data, delta_y_data, delta_theta_data);

