% 
% Script: demo2_PrimitivesTest.m
% 
% Dependencies:
%   +demos/data/visualtracking/state_order_2.mat
%   +demos/data/visualtracking/Euler 7_corrected.mat
%   +demos/data/visualtracking/Euler 8_corrected.mat
%   +demos/data/visualtracking/Euler 9_corrected.mat
%   +demos/data/visualtracking/Euler 10_corrected.mat
%   +demos/data/visualtracking/Euler 11_corrected.mat
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


clear; clc; close all;

% [0] == Script setup
% Add dependencies to classpath
addpath('../');
load('data/visualtracking/state_order_carpet_MTA3.mat')
% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Define experimental parameters after investigating video. 
frame_start_list = [118,131,156,248,269,197,220,228,262,237];




for i = 1:10
    % Define robot.
    exp_3(i).params.robot_name = 'MTA3';
    exp_3(i).params.substrate = 'carpet';
    exp_3(i).params.substrate = 'carpet';
    exp_3(i).params.n_unique_states = 8;
    % Define pixel length.
    exp_3(i).params.pixel_length = 1/8.6343;
    exp_3(i).params.n_markers = 6;
    exp_3(i).params.transition_time= .55;

    
    % Extract and store first frame information.
    exp_3(i).params.frame_1 = frame_start_list(i);
    % Extract and store raw data from each trial.
    filename = ['data/visualtracking/MTA_3_Euler_', num2str(i+5), '.mat'];
    exp_3(i).raw_data = load(filename).tracking_data;  
end


% [2] == Instantiate experimental motion primitive data analysis and plot
% results.

% Set up figure. 
figure(1)
t = tiledlayout(2, 5);

for i = 1:10
    all_trials(i) = offlineanalysis.PrimitivesTest(exp_3(i).raw_data, exp_3(i).params, state_order_carpet_MTA3(i,:));
    nexttile;
    all_trials(i).plot;
    title("Trial " +num2str(i))
    % Extract data to store in a more convenient way for gait synthesis.
    delta_x_data(i, :) = all_trials(i).delta_x;
    delta_y_data(i, :) = all_trials(i).delta_y;
    delta_theta_data(i, :) = all_trials(i).delta_theta;
end
% Add legend.
lgd = legend('Continuous robot position', 'Actual keyframe positions', ...
                  'Keyframe positions reconstructed from gait twists');
lgd.Layout.Tile = 'north';
lgd.Orientation = 'horizontal';

% Add colorbar.
a = colorbar;
a.Label.String = 'Number of motion primitives executed';
a.Layout.Tile = 'east';

title(t, 'Motion primitive exploration (Euler tours) for orange robot on black mat','FontSize',24)


% [3] == Concatenate all motion primitive motion data into a 3D matrix.
motion_primitive_data = cat(3, delta_x_data, delta_y_data, delta_theta_data);
% [4] == Set up parameters for gait synthesis.
params_3.robot_name = 'MTA3';
params_3.substrate= 'carpet';
params_3.n_unique_states = 8;

% [3] == Instantiate GaitSynthesizer() object to generate gaits from motion
% primitive data.
gait_synthesis = offlineanalysis.GaitSynthesizer( motion_primitive_data , params_3)
gaits = gait_synthesis.solutions

save data/MTA3_motion_primitives_carpet.mat motion_primitive_data


