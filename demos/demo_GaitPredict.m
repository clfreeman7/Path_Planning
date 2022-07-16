% 
% Script: demo_GaitPredict.m
%  
% Dependencies:
% Dependencies:
%   +demos/data/experiment_2_motion_primitives.mat
%
%   +offlineanalysis/GaitPredict
%   +offlineanalysis/Gait
%
% Description: 
%   Demonstrate how to create Gait objects from experimental motion primitive data.


% [0] == Script setup
clear; clc
% Add dependencies to classpath
addpath('../');
% Load motion primitive (x y theta) data.
load 'data/experiment_2_motion_primitives.mat'
% Define a gait.
gait_sequence = [16,7,5,11,14]; % Gait B

% Find the symmetric permutations.
[allPerms, nPerms] = listpermutations(gait_sequence, true, false)

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Define parameters for GaitPredict() objects.
 % Define robot.
exp_2.params.robot_name = 'orange';
exp_2.params.substrate = 'black mat';



% [2] == Instantiate GaitPredict() objects for each gait permutation.


% Instantiate objects for each gait permutation.
for i = 1:nPerms
    all_gaits(i) = offlineanalysis.GaitPredict(allPerms(i,:), ...
                                                 motion_primitive_data, ...   
                                                 exp_2.params);
end


% [3] == Instantiate gait objects for each gait permutation to build a gait
% library. 

figure
tiledlayout(1, nPerms)
for i = 1:nPerms
    gait_library_predict(i) = gaitdef.Gait(all_gaits(i), exp_2.params);
    nexttile;
    gait_library_predict(i).plot(10)
end


gait_library_predict
for ii = 1:length(gait_library_predict)
  gait_library_predict(ii).gait_name = char(65 + ii - 1);
end


