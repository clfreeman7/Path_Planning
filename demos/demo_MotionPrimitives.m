
% Demonstrate the processing of visual tracking data of motion primitives
% after Euler tour (exhaustive exploration) experiments.
%
% Pre-requisite(s): 
%   TBD
%   


% [0] == Script setup
% Add dependencies to classpath
addpath('./');
clear; clc; close all


% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% Define experimental parameters. 
frameStartList = [206, 142, 260, 363, 316];

% [1] == Load raw data. 
for iTrial = 1:5
    exp1(iTrial).params.frame_1 = frameStartList(iTrial);
    if iTrial == 4
        exp1(iTrial).params.marker_order = [8 6 4 3 1 2 5 7];
    else
        exp1(iTrial).params.marker_order = [8 7 5 3 1 2 4 6];
    end
    filename = ['Euler', ' ', num2str(iTrial), '.mat'];
    exp1(iTrial).rawData = load(filename).all_pt;   % trial 1
end


load('stateOrder.mat')

% [2] == Instantiate experimental motion primitive data analysis
for i = 1:5
    alltrials(i) = offlineanalysis.MotionPrimitives(exp1(i).rawData, exp1(i).params, stateOrder(i,:));
end