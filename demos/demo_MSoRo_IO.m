% 
% Script: demo_MSoRo_IO.m
%  
% Dependencies:
% Dependencies:
%   +demos/data/gait_library_2.mat
%
%   +msoro/+api.MSoRo_IO.m
%
% Description: 
%   Demonstrate how to create use experimental data for open-loop paths. 

% [0] == Script setup
clear; clc
% Add dependencies to classpath
addpath('../');
% Load the gait library data. 
load('data/gait_library_2.mat')

% [1] == Extract and define gait data.
n_gaits = length(gait_library_2);
% Extract the motion data and gait names from the gait library
gait_trans_seq = cell(1, n_gaits);
gait_names = cell(1, n_gaits);

for i = 1:n_gaits
    gait_trans_seq{i} = gait_library_2(i).robo_states;
    gait_names{i} = gait_library_2(i).gait_name;
end

gait_trans_seq
gait_names

% [2] == Define the open-loop canned sequence to be run. 
canned_sequence = 'A7B3D2C4E5';
