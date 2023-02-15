% MTA3 Gait Experiments Data Plots for Three Surfaces
%
% Script: gait_analysis_MTA3.m
%
% Dependencies:
%   gait-analysis_MTA3.mat
%   
% Description:
%  This script plots the pre-analyzed data for the MTA3 gait experiments. 
% If you open this file in an environment with the proper
% gaitdef.Gait and offlineanalysis.GaitTest class definitions, you can use
% the Gait() and GaitTest() objects. You can also use the associated class
% methods like plot(). If not, you should use the GaitTest_struct and 
% gaitdef_Gait_struct structure arrays. 

%% [0] == Script setup
clear; clc; close all

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
set(0, 'DefaultAxesFontSize', 18);

% Decide which figures to show.
show_gait_sequences = true;      % Visualizes the discrete robot configurations in each gait sequence.

%% [1] == Extract and define parameters for each gait experiment.
