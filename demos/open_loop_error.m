% 
% Script: predicted_vs_actual.m
%  
% Dependencies:
%   +demos/data/experiment_2_motion_primitives_corrected.mat
%   +demos/data/NS_motion_primitives.mat
%   +demos/data/exp_S.mat
%   +demos/data/exp_NS.mat
%   +demos/data/gait_library_2_corrected.mat
%   +demos/data/gait_library_S.mat
%   +demos/data/gait_library_NS.mat
%
%   +offlineanalysis/GaitPredict
%   +offlineanalysis/Gait
%
% Description: 
%   Compare experimental and predicted gait trajectories / motion
%   primitives.

% [0] == Script setup
clear; clc; close all
% Add dependencies to classpath
addpath('../');
% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
set(0,'defaultAxesFontSize',16)

% Define gait robot states.
gait_B = [16,7,5,11,14];

% Load motion primitive (x y theta) data.
mps_H = load('data/experiment_2_motion_primitives_corrected.mat').motion_primitive_data;
mps_NS = load('data/NS_motion_primitives.mat').motion_primitive_data;

% Load experiment gait data. 
exp_S = load('data/exp_S.mat').exp_S;
exp_NS = load('data/exp_NS.mat').exp_NS;

% Load experimental gait library data. 
gait_library_H = load('data/gait_library_2_corrected.mat').gait_library_2;
gait_library_S = load('data/gait_library_S.mat').gait_library_S;
gait_library_NS = load('data/gait_library_NS.mat').gait_library_NS;


%% [1] == Instantiate GaitPredict() objects for each gait permutation.
% Define parameters.
exp.params.robot_name = 'orange';
exp.params.substrate = 'black mat';

% Calculate heavy sheath (H) tether predicted gait motions.
predicted_H = offlineanalysis.GaitPredict(gait_B, mps_H, exp.params);

% Calculate no sheath (NS) tether predicted gait motions.
predicted_NS = offlineanalysis.GaitPredict(gait_B, mps_NS, exp.params);

% Average the experimental motions for heavy sheath. 
for i = 1:6
    delta_poses_S((i-1)*59+1:i*59, :, :) = cat(3, exp_S(i).delta_x, exp_S(i).delta_y, exp_S(i).delta_theta);
end

gait_mps_S = zeros(length(delta_poses_S), 240,3);
gait_mps_S(:,exp_S(1).primitive_labels, :) = delta_poses_S;

averaged_S = offlineanalysis.GaitPredict(gait_B, gait_mps_S, exp.params);

for i = 1:6
    delta_poses_NS((i-1)*59+1:i*59, :, :) = cat(3, exp_NS(i).delta_x, exp_NS(i).delta_y, exp_NS(i).delta_theta);
end

gait_mps_NS = zeros(length(delta_poses_NS), 240,3);
gait_mps_NS(:,exp_NS(1).primitive_labels, :) = delta_poses_NS;

% Calculate no sheath (NS) tether averaged gait motions.
averaged_NS = offlineanalysis.GaitPredict(gait_B, gait_mps_NS, exp.params);
figure(1)
averaged_poses(:,:) = averaged_NS.plot(59);
predicted_poses(:,:) = predicted_NS.plot(59);
color_array = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30", "#4DBEEE"];
figure(1)
clf;
title('Error in robot position for no sheath experiments', 'FontSize', 24)
timestamps = 0:.45:295*.45;
for i = 1:6
    hold on
    diff = ((exp_NS(i).poses(1,exp_NS(i).keyframes(2:end)) - ...
            averaged_poses(1,:)).^2+ ...
            (exp_NS(i).poses(2,exp_NS(i).keyframes(2:end)) - ...
            averaged_poses(2,:)).^2).^.5;
    plot(timestamps, abs(diff - diff(1)), 'Color', color_array(i))
    xlim([0 timestamps(end)])
    lgd_str{i} = ['Experiment ', num2str(i)];
end

grid on
xlabel('Time (s)')
ylabel('Euclidean distance b/w predicted and experimental paths (cm)')
legend(lgd_str)


figure(2)
ang(1,:) = rad2deg(averaged_poses(3,:));
plot(timestamps, ang, 'k-', 'LineWidth', 2)
lgd_str{1} = 'Predicted';
for i = 1:6
    hold on
    ang(i+1,:) = exp_NS(i).poses(3,exp_NS(i).keyframes(2:end));
    ang(i+1,:) = rad2deg(ang(i+1,:)-ang(i+1,1));
    ang(i+1, ang(i+1,:)<-100) = ang(i+1,ang(i+1,:)<-100)+360;
    ang(i+1, timestamps>60 & ang(i+1,:)<200) = ang(i+1, timestamps>60 & ang(i+1,:)<200)+360;
    plot(timestamps, ang(i+1, :), 'Color', color_array(i))
    lgd_str{i+1} = ['Experiment ', num2str(i)];
end
grid on
xlim([0 timestamps(end)])
xlabel('Time (s)')
ylabel('Global robot orientation (degrees)')
lgd = legend(lgd_str);
lgd.Orientation = 'horizontal';
lgd.Location = 'north';
title('Robot orientation over time for no sheath experiments', 'FontSize', 24)
axis tight 

figure(3)
title('Error in robot orientation for no sheath experiments', 'FontSize', 24)
hold on
for i = 1:6
    plot(timestamps, ang(i+1,:) - ang(1,:), 'Color', color_array(i))
    lgd_str{i} = ['Experiment ', num2str(i)];
end
grid on
xlim([0 timestamps(end)])
xlabel('Time (s)')
ylabel('Robot orientation error $\theta_{exp} - \theta_{pred}$ (deg)')
lgd = legend(lgd_str);
lgd.Orientation = 'horizontal';
lgd.Location = 'north';



