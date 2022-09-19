% 
% Script: demo_GaitPredict.m
%  
% Dependencies:
%   +demos/data/experiment_2_motion_primitives.mat
%
%   +offlineanalysis/GaitPredict
%   +offlineanalysis/Gait
%
% Description: 
%   Demonstrate how to create Gait objects from experimental motion primitive data.


% [0] == Script setup
clear; clc; close all
% Add dependencies to classpath
addpath('../');
% Load motion primitive (x y theta) data.
load 'data/experiment_2_motion_primitives_corrected.mat'
% Define a gait.
gait_sequence = [16,7,5,11,14]; 
gait_name = "Gait B";

% Find the symmetric permutations.
[allPerms, nPerms] = listpermutations(gait_sequence, true, false)

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
set(0,'defaultAxesFontSize',20)

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
t=tiledlayout(2, 2);
t.TileSpacing = 'compact';
t.Padding = 'compact';
for i = 1:nPerms
    gait_library_predict(i) = gaitdef.Gait(all_gaits(i), exp_2.params);
    h(i) = nexttile;
    gait_library_predict(i).plot(10)
    xlim([-15 20])
    ylim([-2 15])
    xticks(-15:5:20);
    yticks(0:5:15);
    daspect([1 1 1]);
    grid on
end
a = colorbar;
a.Label.String = 'Number of Gaits Executed';
a.Layout.Tile = 'south';

save  data/gait_library_predict_S gait_library_predict
% for ii = 1:length(gait_library_predict)
%   gait_library_predict(ii).gait_name = char(65 + ii - 1);
% end

% [4] == Plot the predicted behavior with variance for each motion
% primitive of the base gait. 

motion_primitive_data = motion_primitive_data(:,gait_sequence,:);
% Find mean, variance, and standard deviation of data.
avg_motions = squeeze(mean(motion_primitive_data, 1))';
avg_motions(3,:) = rad2deg(avg_motions(3,:));
var_motions = squeeze(var(motion_primitive_data, 0, 1))';
stand_devs = var_motions.^(.5);
stand_devs(3,:) = rad2deg(stand_devs(3,:));

figure
t = tiledlayout(3, 1);
t.TileSpacing = "compact";

for i = 1:3
nexttile;
x = 1:length(avg_motions);
xticks(1:length(gait_sequence))
xticklabels(string(all_gaits(1).primitive_labels))
xlim([0.5, length(gait_sequence)+0.5])
switch i 
    case 1
        ylabel('X-axis translation (cm)')
    case 2 
        ylabel('Y-axis translation (cm)')
    case 3 
        ylabel('Rotation (degrees)')
end

grid on 
hold on
plus_sd = avg_motions(i,:)+stand_devs(i,:);
minus_sd = avg_motions(i,:)-stand_devs(i,:);
% plot(x,plus_sd,'r')
% plot(x,minus_sd,'r')
p=fill([x fliplr(x)],[plus_sd fliplr(minus_sd)],'r','FaceAlpha',0.2);
p.EdgeColor = 'r';
stem(x,avg_motions(i,:),'b', 'LineWidth',2,'MarkerFaceColor','b', 'MarkerSize',10)
end
title(t,"Predicted body-frame movement behavior for each motion primitive of "+gait_name, 'FontSize', 22)
t.XLabel.String = 'Motion primitive index';
t.XLabel.FontSize = 20;
legend("\pm 1\sigma","avg. predicted",'FontSize',24)
