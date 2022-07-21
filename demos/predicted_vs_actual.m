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
gait_library_actual = load('data/gait_library_2_corrected.mat').gait_library_2;%gait_library_2;
n_gaits = length(gait_library_actual);
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
for i = 1:n_gaits
    all_gaits(i) = offlineanalysis.GaitPredict(gait_library_actual(i).robo_states, ...
                                                 motion_primitive_data, ...   
                                                 exp_2.params);
end

% [3] == Instantiate gait objects for each gait permutation to build a gait
% library. 

figure
t=tiledlayout(2, length(gait_library_actual));
t.TileSpacing = 'compact';
t.Padding = 'compact';

for i = 1:n_gaits
    gait_library_predict(i) = gaitdef.Gait(all_gaits(i), exp_2.params);
    new_title = "Predicted Gait " + gait_library_actual(i).gait_name + ...
                ": ["+ num2str(gait_library_actual(i).robo_states) + "]";
    nexttile;
    gait_library_predict(i).plot(10)
    title(new_title)
    xlim([-20 15])
    ylim([-12 20])
    xticks(-20:5:15);
    yticks(-10:5:20);
    daspect([1 1 1]);
    grid on
end

for i = 1:n_gaits
    nexttile;
    gait_library_actual(i).plot(10)
    xlim([-20 15])
    ylim([-12 20])
    xticks(-20:5:15);
    yticks(-10:5:20);
    daspect([1 1 1]);
    grid on
end
a = colorbar;
a.Label.String = 'Number of Gaits Executed';
a.Layout.Tile = 'east';


figure
t = tiledlayout(3, n_gaits);
t.TileSpacing = "compact";
t.Padding = 'compact';

% [4] == Plot the predicted behavior with variance for each motion
% primitive of the base gait. 
for j = 1:n_gaits
gait_sequence = gait_library_predict(j).robo_states;
motion_primitive_gait_data= motion_primitive_data(:,gait_sequence,:);
% Find mean, variance, and standard deviation of data.
avg_motions = squeeze(mean(motion_primitive_gait_data, 1))';
avg_motions(3,:) = rad2deg(avg_motions(3,:));
var_motions = squeeze(var(motion_primitive_gait_data, 0, 1))';
stand_devs = var_motions.^(.5);
stand_devs(3,:) = rad2deg(stand_devs(3,:));

delta_poses = gait_library_actual(j).delta_poses;
delta_poses(3,:) = rad2deg(delta_poses(3,:));
var_poses = gait_library_actual(j).var_delta_poses;
stand_dev_poses = var_poses.^(.5);
stand_dev_poses(3,:) = rad2deg(stand_dev_poses(3,:));

for i = 1:3
nexttile((i-1)*n_gaits+j);
x = 1:size(avg_motions,2);
xticks(1:length(gait_sequence))
xticklabels(string(all_gaits(j).primitive_labels))
xlim([0.5, length(gait_sequence)+0.5])
switch i 
    case 1
        ylabel('X-axis translation (cm)')
        title("Gait "+gait_library_actual(j).gait_name + ...
                ": ["+ num2str(gait_library_actual(j).robo_states) + "]")
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
p=fill([x fliplr(x)],[plus_sd fliplr(minus_sd)],'b','FaceAlpha',0.2, 'DisplayName',"\pm 1\sigma predicted");
p.EdgeColor = 'b';
stp = stem(x,avg_motions(i,:),'b', 'LineWidth',2,'MarkerFaceColor','b', 'MarkerSize',10,'DisplayName',"average predicted motion");

plus_sdp = delta_poses(i,:)+stand_dev_poses(i,:);
minus_sdp = delta_poses(i,:)-stand_dev_poses(i,:);
pa=fill([x fliplr(x)],[plus_sdp fliplr(minus_sdp)],'r','FaceAlpha',0.2,'DisplayName',"\pm 1\sigma actual");
pa.EdgeColor = 'r';
sta = stem(x,delta_poses(i,:),'r', 'LineWidth',2,'MarkerFaceColor','r', 'MarkerSize',8,'DisplayName',"average actual motion");
end
end

title(t,'Predicted vs. Actual Motion for each Motion Primitive in a Gait', 'FontSize', 24)
t.XLabel.String = 'Motion primitive index';
t.XLabel.FontSize = 24;
lg = legend([stp p sta pa],'FontSize',24);
lg.Orientation = 'horizontal';
lg.NumColumns = 2;
lg.Layout.Tile = 'north';

