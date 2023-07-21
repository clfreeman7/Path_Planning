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
clear; clc; close all;
% Add dependencies to classpath
addpath('../');

motion_primitive_data = load('data/32SR_motion_primitives.mat').motion_primitive_data;

% [4] == Set up parameters for gait synthesis.
params.robot_name = 'orange';
params.substrate= 'black mat';

% [3] == Instantiate GaitSynthesizer() object to generate gaits from motion
% primitive data.
gait_synthesis = offlineanalysis.GaitSynthesizer( motion_primitive_data )
gaits = struct2table(gait_synthesis.solutions);

% Sort gaits by the linear speed. 
sorted_gaits = sortrows(gaits, 'lin_speed', 'descend');
all_gaits = table2struct(sorted_gaits);

for i = 1:length(all_gaits)
    predicted_NS(i) = offlineanalysis.GaitPredict(all_gaits(i).tail_states, motion_primitive_data, params);
end


figure
t = tiledlayout(2,3);
for i = 1:min(6, length(all_gaits))
    nexttile;
    predicted_poses{i}(:,:) = predicted_NS(i).plot(60);
    title(['Synthesized Gait ', num2str(i), ': [', num2str(all_gaits(i).tail_states)], "]")
    grid on
end

figure

t = tiledlayout(3, 6);
t.TileSpacing = "compact";
t.Padding = 'compact';
for j = 1:6
    mp_seq = predicted_NS(i).primitive_labels;
    mps_gait= motion_primitive_data(:,mp_seq,:);
gait = predicted_NS(i).robo_states;
% Find mean, variance, and standard deviation of data.
avg_motions = squeeze(mean(mps_gait, 1))';
avg_motions(3,:) = rad2deg(avg_motions(3,:));
var_motions = squeeze(var(mps_gait, 0, 1))';
stand_devs = var_motions.^(.5);
stand_devs(3,:) = rad2deg(stand_devs(3,:));

for i = 1:3
nexttile(6*(i-1)+j);
x = 1:size(avg_motions,2);
xticks(1:length(mp_seq))
%xticklabels(string(predicted_gait_H(j).primitive_labels))
xlim([0.5, length(mp_seq)+0.5])
switch i 
    case 1
        ylabel('X-axis translation (cm)')
        title("Predicted Gait "+ num2str(j)+ ": [" +  num2str(gait) + "]")
    case 2 
        ylabel('Y-axis translation (cm)')
    case 3 
        ylabel('Rotation (degrees)')
end

grid on 
hold on
plus_sd = avg_motions(i,:)+2*stand_devs(i,:);
minus_sd = avg_motions(i,:)-2*stand_devs(i,:);
p=fill([x fliplr(x)],[plus_sd fliplr(minus_sd)],'b','FaceAlpha',0.2, 'DisplayName',"\pm 2\sigma predicted");
p.EdgeColor = 'b';
stp = stem(x,avg_motions(i,:),'b', 'LineWidth',2,'MarkerFaceColor','b', 'MarkerSize',10,'DisplayName',"average predicted motion");

end
end
title(t,'Predicted vs. Actual Motion for each Motion Primitive in a Gait', 'FontSize', 24)
t.XLabel.String = 'Motion primitive index';
t.XLabel.FontSize = 24;
lg = legend([stp p],'FontSize',24);
lg.Orientation = 'horizontal';
lg.NumColumns = 2;
lg.Layout.Tile = 'north';

