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
params_3.alpha_var = [0 0 0]';
params_3.MAX_ROTATION = deg2rad(1);


% [3] == Instantiate GaitSynthesizer() object to generate gaits from motion
% primitive data.
gait_synthesis = offlineanalysis.GaitSynthesizer( motion_primitive_data , params_3)
gaits = gait_synthesis.solutions

save data/MTA3_motion_primitives_carpet.mat motion_primitive_data


% Find mean, variance, and standard deviation of data.
avg_motions = squeeze(mean(motion_primitive_data, 1))';
avg_motions(3,:) = rad2deg(avg_motions(3,:));
var_motions = squeeze(var(motion_primitive_data, 0, 1))';
stand_devs = var_motions.^(.5);
stand_devs(3,:) = rad2deg(stand_devs(3,:));


figure
t = tiledlayout(3, 1);
t.TileSpacing = "compact";
t.Padding = 'compact';

for i = 1:3
nexttile;
x = 1:size(avg_motions,2);
xticks(1:length(motion_primitive_data))

xlim([0.5, length(motion_primitive_data)+0.5])
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
p=fill([x fliplr(x)],[plus_sd fliplr(minus_sd)],'b','FaceAlpha',0.2, 'DisplayName',"\pm 1\sigma predicted");
p.EdgeColor = 'b';
stp = stem(x,avg_motions(i,:),'b', 'LineWidth',2,'MarkerFaceColor','b', 'MarkerSize',10,'DisplayName',"average predicted motion");

end


title(t,'Predicted vs. Actual Motion for each Motion Primitive in an Euler Tour on Carpet', 'FontSize', 24)
t.XLabel.String = 'Motion primitive index';
t.XLabel.FontSize = 24;
%lg = legend([stp p sta pa],'FontSize',24);
lg.Orientation = 'horizontal';
lg.NumColumns = 2;
lg.Layout.Tile = 'north';


for i = 1:8
    avg_motions = squeeze(mean(motion_primitive_data(1:i+2,:,:), 1))';
avg_motions(3,:) = rad2deg(avg_motions(3,:));
var_motions = squeeze(var(motion_primitive_data(1:i+2,:,:), 0, 1))';
stand_devs = var_motions.^(.5);
stand_devs(3,:) = rad2deg(stand_devs(3,:));
avg_tot(:,i) = mean(avg_motions, 2);
avg_std(:,i) = mean(stand_devs, 2);


end
figure
tiledlayout(3,1)
hold on
for i = 1:3
    nexttile
    yyaxis left
plot(3:10, avg_std(i,:))
ylabel('Average Standard Deviation')
yyaxis right
plot(3:10, avg_tot(i,:))
ylabel('Average Motion')
switch i
    case 1
        title('X-axis translation (cm)')
    case 2
        title('Y-axis translation (cm)')
    case 3
        title('Rotation (deg)')
end
end
nexttile
    yyaxis left
plot(3:10, mean(avg_std))
ylabel('Average Standard Deviation')
yyaxis right
plot(3:10, mean(avg_tot))
ylabel('Average Motion')
title('Total motion')