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
state_order = load('data/visualtracking/state_order_carpet_MTA3.mat').state_order_carpet_MTA3;
% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Define experimental parameters after investigating video. 
frame_start_list = [145,149,143,144,124,107,117,111,113,110];
marker_order = repmat([2 4 6 5 3 1], 10,1);
marker_order(1,:) =[2 5 6 4 3 1];
marker_order([3,5,6,7,9,10],:) = repmat([3 5 6 4 2 1],6,1);




for i = 1:10
    % Define robot.
    exp(i).params.robot_name = 'MTA3';
    exp(i).params.substrate = 'carpet';
    exp(i).params.n_unique_states = 8;
    % Define pixel length.
    exp(i).params.pixel_length = 1/8.6343;
    exp(i).params.n_markers = 6;
    exp(i).params.transition_time= .55;
    exp(i).params.marker_order = marker_order(i,:);

    
    % Extract and store first frame information.
    exp(i).params.frame_1 = frame_start_list(i);
    % Extract and store raw data from each trial.
    filename = ['data/visualtracking/MTA3/Carpet/Euler_', num2str(i), '_L.mat'];
    if ~isfile(filename)
        filename = ['data/visualtracking/MTA3/Carpet/Euler_', num2str(i), '_L_stitched.mat'];
    end
    exp(i).raw_data = load(filename).tracking_data;  
end
figure(1)
tiledlayout(2,5)
for i = 1:10
    pic_name = ['firstframe_Euler_', num2str(i)];
    % Plot first image of experiment video.
    file_name = ['data/visualtracking/MTA3/Carpet/', pic_name, '_L.jpg'];
    pic = imread(file_name);
    nexttile;
    imshow(pic)
    hold on

    % Plot labeled (i.e., numbered) markers on top of image.
    markers_x(i, :) = exp(i).raw_data(1, 1:3:exp(i).params.n_markers*3-2);
    markers_y(i, :) = exp(i).raw_data(1, 2:3:exp(i).params.n_markers*3-1);
%     if iTrial ~=5
    for m = 1:exp(i).params.n_markers
        text(markers_x(i, m), 1080-markers_y(i, m), num2str(m),'Color','white','FontSize',14);
%     end
    end
    
    % Adjust size.
     xlim([min(markers_x(i, :))-200, max(markers_x(i, :))+200]);
     ylim([min(1080 - markers_y(i, :)) - 200, max(1080 - markers_y(i, :))+200]);
end



% Define first experiment first frame orientation as theta = 0. 
markers_x(1, :) = exp(1).raw_data(1, 1:3:exp(1).params.n_markers*3-2);
markers_y(1, :) = exp(1).raw_data(1, 2:3:exp(1).params.n_markers*3-1);
centroid(:, :, 1) = mean([markers_x(1, :); markers_y(1, :)], 2);
% Move initial position to (0,0) origin.
reference_markers = [markers_x(1, exp(1).params.marker_order); markers_y(1, exp(1).params.marker_order);]...
                    - centroid(:, :, 1);
% For each trial, rotate data to align first frame global orientations.
 for i = 1:10
    % Find initial rotation matrix for each trial to have consistent fixed frame.
    markers_x(i, :) = exp(i).raw_data(1, 1:3:exp(i).params.n_markers*3-2);
    markers_y(i, :) = exp(i).raw_data(1, 2:3:exp(i).params.n_markers*3-1);
    centroid(:, :, i) = mean([markers_x(i, :); markers_y(i, :)], 2);

    % Move initial position to (0,0) origin.
    shifted_markers = [markers_x(i, exp(i).params.marker_order);
                       markers_y(i, exp(i).params.marker_order)]...
                       - centroid(:, :, i);

    % Find orientation of subsequent trials w.r.t. first trial GCS.
    [regParams,~,~] = absor(reference_markers, shifted_markers);
    %exp(i).params.R_1 = [regParams.R zeros(2,1); 0 0 1];
 end



% [2] == Instantiate experimental motion primitive data analysis and plot
% results.

% Set up figure. 
figure(2)
t = tiledlayout(2, 5);

for i = 1:10
    all_trials(i) = offlineanalysis.PrimitivesTest(exp(i).raw_data, exp(i).params, state_order(i,:));
    nexttile;
    all_trials(i).plot;
    title("Trial " +num2str(i))
    % Extract data to store in a more convenient way for gait synthesis.
    delta_x_data(i, :) = all_trials(i).delta_x;
    delta_y_data(i, :) = all_trials(i).delta_y;
    delta_theta_data(i, :) = all_trials(i).delta_theta;
end
% Add legend.
%lgd = legend('Continuous robot position', 'Actual keyframe positions', ...
 %                 'Keyframe positions reconstructed from gait twists');
lgd.Layout.Tile = 'north';
lgd.Orientation = 'horizontal';

% Add colorbar.
a = colorbar;
a.Label.String = 'Number of motion primitives executed';
a.Layout.Tile = 'east';

title(t, 'Motion primitive exploration (Euler tours) for orange robot on carpet','FontSize',24)


% [3] == Concatenate all motion primitive motion data into a 3D matrix.
motion_primitive_data = cat(3, delta_x_data, delta_y_data, delta_theta_data);
% [4] == Set up parameters for gait synthesis.
params_3.robot_name = 'MTA3';
params_3.substrate= 'carpet';
params_3.n_unique_states = 8;
params_3.alpha_var = .0005*[1 1 1]';
params_3.MAX_ROTATION = deg2rad(1);
params_3.MAX_TRANSLATION = .2;
params_3.goal = 2;
params_3.alpha_len = .1;
params_3.n_variations = 300;

% [3] == Instantiate GaitSynthesizer() object to generate gaits from motion
% primitive data.
%gait_synthesis = offlineanalysis.GaitSynthesizer( motion_primitive_data , params_3)
%gaits = gait_synthesis.solutions

save data/MTA3_motion_primitives_carpet_new.mat motion_primitive_data


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


title(t,'Predicted vs. Actual Motion for each MTA3 Motion Primitive in an Euler Tour on Carpet', 'FontSize', 24)
t.XLabel.String = 'Motion primitive index';
t.XLabel.FontSize = 24;
%lg = legend([stp p sta pa],'FontSize',24);
lg.Orientation = 'horizontal';
lg.NumColumns = 2;
lg.Layout.Tile = 'north';
N = size(motion_primitive_data,1);

for i = 1:N-2
    avg_motions = squeeze(mean(motion_primitive_data(1:i+2,:,:), 1))';
avg_motions(3,:) = rad2deg(avg_motions(3,:));
var_motions = squeeze(var(motion_primitive_data(1:i+2,:,:), 0, 1))';
stand_devs = var_motions.^(.5);
stand_devs(3,:) = rad2deg(stand_devs(3,:));
avg_tot(:,i) = mean(avg_motions, 2);
avg_tot_err(:,i) = var(avg_motions,0,2);
avg_std(:,i) = mean(stand_devs, 2);


end
figure
tiledlayout(3,1)
hold on
for i = 1:3
    nexttile
    yyaxis left
plot(3:N, avg_std(i,:))
ylabel('Average Standard Deviation')
yyaxis right
plot(3:N, avg_tot(i,:))
errorbar(3:N, avg_tot(i,:), avg_tot_err(i,:))
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
plot(3:N, mean(avg_std))
ylabel('Average Standard Deviation')
yyaxis right
plot(3:N, mean(avg_tot))
errorbar(3:N, mean(avg_tot), var(avg_tot))
ylabel('Average Motion')
title('Total motion')