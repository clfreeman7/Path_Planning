% 
% Script: demo_GaitTest.m
%  
% Dependencies:
%   +demos/data/visualtracking/Gait A_corrected.mat
%   +demos/data/visualtracking/Gait B_corrected.mat
%   +demos/data/visualtracking/Gait B_corrected.mat
%   +demos/data/visualtracking/Gait D_corrected.mat
%   +demos/data/visualtracking/Gait E_corrected.mat
%
%   +offlineanalysis/GaitTest
%
% Description: 
%   Demonstrate how to process the raw experimental data from the visual
%   tracking of gaits.


% [0] == Script setup
clear; clc; close all
% Add dependencies to classpath
addpath('../');
addpath('data/visualtracking');
gait_sequences = {[16,7,5,11,14];
         [9,16,1];
         [9,16,1];
         [9,16,1];
         [16,7,5,11,14]};
gait_names = {'Gait B_120.mat';
              'Gait E_120.mat';
              'Gait E_60_left.mat';
              'Gait E_60_right.mat';
              'Gait B_120_right.mat'};

n_gaits = length(gait_sequences);

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Extract and define parameters for GaitTest() objects.

% Define experimental parameters after investigating video. 
frame_start_list = [100, 234, 141, 109, 155];      % for orange robot

% Find marker order by investigating first frame.
marker_order_list = [1 3 4 2;
                     1 4 3 2 ;
                     1 4 3 2;
                     1 3 4 2;
                     1 3 4 2];
show_markers = false;

% Extract data.
for i = 1:n_gaits
    % Define robot.
    stab_exp(i).params.robot_name = 'orange';
    stab_exp(i).params.substrate = 'black mat';
    stab_exp(i).params.n_markers = 4;

    stab_exp(i).params.pixel_length = 1/8.6343;
    % Extract and store first frame information.
    stab_exp(i).params.frame_1 = frame_start_list(i);
    stab_exp(i).params.marker_order = marker_order_list(i,:);
    if i == 3 || i ==4
        stab_exp(i).params.n_cycles= 60;
    else
        stab_exp(i).params.n_cycles= 120;
    end
        stab_exp(i).params.marker_order = marker_order_list(i, :);
        % Extract and store raw data from each trial
        filename = gait_names{i};
        stab_exp(i).raw_data = load(filename).tracking_data;  
     
end
    markers_x(1, :) = stab_exp(1).raw_data(1, 1:3:stab_exp(1).params.n_markers*3-2);
    markers_y(1, :) = stab_exp(1).raw_data(1, 2:3:stab_exp(1).params.n_markers*3-1);
     centroid(:, :, 1) = mean([markers_x(1, :); markers_y(1, :)], 2);
 reference_markers = [markers_x(1, [1 3 4 2]);
                              markers_y(1, [1 3 4 2]);]...
                              - centroid(:, :, 1);
 for iTrial = 1:n_gaits
     % Find initial rotation matrix for each trial to have consistent fixed frame.
    markers_x(iTrial, :) = stab_exp(iTrial).raw_data(1, 1:3:stab_exp(iTrial).params.n_markers*3-2);
    markers_y(iTrial, :) = stab_exp(iTrial).raw_data(1, 2:3:stab_exp(iTrial).params.n_markers*3-1);
     centroid(:, :, iTrial) = mean([markers_x(iTrial, :); markers_y(iTrial, :)], 2);
%  
        shifted_markers = [markers_x(iTrial, stab_exp(iTrial).params.marker_order);
                            markers_y(iTrial, stab_exp(iTrial).params.marker_order)]...
                            - centroid(:, :, iTrial);
%         % Find orientation of subsequent trials w.r.t. first trial GCS.
          [regParams,Bfit,ErrorStats]=absor(reference_markers, shifted_markers);
          stab_exp(iTrial).params.R_1 = [regParams.R zeros(2,1); 0 0 1];
 end
% [2] == Instantiate GaitTest() objects for each experimental trial.
% This analyzes the data from each trial to find motion primitive twist
% information.


stab_exp(5).params.R_1 = eye(3);
% Instantiate objects for each gait tested. 
for i = 1:n_gaits
    all_gaits(i) = offlineanalysis.GaitTest(stab_exp(i).raw_data, ...
                                                 gait_sequences{i}(1,:), ...
                                                 stab_exp(i).params);
    gait_defs(i) = gaitdef.Gait(all_gaits(i), stab_exp(i).params);
    delta_poses = [];
     for j = 1:all_gaits(i).n_cycles - 1
         start_idx(j,i) =  all_gaits(i).len_gait*j+1;
         end_idx(j,i) = all_gaits(i).len_gait*(j-1)+1;
         delta_poses(:, j) = all_gaits(i).poses(:, all_gaits(i).keyframes(all_gaits(i).len_gait*j+2)) - all_gaits(i).poses(:, all_gaits(i).keyframes(all_gaits(i).len_gait*(j-1)+2));
         twists(:, j, i) = gait_defs(i).delta_pose_2_twist(delta_poses(:,j), all_gaits(i).transition_time*all_gaits(i).len_gait);
     end
     
end

%delta_pose_2_twist
% Set up figure for plotting
figure(1)
all_gaits(1).plot;
% Add legend.
lgd = legend('Continuous robot position', 'Actual keyframe positions', ...
                  'Keyframe positions reconstructed from motion primitives','robot orientation');
a = colorbar;
a.Label.String = 'Number of gaits executed';
% Set up figure for plotting
figure(2)
all_gaits(2).plot;
% Add legend.
lgd = legend('Continuous robot position', 'Actual keyframe positions', ...
                  'Keyframe positions reconstructed from motion primitives','robot orientation');
lgd.Orientation = 'horizontal';
a = colorbar;
a.Label.String = 'Number of gaits executed';
figure(3)
t = tiledlayout(1,2);
nexttile;
all_gaits(3).plot;
nexttile;
all_gaits(4).plot;
figure(4)
all_gaits(5).plot;
% Add legend.
lgd = legend('Continuous robot position', 'Actual keyframe positions', ...
                  'Keyframe positions reconstructed from motion primitives','robot orientation');

lgd.Layout.Tile = 'north';
lgd.Orientation = 'horizontal';

% Add colorbar.
a = colorbar;
a.Label.String = 'Number of gaits executed';
a.Layout.Tile = 'east';

title(t, 'Effect of the tether being on the left or right side','FontSize',24)
figure
tiledlayout(3,1);
nexttile;
%plot(delta_poses(3,:))
plot(twists(1,1:59,2))
hold on
plot(twists(1,60:end,2))

plot(twists(1,1:59,3))
plot(twists(1,1:59,4))


nexttile;
plot(twists(2,1:59,4))
hold on
nexttile;
plot(twists(3,1:59,4))
hold on

if show_markers
    figure
    tiledlayout(2,2)
    for iTrial = 1:n_gaits
    
    pic_name = gait_names{iTrial};
    pic_name(end-3:end) = [];
    % Plot first image of experiment video.
    pictureName = ['data/visualtracking/firstframe_', pic_name, '.jpg'];
    pic = imread(pictureName);
    nexttile;
    imshow(pic)
    hold on
    
    % Plot labeled (i.e., numbered) markers on top of image.
    markers_x(iTrial, :) = stab_exp(iTrial).raw_data(1, 1:3:stab_exp(iTrial).params.n_markers*3-2);
    markers_y(iTrial, :) = stab_exp(iTrial).raw_data(1, 2:3:stab_exp(iTrial).params.n_markers*3-1);
%     if iTrial ~=5
    for m = 1:stab_exp(iTrial).params.n_markers
        text(markers_x(iTrial, m), 1080-markers_y(iTrial, m), num2str(m),'Color','white','FontSize',14);
%     end
    end
    
    % Adjust size.
     xlim([min(markers_x(iTrial, :))-200, max(markers_x(iTrial, :))+200]);
     ylim([min(1080 - markers_y(iTrial, :)) - 200, max(1080 - markers_y(iTrial, :))+200]);
    
    
end
end
