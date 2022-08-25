% 
% Script: stability_test.m
%  
% Dependencies:
%   +demos/data/visualtracking/Gait B_120.mat
%   +demos/data/visualtracking/Gait E_120.mat
%   +demos/data/visualtracking/Gait E_60_left.mat
%   +demos/data/visualtracking/Gait E_60_right.mat
%   +demos/data/visualtracking/Gait B_120_right.mat
%
%   +offlineanalysis/GaitTest
%   +gaitdef/Gait
%
% Description: 
%   Investigate the stability of the gaits, especially w.r.t. the tether.


% [0] == Script setup
clear; clc; close all

% Add dependencies to classpath
addpath('../');
addpath('data/visualtracking');

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
set(0, 'DefaultAxesFontSize', 18);


%% [1] == Extract and define parameters for GaitTest() objects.
gait_sequences = {[16,7,5,11,14];    % Gait B (rotational)
                  [9,16,1];          % Gait E (translational)
                  [9,16,1];
                  [9,16,1];
                  [16,7,5,11,14]};

gait_names = {'Gait B_120.mat';
              'Gait E_120.mat';
              'Gait E_60_left.mat';
              'Gait E_60_right.mat';
              'Gait B_120_right.mat'}; % Read note below!

% Note: Gait B_120_right.mat does not actually have a consistent
% right-directed tether but is left with this name for consistency with the
% experimental videos. 

n_gaits = length(gait_sequences);


% Define experimental parameters after investigating videos. 
frame_start_list = [382, 234, 141, 109, 76];

% Find marker order by investigating first frame.
marker_order_list = [1 3 4 2;
                     1 4 3 2 ;
                     1 4 3 2;
                     1 3 4 2;
                     1 4 3 2];
show_markers = false;     % plots first frame of each video

% Initialize the stability experiment struct. 
stab_exp = struct('params', [], 'raw_data', []);

% Extract data.
for i = 1:n_gaits
    % Define robot / experiment parameters.
    stab_exp(i).params.robot_name = 'orange';
    stab_exp(i).params.substrate = 'black mat';
    stab_exp(i).params.n_markers = 4;
    stab_exp(i).params.pixel_length = 1/8.6343;     % cm per pixel

    % Define number of gait cycles run.
    if i == 3 || i ==4
        stab_exp(i).params.n_cycles= 60;
    else
        stab_exp(i).params.n_cycles= 120;
    end

    % Extract and store first frame information.
    stab_exp(i).params.frame_1 = frame_start_list(i);
    stab_exp(i).params.marker_order = marker_order_list(i,:);
    
    % Extract and store raw data from each trial
    filename = gait_names{i};
    stab_exp(i).raw_data = load(filename).tracking_data;  
     
end

%% [2] == (Optional) Rotate the data w.r.t the initial global orientation.

% Define first experiment first frame orientation as theta = 0. 
markers_x(1, :) = stab_exp(1).raw_data(1, 1:3:stab_exp(1).params.n_markers*3-2);
markers_y(1, :) = stab_exp(1).raw_data(1, 2:3:stab_exp(1).params.n_markers*3-1);
centroid(:, :, 1) = mean([markers_x(1, :); markers_y(1, :)], 2);
% Move initial position to (0,0) origin.
reference_markers = [markers_x(1, [1 3 4 2]); markers_y(1, [1 3 4 2]);]...
                    - centroid(:, :, 1);

% For each trial, rotate data to align first frame global orientations.
 for i = 1:n_gaits
    % Find initial rotation matrix for each trial to have consistent fixed frame.
    markers_x(i, :) = stab_exp(i).raw_data(1, 1:3:stab_exp(i).params.n_markers*3-2);
    markers_y(i, :) = stab_exp(i).raw_data(1, 2:3:stab_exp(i).params.n_markers*3-1);
    centroid(:, :, i) = mean([markers_x(i, :); markers_y(i, :)], 2);

    % Move initial position to (0,0) origin.
    shifted_markers = [markers_x(i, stab_exp(i).params.marker_order);
                       markers_y(i, stab_exp(i).params.marker_order)]...
                        - centroid(:, :, i);

    % Find orientation of subsequent trials w.r.t. first trial GCS.
    [regParams,~,~]= absor(reference_markers, shifted_markers);
    stab_exp(i).params.R_1 = [regParams.R zeros(2,1); 0 0 1];
 end


%% [3] == Instantiate GaitTest() objects for each experimental trial.
% This analyzes the data from each trial to find motion primitive twist
% information.

% Instantiate objects for each gait tested. 
all_gaits = offlineanalysis.GaitTest.empty(0,n_gaits); 
gait_defs = gaitdef.Gait.empty(0,n_gaits);

% Calculate twists for each gait experiment.
for i = 1:n_gaits
    all_gaits(i) = offlineanalysis.GaitTest(stab_exp(i).raw_data, ...
                                                 gait_sequences{i}(1,:), ...
                                                 stab_exp(i).params);
    gait_defs(i) = gaitdef.Gait(all_gaits(i), stab_exp(i).params);


    % Manually find the twists. 
    % Extract gait / experiment details. 
    n_cycles = all_gaits(i).n_cycles;
    len_gait = all_gaits(i).len_gait;
     for j = 1:n_cycles - 1
         % Record indexes for each motion primitive tail and head. 
         tail_idx{i}(1,j) =  all_gaits(i).keyframes(len_gait*(j-1)+2);
         head_idx{i}(1,j) = all_gaits(i).keyframes(len_gait*j+2);

         % Find change in poses w.r.t. global inertial frame for each motion primitive. 
         delta_poses{i}(:,j) = all_gaits(i).poses(:, head_idx{i}(1,j)) - all_gaits(i).poses(:, tail_idx{i}(1,j));

         % Find global orientation of robot at each motion primitive tail. 
         global_theta{i}(:,j) = all_gaits(i).poses(3, tail_idx{i}(1,j));

         % Convert from global frame to body frame.  
         rot_mat =  [cos(global_theta{i}(1,j)) -sin(global_theta{i}(1,j)); sin(global_theta{i}(1,j)) cos(global_theta{i}(1,j))];
         delta_poses{i}(1:2,j) =rot_mat'*delta_poses{i}(1:2,j);

         % Convert to twists.
         twists{i}(:,j)  = gait_defs(i).delta_pose_2_twist(delta_poses{i}(:,j), all_gaits(i).transition_time*len_gait);
     end
     
end

%% [4] == Plot the full motion data for each experiment.

% Plot both Gait B experiments. 
figure(1)
t1 = tiledlayout(1,2);
nexttile;
all_gaits(1).plot;
title('Experiment 1')
nexttile;
all_gaits(5).plot;
title('Experiment 2')
lgd = legend('Continuous robot position', 'Actual keyframe positions', ...
                  'Keyframe positions reconstructed from motion primitives','robot orientation');
lgd.Layout.Tile = 'north';
lgd.Orientation = 'horizontal';
a = colorbar;
a.Label.String = 'Number of gaits executed';
a.Layout.Tile = 'east';
title(t1, 'Comparison of two experiements of Gait B [16,7,5,11,14]','FontSize',24)

% Plot Gait E with moving tether.
figure(2)
all_gaits(2).plot;
lgd = legend('Continuous robot position', 'Actual keyframe positions', ...
                  'Keyframe positions reconstructed from motion primitives','robot orientation');
lgd.Orientation = 'horizontal';
a = colorbar;
a.Label.String = 'Number of gaits executed';
title('Gait B [9 16 1]','FontSize',24')

% Plot Gait E with fixed tethers.
figure(3)
t3 = tiledlayout(1,2);
nexttile;
all_gaits(3).plot;
title('Tether pulling to the left')
nexttile;
all_gaits(4).plot;
title('Tether pulling to the right')
lgd = legend('Continuous robot position', 'Actual keyframe positions', ...
                  'Keyframe positions reconstructed from motion primitives','robot orientation');

lgd.Layout.Tile = 'north';
lgd.Orientation = 'horizontal';
a = colorbar;
a.Label.String = 'Number of gaits executed';
a.Layout.Tile = 'east';
title(t3, 'Comparison of gait B [9 16 1] with different tether orientations','FontSize',24)

%% [4] == Plot the twist data for each experiment.

figure(4)
t = tiledlayout(3,1);
nexttile;
plot(twists{1}(1,:))
hold on
plot(twists{5}(1,:))
ylabel('$v_x$ (cm/s)')

nexttile;
plot(twists{1}(2,:))
hold on
plot(twists{5}(2,:))
ylabel('$v_y$ (cm/s)')

nexttile;
plot(rad2deg(twists{1}(3,:)))
hold on
plot(rad2deg(twists{5}(3,:)))
ylabel('$\omega$ (deg/s)')

lgd = legend('Experiment 1 ', 'Experiment 2');
lgd.Orientation = 'horizontal';
lgd.Layout.Tile = 'north';
xlabel('Number of Gait Cycles')
title(t,'Twist (body velocity) for two experiments of Gait B [16,7,5,11,14]', 'FontSize',22)

figure(5)
t = tiledlayout(3,1);
nexttile;
scatter(rad2deg(global_theta{1}(1,:)),twists{1}(1,:))
hold on
scatter(rad2deg(global_theta{5}(1,:)),twists{5}(1,:))
xlim([-180 180])
ylabel('$v_x$ (cm/s)')

nexttile;
scatter(rad2deg(global_theta{1}(1,:)),twists{1}(2,:))
hold on
scatter(rad2deg(global_theta{5}(1,:)),twists{5}(2,:))
xlim([-180 180])
ylabel('$v_y$ (cm/s)')

nexttile;
scatter(rad2deg(global_theta{1}(1,:)),twists{1}(3,:))
hold on
scatter(rad2deg(global_theta{5}(1,:)),twists{5}(3,:))
xlim([-180 180])
ylabel('$\omega$ (deg/s)')

lgd = legend('Experiment 1 ', 'Experiment 2');
lgd.Orientation = 'horizontal';
lgd.Layout.Tile = 'north';
xlabel('Global robot orientation $\theta_G$ (deg)')
title(t,'Twist (body velocity) for two experiments of Gait B [16,7,5,11,14]', 'FontSize',22)

figure(6)
t = tiledlayout(3,1);
nexttile;
plot(twists{3}(1,:))
hold on
plot(twists{4}(1,:))
ylabel('$v_x$ (cm/s)')

nexttile;
plot(twists{3}(2,:))
hold on
plot(twists{4}(2,:))
ylabel('$v_y$ (cm/s)')

nexttile;
plot(rad2deg(twists{3}(3,:)))
hold on
plot(rad2deg(twists{4}(3,:)))
ylabel('$\omega$ (deg/s)')

lgd = legend('Tether to left', 'Tether to right');
lgd.Orientation = 'horizontal';
lgd.Layout.Tile = 'north';
xlabel('Number of Gait Cycles')
title(t,'Twist (body velocity) for two experiments of Gait E [9,16,1]', 'FontSize',22)

figure(7)
t = tiledlayout(3,1);
nexttile;
scatter(rad2deg(global_theta{3}(1,:)),twists{3}(1,:))
hold on
scatter(rad2deg(global_theta{4}(1,:)),twists{4}(1,:))
xlim([-180 180])
ylabel('$v_x$ (cm/s)')

nexttile;
scatter(rad2deg(global_theta{3}(1,:)),twists{3}(2,:))
hold on
scatter(rad2deg(global_theta{4}(1,:)),twists{4}(2,:))
xlim([-180 180])
ylabel('$v_y$ (cm/s)')

nexttile;
scatter(rad2deg(global_theta{3}(1,:)),twists{3}(3,:))
hold on
scatter(rad2deg(global_theta{4}(1,:)),twists{4}(3,:))
xlim([-180 180])
ylabel('$\omega$ (deg/s)')

lgd = legend('Tether to left', 'Tether to right');
lgd.Orientation = 'horizontal';
lgd.Layout.Tile = 'north';
xlabel('Global robot orientation $\theta_G$ (deg)')
title(t,'Twist (body velocity) for two experiments of Gait E [9,16,1]', 'FontSize',22)

if show_markers
    figure
    tiledlayout(2,3)
    for i = 1:n_gaits
    
    pic_name = gait_names{i};
    pic_name(end-3:end) = [];
    % Plot first image of experiment video.
    file_name = ['data/visualtracking/firstframe_', pic_name, '.jpg'];
    pic = imread(file_name);
    nexttile;
    imshow(pic)
    hold on
    
    % Plot labeled (i.e., numbered) markers on top of image.
    markers_x(i, :) = stab_exp(i).raw_data(1, 1:3:stab_exp(i).params.n_markers*3-2);
    markers_y(i, :) = stab_exp(i).raw_data(1, 2:3:stab_exp(i).params.n_markers*3-1);
%     if iTrial ~=5
    for m = 1:stab_exp(i).params.n_markers
        text(markers_x(i, m), 1080-markers_y(i, m), num2str(m),'Color','white','FontSize',14);
%     end
    end
    
    % Adjust size.
     xlim([min(markers_x(i, :))-200, max(markers_x(i, :))+200]);
     ylim([min(1080 - markers_y(i, :)) - 200, max(1080 - markers_y(i, :))+200]);
    
    
end
end



