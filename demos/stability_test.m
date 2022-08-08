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
         [9,16,1]};
gait_names = {'Gait B_120.mat';
              'Gait E_120.mat';
              'Gait E_60_left.mat';
              'Gait E_60_right.mat'};

n_gaits = length(gait_sequences);

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Extract and define parameters for GaitTest() objects.

% Define experimental parameters after investigating video. 
frame_start_list = [100, 234, 201, 109];      % for orange robot

% Find marker order by investigating first frame.
marker_order_list = [1 2 3 4;
                     1 2 3 4 ;
                     1 2 3 4;
                     1 2 3 4];
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
    if i == 1 || i ==2
        stab_exp(i).params.n_cycles= 120;
    else
        stab_exp(i).params.n_cycles= 60;
    end
        stab_exp(i).params.marker_order = marker_order_list(i, :);
        % Extract and store raw data from each trial
        filename = gait_names{i};
        stab_exp(i).raw_data = load(filename).tracking_data;  
     
end
%  markers_x(1, :) = load('data/visualtracking/Euler 9_corrected.mat').all_pt(1, 1:3:22);
%     markers_y(1, :) = load('data/visualtracking/Euler 9_corrected.mat').all_pt(1, 2:3:23);
%     centroid(:, :, 1) = mean([markers_x(1, :); markers_y(1, :)], 2);
% reference_markers = [markers_x(1, [8 7 4 3 1 2 5 6]);
%                              markers_y(1, [8 7 4 3 1 2 5 6]);]...
%                              - centroid(:, :, 1);
% for iTrial = 1:5
%     % Find initial rotation matrix for each trial to have consistent fixed frame.
%         markers_x(iTrial, :) = stab_exp(iTrial).raw_data(1, 1:3:22);
%     markers_y(iTrial, :) = stab_exp(iTrial).raw_data(1, 2:3:23);
%     centroid(:, :, iTrial) = mean([markers_x(iTrial, :); markers_y(iTrial, :)], 2);
%   
%         shifted_markers = [markers_x(iTrial, stab_exp(iTrial).params.marker_order);
%                            markers_y(iTrial, stab_exp(iTrial).params.marker_order)]...
%                            - centroid(:, :, iTrial);
%         % Find orientation of subsequent trials w.r.t. first trial GCS.
%          [regParams,Bfit,ErrorStats]=absor(reference_markers, shifted_markers);
%          stab_exp(iTrial).params.R_1 = [regParams.R zeros(2,1); 0 0 1];
% end
% [2] == Instantiate GaitTest() objects for each experimental trial.
% This analyzes the data from each trial to find motion primitive twist
% information.



% Instantiate objects for each gait tested. 
for i = 1:n_gaits
    all_gaits(i) = offlineanalysis.GaitTest(stab_exp(i).raw_data, ...
                                                 gait_sequences{i}(1,:), ...
                                                 stab_exp(i).params);
end
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
%t = tiledlayout(1,2);
%nexttile;
all_gaits(3).plot;
%nexttile;
figure(4)
all_gaits(4).plot;

% Add legend.
lgd = legend('Continuous robot position', 'Actual keyframe positions', ...
                  'Keyframe positions reconstructed from motion primitives','robot orientation');

%lgd.Layout.Tile = 'north';
lgd.Orientation = 'horizontal';

% Add colorbar.
a = colorbar;
a.Label.String = 'Number of gaits executed';
%a.Layout.Tile = 'east';

title(t, 'Effect of the tether being on the left or right side','FontSize',24)

if show_markers
    names = 'ABCDEF';
    figure
    tiledlayout(2,3)
    for iTrial = 1:5
    
    
    % Plot first image of experiment video.
    pictureName = sprintf('data/visualtracking/firstframe%c.jpg', names(iTrial));
    pic = imread(pictureName);
    nexttile;
    imshow(pic)
    hold on
    
    % Plot labeled (i.e., numbered) markers on top of image.
    markers_x(iTrial, :) = stab_exp(iTrial).raw_data(1, 1:3:22);
    markers_y(iTrial, :) = stab_exp(iTrial).raw_data(1, 2:3:23);
%     if iTrial ~=5
    for m = 1:8
        text(markers_x(iTrial, m), 400 - markers_y(iTrial, m), num2str(m),'Color','white','FontSize',14);
%     end
    end
    
    % Adjust size.
    xlim([min(markers_x(iTrial, :))-100, max(markers_x(iTrial, :))+100]);
    ylim([min(400 - markers_y(iTrial, :)) - 100, max(400 - markers_y(iTrial, :))+100]);
    
    
end
end
