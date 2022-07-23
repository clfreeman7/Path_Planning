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

gait_sequences = {[3,7,2]; 
         [16,7,5,11,14];
         [8,9,16]; 
         [4,14];
         [9,16,1]};

n_gaits = length(gait_sequences);

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Extract and define parameters for GaitTest() objects.

% Define experimental parameters after investigating video. 
frame_start_list = [607, 103, 176, 226, 368];      % for orange robot

% Find marker order by investigating first frame.
marker_order_list = [2 1 4 6 7 8 5 3;
                     5 3 2 1 4 6 7 8;
                     2 3 5 8 7 6 4 1;
                     4 2 1 3 5 7 8 6;
                    7 6 4 2 1 3 5 8];
show_markers = true;

% Extract data.
for i = 1:n_gaits
    % Define robot.
    gait_exp_2(i).params.robot_name = 'orange';
    gait_exp_2(i).params.substrate = 'black mat';
    
    % Extract and store first frame information.
    gait_exp_2(i).params.frame_1 = frame_start_list(i);
        gait_exp_2(i).params.marker_order = marker_order_list(i, :);
    % Extract and store raw data from each trial.
    filename = ['data/visualtracking/Gait', ' ', num2str(char('A' + i -1)), '_corrected.mat'];
    gait_exp_2(i).raw_data = load(filename).all_pt;  
end
 markers_x(1, :) = load('data/visualtracking/Euler 9_corrected.mat').all_pt(1, 1:3:22);
    markers_y(1, :) = load('data/visualtracking/Euler 9_corrected.mat').all_pt(1, 2:3:23);
    centroid(:, :, 1) = mean([markers_x(1, :); markers_y(1, :)], 2);
reference_markers = [markers_x(1, [8 7 4 3 1 2 5 6]);
                             markers_y(1, [8 7 4 3 1 2 5 6]);]...
                             - centroid(:, :, 1);
for iTrial = 1:5
    % Find initial rotation matrix for each trial to have consistent fixed frame.
        markers_x(iTrial, :) = gait_exp_2(iTrial).raw_data(1, 1:3:22);
    markers_y(iTrial, :) = gait_exp_2(iTrial).raw_data(1, 2:3:23);
    centroid(:, :, iTrial) = mean([markers_x(iTrial, :); markers_y(iTrial, :)], 2);
  
        shifted_markers = [markers_x(iTrial, gait_exp_2(iTrial).params.marker_order);
                           markers_y(iTrial, gait_exp_2(iTrial).params.marker_order)]...
                           - centroid(:, :, iTrial);
        % Find orientation of subsequent trials w.r.t. first trial GCS.
         [regParams,Bfit,ErrorStats]=absor(reference_markers, shifted_markers);
         gait_exp_2(iTrial).params.R_1 = [regParams.R zeros(2,1); 0 0 1];
end
% [2] == Instantiate GaitTest() objects for each experimental trial.
% This analyzes the data from each trial to find motion primitive twist
% information.

% Set up figure for plotting
figure(1)
t = tiledlayout(2, 3);

% Instantiate objects for each gait tested. 
for i = 1:n_gaits
    all_gaits(i) = offlineanalysis.GaitTest(gait_exp_2(i).raw_data, ...
                                                 gait_sequences{i}(1,:), ...
                                                 gait_exp_2(i).params);
    nexttile;
    all_gaits(i).plot;        

end

% Add legend.
lgd = legend('Continuous robot position', 'Actual keyframe positions', ...
                  'Keyframe positions reconstructed from motion primitives','robot orientation');
lgd.Layout.Tile = 'north';
lgd.Orientation = 'horizontal';

% Add colorbar.
a = colorbar;
a.Label.String = 'Number of gaits executed';
a.Layout.Tile = 'east';

title(t, 'Experimental results  of synthesized gaits for orange robot on black mat','FontSize',24)

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
    markers_x(iTrial, :) = gait_exp_2(iTrial).raw_data(1, 1:3:22);
    markers_y(iTrial, :) = gait_exp_2(iTrial).raw_data(1, 2:3:23);
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
