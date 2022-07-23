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
load('data/visualtracking/state_order_2.mat')
% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Define experimental parameters after investigating video. 
frame_start_list = [76, 286, 257, 404, 808];

% Find marker order by investigating first frame.
marker_order_list = [7 8 5 3 1 2 4 6;
                     7 8 5 3 2 1 4 6;
                     8 7 4 3 1 2 5 6;
                     8 7 5 3 1 2 4 6;
                    7 8 5 3 2 1 4 6];
show_markers = true;

for i = 1:5
    % Define robot.
    exp_2(i).params.robot_name = 'orange';
    exp_2(i).params.substrate = 'black mat';
    
    % Define pixel length.
    exp_2(i).params.pixel_length = .23898;
    
    % Extract and store first frame information.
    exp_2(i).params.frame_1 = frame_start_list(i);

        % Look at image to determine marker order.
    exp_2(i).params.marker_order = marker_order_list(i, :);
    % Extract and store raw data from each trial.
    filename = ['data/visualtracking/Euler', ' ', num2str(i+6), '_corrected.mat'];
    exp_2(i).raw_data = load(filename).all_pt;  
end
for iTrial = [3, 1, 2, 4, 5]
    % Find initial rotation matrix for each trial to have consistent fixed frame.
        markers_x(iTrial, :) = exp_2(iTrial).raw_data(1, 1:3:22);
    markers_y(iTrial, :) = exp_2(iTrial).raw_data(1, 2:3:23);
    centroid(:, :, iTrial) = mean([markers_x(iTrial, :); markers_y(iTrial, :)], 2);
    if iTrial == 3
        reference_markers = [markers_x(iTrial, exp_2(iTrial).params.marker_order);
                             markers_y(iTrial, exp_2(iTrial).params.marker_order);]...
                             - centroid(:, :, iTrial);
    else
        shifted_markers = [markers_x(iTrial, exp_2(iTrial).params.marker_order);
                           markers_y(iTrial, exp_2(iTrial).params.marker_order)]...
                           - centroid(:, :, iTrial);
        % Find orientation of subsequent trials w.r.t. first trial GCS.
         [regParams,Bfit,ErrorStats]=absor(reference_markers, shifted_markers);
         exp_2(iTrial).params.R_1 = [regParams.R zeros(2,1); 0 0 1];
    end
end

% [2] == Instantiate experimental motion primitive data analysis and plot
% results.

% Set up figure. 
figure(1)
t = tiledlayout(2, 3);

for i = 1:5
    all_trials(i) = offlineanalysis.PrimitivesTest(exp_2(i).raw_data, exp_2(i).params, state_order_2(i,:));
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

save data/experiment_2_motion_primitives_corrected.mat motion_primitive_data


if show_markers
    figure
    tiledlayout(2,3)
    for iTrial = 1:5
    
    
    % Plot first image of experiment video.
    pictureName = sprintf('data/visualtracking/firstframe%02d.jpg', iTrial+6);
    pic = imread(pictureName);
    nexttile;
    imshow(pic)
    hold on
    
    % Plot labeled (i.e., numbered) markers on top of image.
    markers_x(iTrial, :) = exp_2(iTrial).raw_data(1, 1:3:22);
    markers_y(iTrial, :) = exp_2(iTrial).raw_data(1, 2:3:23);
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
