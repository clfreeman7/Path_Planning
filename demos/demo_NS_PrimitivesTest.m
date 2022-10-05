% 
% Script: demo_NS_PrimitivesTest.m
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
load('data/visualtracking/state_order_NS.mat')
% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Define experimental parameters after investigating video. 
frame_start_list = [173, 73, 74, 226, 111];

% Find marker order by investigating first frame.
marker_order_list = [2 4 3 1;
                     1 3 4 2;
                     2 3 4 1;
                     2 4 3 1;
                     2 4 3 1];
show_markers = false;

for i = 1:5
    % Define robot.
    exp_NS(i).params.robot_name = 'orange';
    exp_NS(i).params.substrate = 'black mat';
    
    exp_NS(i).params.n_markers = 4;
    exp_NS(i).params.pixel_length = 1/8.6343;     % cm per pixel
    
    % Extract and store first frame information.
    exp_NS(i).params.frame_1 = frame_start_list(i);

        % Look at image to determine marker order.
    exp_NS(i).params.marker_order = marker_order_list(i, :);
    % Extract and store raw data from each trial.
    filename = ['data/visualtracking/euler_NS_', num2str(i),'.mat'];
    exp_NS(i).raw_data = load(filename).tracking_data;  
end
for iTrial = 1:5
    % Find initial rotation matrix for each trial to have consistent fixed frame.
        markers_x(iTrial, :) = exp_NS(iTrial).raw_data(1, 1:3:exp_NS(i).params.n_markers*3-2);
    markers_y(iTrial, :) = exp_NS(iTrial).raw_data(1, 2:3:exp_NS(i).params.n_markers*3-1);
    centroid(:, :, iTrial) = mean([markers_x(iTrial, :); markers_y(iTrial, :)], 2);
    if iTrial == 1
        reference_markers = [markers_x(iTrial, exp_NS(iTrial).params.marker_order);
                             markers_y(iTrial, exp_NS(iTrial).params.marker_order);]...
                             - centroid(:, :, iTrial);
    else
        shifted_markers = [markers_x(iTrial, exp_NS(iTrial).params.marker_order);
                           markers_y(iTrial, exp_NS(iTrial).params.marker_order)]...
                           - centroid(:, :, iTrial);
        % Find orientation of subsequent trials w.r.t. first trial GCS.
         [regParams,Bfit,ErrorStats]=absor(reference_markers, shifted_markers);
         exp_NS(iTrial).params.R_1 = [regParams.R zeros(2,1); 0 0 1];
    end
end

%% [2] == Instantiate experimental motion primitive data analysis and plot
% results.

% Set up figure. 
figure(1)
t = tiledlayout(2, 3);

for i = 1:5
    all_trials(i) = offlineanalysis.PrimitivesTest(exp_NS(i).raw_data, exp_NS(i).params, state_order_NS(i,:));
    nexttile;
    all_trials(i).plot;
    title("Trial " +num2str(i))
    % Extract data to store in a more convenient way for gait synthesis.
    delta_x_data(i, :) = all_trials(i).delta_x;
    delta_y_data(i, :) = all_trials(i).delta_y;
    delta_theta_data(i, :) = all_trials(i).delta_theta;
end
% Add legend.
%lgd.Layout.Tile = 'north';
%lgd.Orientation = 'horizontal';

% Add colorbar.
a = colorbar;
a.Label.String = 'Number of motion primitives executed';
%a.Layout.Tile = 'east';
%lgd = legend('Continuous robot position', 'Actual keyframe positions', ...
 %                 'Keyframe positions reconstructed from gait twists');

title(t, 'Motion primitive exploration (Euler tours) for no sheath orange robot on black mat','FontSize',24)


%% [3] == Concatenate all motion primitive motion data into a 3D matrix.
motion_primitive_data = cat(3, delta_x_data, delta_y_data, delta_theta_data);

save data/NS_motion_primitives.mat motion_primitive_data


% [4] == Compare all Euler Tour data with the sheathed counterparts.
motion_primitive_data_S = load('data/experiment_2_motion_primitives_corrected.mat').motion_primitive_data;
figure(2)
plot_all_deviations(motion_primitive_data_S)
title('Motion primitive variation for sheathed orange robot on black mat')

figure(3)
plot_all_deviations(motion_primitive_data)
title('Motion primitive variation for no sheath orange robot on black mat')

%% [5] == Instantiate GaitPredict() objects for each gait permutation.

% [6] == Compare all Gait data with the sheathed counterparts.


if show_markers
    figure
    tiledlayout(2,3)
    for i = 1:5
    
    % Plot first image of experiment video.
    file_name = ['data/visualtracking/firstframe_euler_NS_0', num2str(i), '.jpg'];
    pic = imread(file_name);
    nexttile;
    imshow(pic)
    hold on
    
    % Plot labeled (i.e., numbered) markers on top of image.
    markers_x(i, :) = exp_NS(i).raw_data(1, 1:3:exp_NS(i).params.n_markers*3-2);
    markers_y(i, :) = exp_NS(i).raw_data(1, 2:3:exp_NS(i).params.n_markers*3-1);
%     if iTrial ~=5
    for m = 1:exp_NS(i).params.n_markers
        text(markers_x(i, m), 1080-markers_y(i, m), num2str(m),'Color','white','FontSize',14);
%     end
    end
    
    % Adjust size.
     xlim([min(markers_x(i, :))-200, max(markers_x(i, :))+200]);
     ylim([min(1080 - markers_y(i, :)) - 200, max(1080 - markers_y(i, :))+200]);
    
    
end
end

function plot_all_deviations(mp_data)
    avg_motions = squeeze(mean(mp_data, 1))';
    avg_motions(3,:) = rad2deg(avg_motions(3,:));
    var_motions = squeeze(var(mp_data, 0, 1))';
    var_motions(3,:) = rad2deg(var_motions(3,:));
    stand_devs = var_motions.^(.5);
    tiledlayout(3, 1)
        for i = 1:3
        nexttile;
        x = 1:length(avg_motions);

        xlabel('Motion Primitive Index')

        switch i 
            case 1
                title('X-axis translation for motion primitives')
                ylabel('X-axis translation (cm)')
            case 2 
                title('Y-axis translation for motion primitives')
                ylabel('Y-axis translation (cm)')
            case 3 
                title('Rotation for motion primitives')
                ylabel('Rotation (degrees)')
        end

        grid on 
        grid minor
        hold on
        plus_sd = avg_motions(i,:)+stand_devs(i,:);
        minus_sd = avg_motions(i,:)-stand_devs(i,:);
        % plot(x,plus_sd,'r')
        % plot(x,minus_sd,'r')
        p=fill([x fliplr(x)],[plus_sd fliplr(minus_sd)],'r','FaceAlpha',0.2);
        p.EdgeColor = 'r';
        plot(x,avg_motions(i,:),'b')
        end
end
