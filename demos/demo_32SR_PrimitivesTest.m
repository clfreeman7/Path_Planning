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
addpath('data/visualtracking/orange MTA4 threaded');
load('data/visualtracking/state_order_NS.mat')
% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Define experimental parameters after investigating video. 
frame_start_list = [234, 102, 109, 106, 120];

% Find marker order by investigating first frame.
marker_order_list = [1 3 4 2;
                     1 3 4 2;
                     1 3 4 2;
                     2 3 4 1;
                     1 4 3 2];
show_markers = false;

for i = 1:5
    % Define robot.
    exp_32SR(i).params.robot_name = 'orange';
    exp_32SR(i).params.substrate = 'black mat';
    
    exp_32SR(i).params.n_markers = 4;
    exp_32SR(i).params.pixel_length = 1/8.6343;     % cm per pixel
    
    % Extract and store first frame information.
    exp_32SR(i).params.frame_1 = frame_start_list(i);

    % Look at image to determine marker order.
    exp_32SR(i).params.marker_order = marker_order_list(i, :);
    % Extract and store raw data from each trial.
    filename = ['tracked_data_E', num2str(i),'.mat'];
    exp_32SR(i).raw_data = load(filename).tracking_data;  
end
for iTrial = 1:5
    % Find initial rotation matrix for each trial to have consistent fixed frame.
    markers_x(iTrial, :) = exp_32SR(iTrial).raw_data(1, 1:3:exp_32SR(i).params.n_markers*3-2);
    markers_y(iTrial, :) = exp_32SR(iTrial).raw_data(1, 2:3:exp_32SR(i).params.n_markers*3-1);
    centroid(:, :, iTrial) = mean([markers_x(iTrial, :); markers_y(iTrial, :)], 2);
    if iTrial == 1
        reference_markers = [markers_x(iTrial, exp_32SR(iTrial).params.marker_order);
                             markers_y(iTrial, exp_32SR(iTrial).params.marker_order);]...
                             - centroid(:, :, iTrial);
    else
        shifted_markers = [markers_x(iTrial, exp_32SR(iTrial).params.marker_order);
                           markers_y(iTrial, exp_32SR(iTrial).params.marker_order)]...
                           - centroid(:, :, iTrial);
        % Find orientation of subsequent trials w.r.t. first trial GCS.
         [regParams,Bfit,ErrorStats] = absor(reference_markers, shifted_markers);
         exp_32SR(iTrial).params.R_1 = [regParams.R zeros(2,1); 0 0 1];
    end
end

%% [2] == Instantiate experimental motion primitive data analysis and plot
% results.

% Set up figure. 
figure(1)
t = tiledlayout(2, 3);

for i = 1:5
    all_trials(i) = offlineanalysis.PrimitivesTest(exp_32SR(i).raw_data, exp_32SR(i).params, state_order_NS(i,:));
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

save data/32SR_motion_primitives.mat motion_primitive_data


% [4] == Compare all Euler Tour data with the sheathed counterparts.
motion_primitive_data_S = load('data/experiment_2_motion_primitives_corrected.mat').motion_primitive_data;
figure('Position',[100 300 1500 900])
t_S = plot_all_deviations(motion_primitive_data_S);
%title(t_S, 'Motion primitive variation for orange robot on black mat with sheath tether')

motion_primitive_data_NS = load('data/NS_motion_primitives.mat').motion_primitive_data;
figure('Position',[100 300 1500 900])
t_NS = plot_all_deviations(motion_primitive_data_NS);
%title(t_NS, 'Motion primitive variation for orange robot on black mat with no sheath tether (28 AWG)')

motion_primitive_data_NS_32 = load('data/NS_32_motion_primitives.mat').motion_primitive_data;
figure('Position',[100 300 1500 900])
t_NS_32 = plot_all_deviations(motion_primitive_data_NS_32);
%title(t_NS, 'Motion primitive variation for orange robot on black mat with no sheath tether')


figure('Position',[100 300 1500 900])
t_32SR = plot_all_deviations(motion_primitive_data);
%title(t_32SR, 'Motion primitive variation for orange robot on black mat with slip ring tether')

%% [5] == Instantiate GaitPredict() objects for each gait permutation.

% [6] == Compare all Gait data with the sheathed counterparts.


if show_markers
    figure
    tiledlayout(2,3)
    for i = 1:5
    
    % Plot first image of experiment video.
    file_name = ['firstframe_Trial_Euler_', num2str(i), '.jpg'];
    pic = imread(file_name);
    nexttile;
    imshow(pic)
    hold on
    
    % Plot labeled (i.e., numbered) markers on top of image.
    markers_x(i, :) = exp_32SR(i).raw_data(1, 1:3:exp_32SR(i).params.n_markers*3-2);
    markers_y(i, :) = exp_32SR(i).raw_data(1, 2:3:exp_32SR(i).params.n_markers*3-1);
%     if iTrial ~=5
    for m = 1:exp_32SR(i).params.n_markers
        text(markers_x(i, m), 1080-markers_y(i, m), num2str(m),'Color','white','FontSize',14);
%     end
    end
    
    % Adjust size.
     xlim([min(markers_x(i, :))-200, max(markers_x(i, :))+200]);
     ylim([min(1080 - markers_y(i, :)) - 200, max(1080 - markers_y(i, :))+200]);
    
    
end
end

function t = plot_all_deviations(mp_data)
    avg_motions = squeeze(mean(mp_data, 1))';
    avg_motions(3,:) = rad2deg(avg_motions(3,:));
    var_motions = squeeze(var(mp_data, 0, 1))';
    stand_devs = var_motions.^(.5);
    stand_devs(3,:) = rad2deg(stand_devs(3,:));
    t = tiledlayout(3, 1);
    xlabel(t, 'Motion Primitive Index','FontSize',20);
    t.TileSpacing = 'tight';
    t.Padding = 'tight';
        for i = 1:3
        nexttile;
        x = 1:length(avg_motions);
        xlim([0 240])
        switch i 
            case 1
                %title('X-axis translation for motion primitives')
                ylabel('X-axis translation (cm)','FontSize',20)
                xticks(0:40:240)
                ax = gca;
                ax.XAxis.Visible = 'off';
                set(gca,'FontSize',16);
                ylim([-2.5 2.5])
            case 2 
                %title('Y-axis translation for motion primitives')
                ylabel('Y-axis translation (cm)','FontSize',20)
                xticks(0:40:240)
                ax = gca;
                ax.XAxis.Visible = 'off';
                set(gca,'FontSize',16);
                ylim([-2.5 2.5])

            case 3 
               % title('Rotation for motion primitives')
                ylabel('Rotation (deg)','FontSize',20)
                set(gca,'FontSize',16);
                ylim([-15 15])
                xticks(0:40:240)
        end
        grid on 
        hold on
        yline(0)
        plus_sd = avg_motions(i,:)+stand_devs(i,:);
        minus_sd = avg_motions(i,:)-stand_devs(i,:);
        % plot(x,plus_sd,'r')
        % plot(x,minus_sd,'r')
        p=fill([x fliplr(x)],[plus_sd fliplr(minus_sd)],'r','FaceAlpha',0.25);
        p.EdgeColor = 'none';
        plot(x,avg_motions(i,:),'r','LineWidth',1)
        end
end

