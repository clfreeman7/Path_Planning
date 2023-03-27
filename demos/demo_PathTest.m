% 
% Script: demo_PathTest.m
%
% Description: 
%   Demonstrate how to process the raw experimental data from the visual
%   tracking of paths (combinations of gaits).


% [0] == Script setup
clear; clc; close all
show_markers = false;

% Add dependencies to classpath
addpath('../');
filename = 'data/visualtracking/orange MTA4 threaded/Trial_28.mat';
% 
% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

% [1] == Load data for the path tests and the gait library.
raw_data = load(filename).tracking_data;
path_sequence.gait_names = load(filename).performed_seq;
path_sequence.gait_durations = load(filename).performed_num_cycle;
gait_library = load('data/gait_lib_threaded.mat').gait_lib_threaded;
%raw_data = load(paths{1}).tracking_data; 

% [2] == Extract and define parameters for PathTest() objects.
% Define experimental parameters after investigating video. 
frame_start_list = 2;      % for orange robot

% Find marker order by investigating first frame.
marker_order = [2 4 3 1];
% Define robot.
path_test.params.robot_name = 'orange threaded';
path_test.params.substrate = 'black mat';
path_test.params.n_markers = 4;
path_test.params.frame_1 = frame_start_list;


% Define theta = 0 for the robot based on the ground truth for the gait
% library calculation:
reference_data = load('data/visualtracking/orange MTA4 threaded/B_60_32SR_NF_L_1.mat').tracking_data;
ref_marker_order = [2 3 4 1];
markers_x(1, :) = reference_data(1, 1:3:path_test.params.n_markers*3-2);
markers_y(1, :) = reference_data(1, 2:3:path_test.params.n_markers*3-1);
centroid(:, :, 1) = mean([markers_x(1, :); markers_y(1, :)], 2);
% Move initial position to (0,0) origin.
reference_markers = [markers_x(1, ref_marker_order); markers_y(1, ref_marker_order);]...
                    - centroid(:, :, 1);

 markers_x(2, :) = raw_data(1, 1:3:path_test.params.n_markers*3-2);
    markers_y(2, :) = raw_data(1, 2:3:path_test.params.n_markers*3-1);
    centroid(:, :, 2) = mean([markers_x(2, :); markers_y(2, :)], 2);

    % Move initial position to (0,0) origin.
    shifted_markers = [markers_x(2, marker_order);
                       markers_y(2, marker_order)]...
                       - centroid(:, :, 2);

    % Find orientation of subsequent trials w.r.t. first trial GCS.
    [regParams,~,~] = absor(reference_markers, shifted_markers);
    path_test.params.R_1 = [regParams.R zeros(2,1); 0 0 1];
path_test.params.R_1 = eye(3);



path_1 = offlineanalysis.PathTest( raw_data, path_sequence, gait_library,  path_test.params)
path_1.plot

tested_gaits = gaitdef.Gait.empty(0,length(path_1.gait_names));
for i = 1:length(path_1.gait_names)
    gt_params = path_test.params;
    gt_params.R_1 = [];
    tested_gaits(i) = gaitdef.Gait(path_1.gait_tests{2*i-1},gt_params);
    tested_gaits(i).gait_name = path_1.gait_names(i);
end
figure
predicted_1 = offlineanalysis.PathPredict( path_sequence, gait_library, path_test.params);
predicted_1.plot

if show_markers

    figure
    
    % Plot first image of experiment video.
    pictureName = 'data/visualtracking/orange MTA4 threaded/firstframe_Trial_27.jpg';
    pic = imread(pictureName);
    nexttile;
    imshow(pic)
    hold on
    
    % Plot labeled (i.e., numbered) markers on top of image.
    markers_x = raw_data(1, 1:3:path_test.params.n_markers*3-2);
    markers_y = raw_data(1, 2:3:path_test.params.n_markers*3-1);
%     if iTrial ~=5
    for m = 1:4
        text(markers_x(1, m), 1080 - markers_y(1, m), num2str(m),'Color','white','FontSize',14);
%     end
    end
    
    % Adjust size.
    xlim([min(markers_x)-200, max(markers_x)+200]);
    ylim([min(1080 - markers_y) - 200, max(1080 - markers_y)+200]);
    
    

end

