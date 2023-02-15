% MTA3 Gait Experiments Data Analysis for Three Surfaces
%
% Script: gait_analysis_MTA3.m
%
% Dependencies:
%   +demos/data/visualtracking
%   +offlineanalysis/GaitTest
%   +gaitdef/Gait
%   
% Description:
%  This script analyzes all of the data from the MTA3 gait experiments and 
% saves the results as structs. There are no plots in this script; it is 
% meant to act as a helper so that plotting scripts don't have to reanalyze
% the data.

%% [0] == Script setup
clear; clc; close all

% Add dependencies to classpath
cd 'D:\clfreeman7\Documents\GitHub\IROS2022\demos';
addpath('../');
addpath('data/visualtracking');

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
set(0, 'DefaultAxesFontSize', 18);

% Decide which figures to show.
show_markers = false;             % Labels the markers on the robot to determine marker order for data alignment.
show_gait_sequences = false;      % Visualizes the discrete robot configurations in each gait sequence.

%% [1] == Extract and define parameters for each gait experiment.

% From Black Mat (20230127/20230129) Experiments:
% Writing the gait names.
mat_T = 'ABC';    % Translation gaits on the black mat.
mat_R = 'DEF';      % Rotation gaits on the black mat.
ad_hoc = 'GHI';
names_bm = [mat_T mat_R ad_hoc];

% Preallocate space by inputting the number of experiments. 
n_exps_bm = 18;
gait_names = cell(n_exps_bm,1);

% Defining the experiment data file location.
exp_location{1} = 'data/visualtracking/MTA3/Black_Mat/';

for i = 1:9
    % (L): Robot marker pointed to left, tether on left.
    vid_name = ['3',names_bm(i),'_60_L'];
    gait_names{2*i-1} = [vid_name,'.mat'];

    % (Lf): Robot marker pointed to right, tether on left ("left flipped").
    vid_name = ['3',names_bm(i),'_60_Lf'];
    gait_names{2*i} = [vid_name,'.mat'];
end

% Investigate gcf videos to define when motion begins. 
frame_start_list(1:2:18) = [130,107,103,115,81,124,111,109,108];     % Black Mat L
frame_start_list(2:2:18) = [98,105,121,130,82,106,115,121,124];      % Black Mat Lf

% From Whiteboard (20230127/20230129) Experiments:
% Writing the gait names.
whiteboard_T = 'JKL';    % Translation gaits on the black mat.
whiteboard_R = 'MNO';      % Rotation gaits on the black mat.
names_wb = [whiteboard_T whiteboard_R mat_T(1) mat_R(1) ad_hoc];

% Preallocate space by inputting the number of experiments. 
n_exps_wb = 14;
gait_names = [gait_names; cell(n_exps_wb,1)];

% Defining the experiment data file location.
exp_location{2} = 'data/visualtracking/MTA3/Whiteboard/';

for i = 1:8
    % (L): Robot marker pointed to left, tether on left.
    vid_name = ['3',names_wb(i),'_60_L'];
    gait_names{n_exps_bm + i} = [vid_name,'.mat'];
end

for i = 1:3
    % (L): Robot marker pointed to left, tether on left.
    vid_name = ['3',names_wb(8 + i),'_60_L'];
    gait_names{n_exps_bm + 8 + 2*i-1} = [vid_name,'.mat'];

    % (Lf): Robot marker pointed to right, tether on left ("left flipped").
    vid_name = ['3',names_wb(8 + i),'_60_Lf'];
    gait_names{n_exps_bm + 8 + 2*i} = [vid_name,'.mat'];
end

% Investigate gcf videos to define when motion begins. 
frame_start_list(19:26) = [38,35,34,33,52,30,34,30];      % Whiteboard L (not ad-hoc)
frame_start_list(27:32) = [117,124,118,106,101,123];      % Whiteboard ad-hoc (L and Lf)

% From Carpet (20230201) Experiments:
% Writing the gait names.
carpet_T = 'QRS';    % Translation gaits on the black mat.
carpet_R = 'TUV';      % Rotation gaits on the black mat.
names_ct = [carpet_T carpet_R mat_T(1) mat_R(1) ad_hoc];

% Preallocate space by inputting the number of experiments. 
n_exps_ct = 11;
gait_names = [gait_names; cell(n_exps_ct,1)];

% Defining the experiment data file location.
exp_location{3} = 'data/visualtracking/MTA3/Carpet/';

for i = 1:11
    % (L): Robot marker pointed to left, tether on left.
    vid_name = ['3',names_ct(i),'_60_L'];
    gait_names{n_exps_bm + n_exps_wb + i} = [vid_name,'.mat'];
end

% Investigate gcf videos to define when motion begins. 
frame_start_list(33:43) = [153,142,150,146,153,138,144,146,168,148,151];     % Black Mat L

n_gaits = length(gait_names);

% Find marker order by investigating first frame.
marker_order_list = repmat([1 3 5 6 4 2],n_gaits, 1);
marker_order_list([3,9,11,17,21,22,23,25,27,31,41,42],:) = repmat([1 2 4 6 5 3],12,1);
marker_order_list([24,29,34],:) = repmat([1 2 5 6 4 3],3,1);

marker_order_list([5,36,39,40],:) = repmat([2 1 4 6 5 3],4,1);
marker_order_list(15,:) = [2 3 5 6 4 1];

marker_order_list(4,:) = [5 4 2 1 3 6];
marker_order_list(18,:) = [5 6 3 1 2 4];

marker_order_list([2,10,12,16,30,32],:) = repmat([6 4 2 1 3 5],6,1);
marker_order_list([6,8,28],:) = repmat([6 5 2 1 3 4],3,1);
marker_order_list(14,:) = [6 5 3 1 2 4];

%% [2] == Build Experiment Matrix
if isrow(gait_names)
    gait_names = gait_names';
end
n_gaits = length(gait_names);

% Extract characteristics of each gait test.
file_names = split(gait_names, '.');
gait_characteristics = cell(n_gaits, 7);
for i = 1:n_gaits
    gait_characteristics(i, 1) = num2cell(i);
    n_underscores = count(gait_names{i}, '_');
    if n_underscores == 2
        gait_characteristics(i, [2,3,6]) = split(file_names(i,1), '_');
        gait_characteristics(i, 4) = cellstr('NS');
        gait_characteristics(i, 5) = cellstr('NF');
        if ismember(i,1:18)
            gait_characteristics(i, 7) = cellstr('Black Mat');
        elseif ismember(i,19:32)
            gait_characteristics(i, 7) = cellstr('Whiteboard');
        else 
            gait_characteristics(i, 7) = cellstr('Carpet');
        end
    else
        gait_characteristics(i, 2:7) = split(file_names(i,1), '_');
    end
end
characteristics = ["Experiment", "Gait", "#Cycles", "Tether", ...
              "Protocol", "Placement", "Surface"];
% Build table.
experiments = cell2table(gait_characteristics, "VariableNames", ...
              characteristics);
sorted_exps = sortrows(experiments, characteristics([2, 4, 5, 6, 7]));
% Construct titles. 
title_components = string(gait_characteristics);
title_components(strcmp(experiments.Protocol, 'F'), 5) = 'following';
title_components(strcmp(experiments.Protocol, 'NF'), 5) = 'not following';
title_components(strcmp(experiments.Placement, 'L'), 6) = 'left';
title_components(strcmp(experiments.Placement, 'Lf'), 6) = 'left (flipped)';
title_components = [repmat(["Experiment" ":" "cycles of Gait" "with" "tether (" "," ")," "on"], n_gaits, 1) title_components];
title_matrix = title_components(:, [1,9,2,11,3,10,4,12,5,14,6,13,7,8,15]);
exp_title = join(title_matrix);

% Initialize the stability experiment struct. 
MTA3 = struct('params', [], 'raw_data', []);
gait_sequences = cell(n_gaits,1);

% Extract data.
for i = 1:n_gaits
    % Define robot / experiment parameters.
    MTA3(i).params.robot_name = 'MTA3';
    if ismember(i,1:18)
        MTA3(i).params.substrate = 'black mat';
        filename = [exp_location{1} gait_names{i}];
    elseif ismember(i,19:32)
        MTA3(i).params.substrate = 'whiteboard';
        filename = [exp_location{2} gait_names{i}];
    else
        MTA3(i).params.substrate = 'carpet'; 
        filename = [exp_location{3} gait_names{i}];
    end
    MTA3(i).params.n_markers = 6;
    MTA3(i).params.n_unique_states = 8;
    MTA3(i).params.transition_time = .55;
    MTA3(i).params.pixel_length = 1/8.6343;     % cm per pixel
    if ismember(i,18:26)
        MTA3(i).params.pixel_length = MTA3(i).params.pixel_length/480*1080;     
    end

    % Define number of gait cycles run.
    MTA3(i).params.n_cycles= str2double(gait_characteristics{i,3}); 
         % accounts for pause after first cycle in some of the videos

    % Extract and store first frame information.
    MTA3(i).params.frame_1 = frame_start_list(i);
    MTA3(i).params.marker_order = marker_order_list(i,:);

    MTA3(i).raw_data = load(filename).tracking_data;  
    switch experiments.Gait{i}(2)
        case "A"
            gait_sequences{i} = [6,3];
        case "B"
            gait_sequences{i} = [8,2,1];
        case "C"
            gait_sequences{i} = [6,4,5,7,1];
        case "D"
            gait_sequences{i} = [4,2,1];
        case "E"
            gait_sequences{i} = [3,4,2,6,7,1];
        case "F"
            gait_sequences{i} = [7,5,6,4,2,1];
        case "G"
            gait_sequences{i} = [1,8];
        case "H"
            gait_sequences{i} = [2,3,5];
        case "I"
            gait_sequences{i} = [5,8,4,1];
        case "J"
            gait_sequences{i} = [6,4,5,2]; 
        case "K"
            gait_sequences{i} = [4,7,5,1];
        case "L"
            gait_sequences{i} = [5,3];
        case "M"
            gait_sequences{i} = [5,1];
        case "N"
            gait_sequences{i} = [1,4,2,6];
        case "O"
            gait_sequences{i} = [5,7,6,2];
        case "Q"
            gait_sequences{i} = [7,3];
        case "R"
            gait_sequences{i} = [7,4,3];
        case "S"
            gait_sequences{i} = [4,5,3,2];
        case "T"
            gait_sequences{i} = [2,6,5,3,8,4];
        case "U"
            gait_sequences{i} = [3,8,7,6,4,5,2];
        case "V"
            gait_sequences{i} = [7,4,2,6,5,3,1];
    end
end

%% [3] - Find initial orientation for each experiment. 

% Define first experiment first frame orientation as theta = 0. 
markers_x(1, :) = MTA3(1).raw_data(1, 1:3:MTA3(1).params.n_markers*3-2);
markers_y(1, :) = MTA3(1).raw_data(1, 2:3:MTA3(1).params.n_markers*3-1);
centroid(:, :, 1) = mean([markers_x(1, :); markers_y(1, :)], 2);
% Move initial position to (0,0) origin.
reference_markers = [markers_x(1, marker_order_list(1,:)); markers_y(1, marker_order_list(1,:));]...
                    - centroid(:, :, 1);
% For each trial, rotate data to align first frame global orientations.
 for i = 1:n_gaits
    % Find initial rotation matrix for each trial to have consistent fixed frame.
    markers_x(i, :) = MTA3(i).raw_data(1, 1:3:MTA3(i).params.n_markers*3-2);
    markers_y(i, :) = MTA3(i).raw_data(1, 2:3:MTA3(i).params.n_markers*3-1);
    centroid(:, :, i) = mean([markers_x(i, :); markers_y(i, :)], 2);

    % Move initial position to (0,0) origin.
    shifted_markers = [markers_x(i, MTA3(i).params.marker_order);
                       markers_y(i, MTA3(i).params.marker_order)]...
                       - centroid(:, :, i);

    % Find orientation of subsequent trials w.r.t. first trial GCS.
    [regParams,~,~] = absor(reference_markers, shifted_markers);
    MTA3(i).params.R_1 = [regParams.R zeros(2,1); 0 0 1];
 end


%% [4] = Instantiate GaitTest() and gaitdef.Gait() objects for each experiment.

%This analyzes the data from each trial to find motion primitive twist information.
% Instantiate objects for each gait tested. 
all_gaits = offlineanalysis.GaitTest.empty(0,n_gaits); 
gait_defs = gaitdef.Gait.empty(0,n_gaits);
tail_idx = cell(n_gaits, 1);
head_idx = cell(n_gaits, 1);
delta_poses = cell(n_gaits,1);
global_theta = cell(n_gaits,1);
twists = cell(n_gaits,1);
cent_rot = cell(n_gaits,1);
rad_curv = cell(n_gaits,1);
orientation = cell(n_gaits,1);
lin_vel{i} = cell(n_gaits,1);
avg_lin_speed = zeros(1,n_gaits);
err_lin_speed = zeros(1,n_gaits);
avg_rot_speed = zeros(1,n_gaits);
err_rot_speed = zeros(1,n_gaits);

% Calculate twists for each gait experiment.
for i = 1:n_gaits
    all_gaits(i) = offlineanalysis.GaitTest(MTA3(i).raw_data, ...
                                                 gait_sequences{i}(1,:), ...
                                                 MTA3(i).params);
    gait_defs(i) = gaitdef.Gait(all_gaits(i), MTA3(i).params);
    % Manually find the twists. 
    % Extract gait / experiment details. 
    n_cycles = all_gaits(i).n_cycles;
    len_gait = all_gaits(i).len_gait;
     for j = 1:n_cycles
         % Record indexes for each gait tail and head. 
         tail_idx{i}(1,j) = all_gaits(i).keyframes(len_gait*(j-1)+2);
         head_idx{i}(1,j) = all_gaits(i).keyframes(len_gait*j+2);

         % Find change in poses w.r.t. global inertial frame for each gait. 
         delta_poses{i}(:,j) = all_gaits(i).poses(:, head_idx{i}(1,j)) - all_gaits(i).poses(:, tail_idx{i}(1,j));
         %delta_poses{i}(1:2,j) = all_gaits(i).pixel_length*delta_poses{i}(1:2,j);

         % Find global orientation of robot at each gait tail. 
         global_theta{i}(:,j) = all_gaits(i).poses(3, tail_idx{i}(1,j));

         % Convert from global frame to body frame.  
         rot_mat =  [cos(global_theta{i}(1,j)) -sin(global_theta{i}(1,j)); sin(global_theta{i}(1,j)) cos(global_theta{i}(1,j))];
         delta_poses{i}(1:2,j) = rot_mat'*delta_poses{i}(1:2,j);
         R =  [cos(delta_poses{i}(3,j)) -sin(delta_poses{i}(3,j)); sin(delta_poses{i}(3,j)) cos(delta_poses{i}(3,j))];
         % Convert to twists.
         twists{i}(:,j)  = gait_defs(i).delta_pose_2_twist(delta_poses{i}(:,j), all_gaits(i).transition_time*len_gait);

         % Find instantaneous center of rotation.
         cent_rot{i}(:,j) = (eye(2) - R)\delta_poses{i}(1:2, j);

         % Find instantaneous radius of curvature.
         rad_curv{i}(1,j) = norm(cent_rot{i}(:,j));
     end

     lin_vel{i} = (twists{i}(1,:).^2+twists{i}(2,:).^2).^.5;
     avg_lin_speed(1,i) = mean(lin_vel{i});
     err_lin_speed(1,i) = var(lin_vel{i}).^.5;
     avg_rot_speed(1,i) = abs(rad2deg(mean(twists{i}(3,:))));
     err_rot_speed(1,i) = rad2deg(var(twists{i}(3,:)).^.5);
end
speed_data = [avg_lin_speed; err_lin_speed; avg_rot_speed; err_rot_speed];

%% [5] = Convert all custom class objects into structs for easy plotting. 
for i = n_gaits:-1:1
    GaitTest_struct(1,i) = struct(all_gaits(i));
    gait_defs(i).tether_placement = string(experiments.Placement(i));
    gait_defs(i).gait_name = string(experiments.Gait(i));
    gaitdef_Gait_struct(1,i) = struct(gait_defs(i));
end

%% [6] = Save all data. 
% Note: If you open this .mat file in an environment with the proper
% gaitdef.Gait and offlineanalysis.GaitTest class definitions, you can use
% the Gait() and GaitTest() objects. If not, you should use the
% GaitTest_struct and gaitdef_Gait_struct structure arrays. 

save gait_analysis_MTA3.mat
