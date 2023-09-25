clear all;
clc;

%[] Input video name for writing
vid_name = 'Trial_30';
%[] Initalize object to write tracked video frames
params.vwrite_tracked = VideoWriter(append(vid_name,'_gcf'),'MPEG-4');
open(params.vwrite_tracked);

%[] Initialize camera and camera properties
choose_cam = menu("Initialize camera",'Yes','No');
% Camera for tracking
if choose_cam == 1
    cam = webcam(1);
    cam.Resolution = '1920x1080';
    preview(cam);
end
choose_cam_prop = menu("Adjust Camera properties",'Yes','No');
if choose_cam_prop == 1
    cam = camera_properties_marker_1920_Blue(cam);
    cam.Resolution = '1920x1080';
end

%Distortion parameter
% distortion = load('D:\Arun Niddish\Vision\Visual Tracking\Callibration images(1920x1080)\cameraParams');   %Parameters for distortion correction for 1920x1080
distortion =load('cameraParams.mat');

pause(2);
setup_img = cam.snapshot;
setup_img = undistortImage(setup_img,distortion.cameraParams);

clear cam;
% setup_img = imread("world_5.jpg");

%[] World data's
world_params = world_data(setup_img);

obstacle_bin_img = world_params.obstacle_bin_img;
obstacle_bin_img = im2uint8(obstacle_bin_img);
robo_init_position = [mean(world_params.robo_init_position(:,1)) mean(world_params.robo_init_position(:,2))]; % In pixel
target_position = world_params.target_position;  % In pixel
obstaclesBB = world_params.obstaclesBB;   % Rectangular Bounding Box

%[] World parameters
pixel_to_cm = 8.5254;

% [0] == Script usage
% GAIT_LIBRARY_MAT = 'data/gait_library_5.mat';
GAIT_LIBRARY_MAT = 'data/gait_lib_threaded.mat';
MSORO_PGM = 'data/starfish1.pgm';

%[] Grid world properties
% gridS.dg = 1.565430166666667;     % grid size, or pixels-to-cm (cm/px) conversion
gridS.dg = (1920/pixel_to_cm)/336;    % grid size, or grids-to-cm (cm/px) conversion
gridS.cmin = [0, 0];              % physical world coordinates cooresponding to grid world bottom-left location
gridS.size = [336, 336];          % grid world dimensions [x, y]

% Cost map construction
rad_falloff = 23;             % radial decay constant for obstacle costs (grid units)
% tune this between ~20-25 to adjust % trajectory results

% Planning parameters
obstacle_threshold = 20;      % cost value at which a location is identified as obstacle
stop_radius = 23*gridS.dg;    % goal radius within which planning completes - in cm

% [1] == Script setup
addpath('../ivalibs/fastmarch');
addpath('../ivalibs/groups');
addpath('../ivalibs/control');
addpath('../');

set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

world_img = obstacle_bin_img;
start_pose = [robo_init_position(1,1)/pixel_to_cm ; robo_init_position(1,2)/pixel_to_cm ; 0];  % In cm
goal_position = target_position'/pixel_to_cm;   % In cm

% [3] == Configure trajectory planner
%  [3a] == Instantiate and configure RTGaitPlanner
params.gridS = gridS;
params.obstacle_threshold = obstacle_threshold;
params.radStop = stop_radius;
rtGaitPlanner = planning.RTGaitPlanner(params);

%  [3b] == Set planning scenario (world)
cf = rtGaitPlanner.setScenario( world_img, rad_falloff );

%  [3c] == Set MSoRo gaits for planning
gait_library = load(GAIT_LIBRARY_MAT).gait_lib_threaded;
rot_gaits = gait_library(1);     % rotational gaits
% trans_gaits = gait_library(2:3:5);
% trans_gaits = [gait_library(2) gait_library(4) gait_library(5)];
trans_gaits = gait_library(2:5);

rtGaitPlanner.setGaits( rot_gaits, trans_gaits );

% [4] == Plan discrete-time controlled trajectory
trajectory_plan = rtGaitPlanner.planTrajectory(SE2(start_pose(1:2), start_pose(3)), goal_position);
trajectory_data{1} = trajectory_plan;

%[2] Forward Kinematics of trajectory poses
SE2_script = @(theta) [cos(theta),-sin(theta); sin(theta),cos(theta)];
initial_position = start_pose(1:2,:);
initial_orientation = 0;
fk_pose_count = 0;
for i = 1:size(trajectory_plan.gait_names,2)
    gait_perform_seq_FK = string(trajectory_plan.gait_names(i));
    sequence_length = trajectory_plan.gait_durations(1,i);
    if gait_perform_seq_FK == "B"
        dp = gait_library(1,1).Delta_Pose(1:2,1);
        dp_theta = gait_library(1,1).Delta_Pose(3,1);
    end
    if gait_perform_seq_FK == "G"
        dp = gait_library(1,2).Delta_Pose(1:2,1);
        dp_theta = gait_library(1,2).Delta_Pose(3,1);
    end
    if gait_perform_seq_FK == "I"
        dp = gait_library(1,3).Delta_Pose(1:2,1);
        dp_theta = gait_library(1,3).Delta_Pose(3,1);
    end
    if gait_perform_seq_FK == "J"
        dp = gait_library(1,4).Delta_Pose(1:2,1);
        dp_theta = gait_library(1,4).Delta_Pose(3,1);
    end
    if gait_perform_seq_FK == "K"
        dp = gait_library(1,5).Delta_Pose(1:2,1);
        dp_theta = gait_library(1,5).Delta_Pose(3,1);
    end
    clear gait_perform_dir;
    if i == 1
        current_position = initial_position;
        current_orientation = initial_orientation;
        %         fk_pose(1,:) = [initial_position',initial_orientation];
        fk_pose(1,:) = initial_position';
        fk_theta(1,:) = current_orientation;
    end
    [new_position,current_orientation,orientation_theta] = FK_simulation_step(dp,dp_theta,sequence_length,current_position,current_orientation(end,end));
    current_position = new_position(:,end);
    %     fk_pose(i+1,:) = [new_position',current_orientation];
    fk_pose = [fk_pose' new_position];
    fk_pose = fk_pose';
    fk_theta = [fk_theta;orientation_theta];
end

% Obatining trajectory pose for plotting
trajectory_position = fk_pose';
fk_pose_data{fk_pose_count+1} = trajectory_position;
clear fk_pose;
clear fk_theta;

% Visualize result
%   Extract MSoRo outline
msoro_img_file = MSORO_PGM;
msoro_img_scaling = 35/525.3047;   % pixel-to-cm (i.e. cm/px) scaling for MSoRo image
num_outline_pnts = 500;
msoro_outline = planning.RTGaitPlanner.img2msoroOutline(msoro_img_file, msoro_img_scaling, num_outline_pnts);  % MSoRo outline coordinates (cm)

%   Plot trajectory /w MSoRo outline overlaid
fig_hdl = figure;
scen_props.image = cf;
scen_props.px2cm = gridS.dg;
scen_props.radStop = 50*scen_props.px2cm;
scen_props.startPose = start_pose;
scen_props.goalPosition = goal_position;
planning.RTGaitPlanner.plotTrajectory(trajectory_plan, scen_props, gait_library, fig_hdl, msoro_outline);
keyboard
input('Press <Enter> to continue');

%[] Tracking parameters
params.number_of_markers = 4;
params.overlay = 0;
params.stop_radius = stop_radius*pixel_to_cm;
params.target_position = target_position;
params.obstaclesBB = obstaclesBB;
params.pixel_to_cm = pixel_to_cm;
%params.PrevPt
%params.P0

% Initilaize object for online tracking
tracker_obj = OnlineTracking(params);
PrevPt = [];
P0 = [];
robo_centroid = [];

%[] Initiating object for class - Arduino
% msoro_robo = MSoRo_IO(params);
msoro_robo = MSoRo_IO();
%  MSoRo serial port configuration
port = 'COM11';
baud = 9600;
timeout = 120;

% Connect to arduino serial port
msoro_robo.connect(port,baud,timeout);
pause(3);

% Set gait motion model(s)
% gait_motion_models = load(GAIT_LIBRARY_MAT).gait_library_3([1, 3]);
% % gait_motion_models = load(GAIT_LIBRARY_MAT).gait_library_2([2, 5]);
%
% % Defining each gait sequence
% for i = 1:size(gait_motion_models,2)
%     gait_names(i,1) = gait_motion_models(i).gait_name;
%     gait_trans_seq(i,1) = {gait_motion_models(i).robo_states};
% end

% gait_names = cellstr(gait_names);
% gait_names = gait_names';
% gait_trans_seq = gait_trans_seq';


%[0] Defining gait definition manually
gait_trans_seq = {[3 15 1],[2 8 1],[9 12 1],[5 14 1],[16 7 5 11 14]};
gait_names = {'G','I','J','K','B'};

msoro_robo.define_gait(gait_trans_seq,gait_names);

%[] Modified trajectory data based on gait window size
gait_seq_window = 20;

% trajectory_gait_data = gait_sequence(trajectory_plan,gait_seq_window);
trajectory_gait_data = gait_sequence_tunnel(trajectory_plan,gait_seq_window);

gait_perform_seq = trajectory_gait_data.gait_perform_seq;
num_gait_cycles = trajectory_gait_data.num_gait_cycles;

input('Press <Enter> to start');

%[] Initialize --> Pollable Data Queue-(P)
P = parallel.pool.PollableDataQueue;
L = parallel.pool.PollableDataQueue;

%[] Initialize --> parfeval-(f)
parpool('local',1);
freq = 25;
% f = parfeval(@getFrameFromCamera,0,P,L,freq,distortion);
f = parfeval(@getFrameFromCamera,0,P,L,distortion,vid_name);

pause(1.5);


%[] Main script - Polling data's ; Other controls of the code

% Polling data; Tracking;
idx = 0;
jj = 1;
poll_data = 0;
tracking_window = 10;
check_complete = "#Completed";  % Flag word to check gait completion status from arduino
curr_time = 0;
overall_switch = 0;
path_switch = 0;
gait_seq_ind = 0;

while(true)
    
    if P.QueueLength > 0
        
        cam_data = poll(P);
        newim = cam_data.img;
        ts = cam_data.time_stamp;
        poll_data = poll_data + 1;
        idx = idx + 1;
        [output_data] = tracker_obj.tracking(newim,PrevPt,P0,robo_centroid,trajectory_position,idx);
        tracking_data(idx,:) = [output_data.tracking_data(idx,:) ts];
        PrevPt = reshape(output_data.tracking_data(idx,1:12),[3,4])';
        robo_centroid = output_data.robo_centroid;
        
        if idx == 1
            P0 = output_data.P0;
        end
        
    end
    
    %Start the robot
    if jj == 1 || contains(data_recieve,check_complete) == 1
        
        if jj <= size(gait_perform_seq,2)
            
            if jj~=1
                
                while L.QueueLength > 0
                    rem_data_queue = poll(L);
                end
                
                rem_data_queue = rem_data_queue - poll_data
                
                if rem_data_queue > 0
                    
                    while(rem_data_queue)
                        
                        fprintf('Remaining data left for gait switching = %d\n',rem_data_queue);
                        cam_data = poll(P);
                        newim = cam_data.img;
                        ts = cam_data.time_stamp;
                        poll_data = poll_data + 1;
                        idx = idx + 1;
                        [output_data] = tracker_obj.tracking(newim,PrevPt,P0,robo_centroid,trajectory_position,idx);
                        tracking_data(idx,:) = [output_data.tracking_data(idx,:) ts];
                        PrevPt = reshape(output_data.tracking_data(idx,1:12),[3,4])';
                        robo_centroid = output_data.robo_centroid;
                        
                        rem_data_queue = rem_data_queue - 1;
                        
                    end
                    
                end
                clear rem_data_queue;
                
                % Polling gait switch frames
                overall_switch = overall_switch + 1;
                gait_switch_frames(overall_switch) = poll_data;
                
            end
            msoro_robo.start_gait(gait_perform_seq{jj}, num_gait_cycles(jj));
            index = find(gait_names == string(gait_perform_seq{jj}));
            complete_gait_time = (size(cell2mat(gait_trans_seq(index)),2)*0.450*num_gait_cycles(jj))+0.5;
            gait_seq_ind = gait_seq_ind + 1;
            performed_seq(gait_seq_ind) = string(gait_perform_seq{jj});
            performed_num_cycle(gait_seq_ind) = num_gait_cycles(jj);
            tic
            jj = jj+1;
            data_recieve = "Void";
        end
        
    end
    
    % Initialize to read the serial port for gait cycle finish command
    if jj > size(gait_perform_seq,2) && contains(data_recieve,check_complete) == 1
        
        while L.QueueLength > 0
            rem_data_queue = poll(L);
        end
        
        %         rem_data_queue = abs(rem_data_queue - poll_data)
        
        rem_data_queue = rem_data_queue - poll_data
        
        if rem_data_queue > 0
            
            while(rem_data_queue)
                fprintf('Remaining data left for recalculation = %d\n',rem_data_queue);
                cam_data = poll(P);
                newim = cam_data.img;
                ts = cam_data.time_stamp;
                poll_data = poll_data + 1;
                idx = idx + 1;
                [output_data] = tracker_obj.tracking(newim,PrevPt,P0,robo_centroid,trajectory_position,idx);
                tracking_data(idx,:) = [output_data.tracking_data(idx,:) ts];
                PrevPt = reshape(output_data.tracking_data(idx,1:12),[3,4])';
                robo_centroid = output_data.robo_centroid;
                
                rem_data_queue = rem_data_queue - 1;
            end
            
        end
        clear rem_data_queue;
        
        % Goal/Final target reached check
        curr_robo_position = [mean(tracking_data(idx,1:3:12)) mean(tracking_data(idx,2:3:12))];  %Pixel
        X = [curr_robo_position/pixel_to_cm;target_position/pixel_to_cm];
        d = pdist(X,'euclidean');
        
        if d <= stop_radius
            cancel(f);
            clear f;
            delete(gcp('nocreate'));
            fprintf('Goal reached - Experiment done\n');
            break;
        end
        
        if d > stop_radius
            fprintf('Calculating new path\n');
            
            clear trajectory_plan;
            exist('trajectory_plan','var');
            %             trajectory_plan = rtGaitPlanner.planTrajectory(SE2(curr_robo_position/8.524,atan2(tracking_data(idx,26),tracking_data(idx,25))), goal_position);
            trajectory_plan = rtGaitPlanner.planTrajectory(SE2(curr_robo_position/8.524,wrapTo2Pi(output_data.robo_rotated(1))), goal_position);
            
            %             clear trajectory_gait_data;
            %             trajectory_gait_data = gait_sequence(trajectory_plan,gait_seq_window);
            trajectory_gait_data = gait_sequence_tunnel(trajectory_plan,gait_seq_window);
            
            gait_perform_seq = trajectory_gait_data.gait_perform_seq;
            num_gait_cycles = trajectory_gait_data.num_gait_cycles;
            
            jj = 1;
            
            % Polling gait switch frames
            path_switch = path_switch + 1;
            path_recalc_frames(path_switch) = poll_data;
            
            % Polling trajectory datas
            trajectory_data{path_switch+1} = trajectory_plan;
            
            initial_position = (curr_robo_position/pixel_to_cm)';
            initial_orientation = wrapTo2Pi(output_data.robo_rotated(1));
            for i = 1:size(trajectory_plan.gait_names,2)
                gait_perform_seq_FK = string(trajectory_plan.gait_names(i));
                sequence_length = trajectory_plan.gait_durations(1,i);
                if gait_perform_seq_FK == "B"
                    dp = gait_library(1,1).Delta_Pose(1:2,1);
                    dp_theta = gait_library(1,1).Delta_Pose(3,1);
                end
                if gait_perform_seq_FK == "G"
                    dp = gait_library(1,2).Delta_Pose(1:2,1);
                    dp_theta = gait_library(1,2).Delta_Pose(3,1);
                end
                if gait_perform_seq_FK == "I"
                    dp = gait_library(1,3).Delta_Pose(1:2,1);
                    dp_theta = gait_library(1,3).Delta_Pose(3,1);
                end
                if gait_perform_seq_FK == "J"
                    dp = gait_library(1,4).Delta_Pose(1:2,1);
                    dp_theta = gait_library(1,4).Delta_Pose(3,1);
                end
                if gait_perform_seq_FK == "K"
                    dp = gait_library(1,5).Delta_Pose(1:2,1);
                    dp_theta = gait_library(1,5).Delta_Pose(3,1);
                end
                clear gait_perform_dir;
                if i == 1
                    current_position = initial_position;
                    current_orientation = initial_orientation;
                    %         fk_pose(1,:) = [initial_position',initial_orientation];
                    fk_pose(1,:) = initial_position';
                    fk_theta(1,:) = current_orientation;
                end
                [new_position,current_orientation,orientation_theta] = FK_simulation_step(dp,dp_theta,sequence_length,current_position,current_orientation(end,end));
                current_position = new_position(:,end);
                %     fk_pose(i+1,:) = [new_position',current_orientation];
                fk_pose = [fk_pose' new_position];
                fk_pose = fk_pose';
                fk_theta = [fk_theta;orientation_theta];
            end
            
            % Obatining trajectory pose for plotting
            %             clear trajectory_position;
            trajectory_position = fk_pose';
            fk_pose_data{fk_pose_count+1} = trajectory_position;
            clear fk_pose;
            clear fk_theta;
        end
        
        
        
        while P.QueueLength > 1
            
            flush_queue = poll(P);
            poll_data = poll_data + 1;
            
        end
        fprintf('Flushed queue after path recalculation\n');
        
    end
    
    curr_time = toc;
    
    if curr_time > complete_gait_time && contains(data_recieve,check_complete) ~= 1
        data_recieve = strtrim(readline(msoro_robo.ser_device));
        fprintf('%s\n',data_recieve);
        %         %Rest for robot
        %         pause(2);
    end
    
end
