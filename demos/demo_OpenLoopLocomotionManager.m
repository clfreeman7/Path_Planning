%
% Script: demo_RobotLocomotionManager.m
%
% Dependencies: 
%   locomgmt.RobotLocomotionManager
%   msoro.api.MSoRo_IO
%   gaitdef.Gait
%   gaitdef.GaitDir
%   gaitdef.GaitType
%
% Description: 
%   Demonstrate basic high-level flow of logic when managing robot 
%   trajectory execution.
%
%   


% [0] == Script setup
% Add dependencies to classpath
addpath('../');

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');


% [1] == Script usage parameter(s):
GAIT_LIBRARY_MAT = 'data/gait_library_2.mat';
TRAJECTORY_PLAN_MAT = 'data/msoro_openloop_trajectory.mat';

OUTPUT_MAT_FILE = 'data/msoro_exp_scenario01.mat';

%  MSoRo serial port configuration
port = '/dev/ttyUSB0';
baud = 9600;
timeout = 30;


% [2] == Instantiate Matlab-ROS framework for trajectory planning and tracking
locom_mgr_params.robo_pose_update_period = 0.5;   % sec. (periodicity at which to retrieve updated robot pose)
locom_mgr = locomgmt.OpenLoopLocomotionManager( locom_mgr_params );

% Configure logging (optional)
logging_params.log_level = 0;             % not currently used
logging_params.data_collect_level = 1;    % collect ALL data
locom_mgr.set_logging( logging_params );

% Set gait motion model(s)
gait_motion_models = load(GAIT_LIBRARY_MAT).gait_library_2([2, 5]);
locom_mgr.set_gait_models(gait_motion_models); 

% Initialize
msoro_io_params.port = port;
msoro_io_params.baud = baud;
msoro_io_params.timeout = timeout;
locom_mgr.initialize(msoro_io_params); 


% [3] == Begin robot locomotion control/management
input('Press <Enter> to begin robot loocmotion management.');
trajectory_plan = load(TRAJECTORY_PLAN_MAT).robo_traj_plan;
locom_mgr.start( trajectory_plan );   % begin managing/executing planned trajectory


% Save results to output .mat file
locom_mgr.save_scenario_results( OUTPUT_MAT_FILE );




