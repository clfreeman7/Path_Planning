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

%  MSoRo serial port config.
port = '/dev/ttyUSB0';
baud = 9600;
timeout = 30;


% [2] == Instantiate Matlab-ROS framework for trajectory planning and tracking
locom_mgr_params.robo_pose_update_period = 1;   % sec. (periodicity at which to retrieve updated robot pose)
% locom_mgr_params.robo_cntrl_update_period = 5;       % sec. (periodicity at which to compute update control)
locom_mgr = locomgmt.RobotLocomotionManager_Impl( locom_mgr_params );

% Set gait motion model(s)
gait_motion_models = load(GAIT_LIBRARY_MAT).gait_library_2;
locom_mgr.set_gait_models(gait_motion_models); 

% Set open-loop trajectory plan
trajectory_plan = load(TRAJECTORY_PLAN_MAT).robo_traj_plan;
locom_mgr.set_trajectory_plan(trajectory_plan);

% Initialize
msoro_io_params.port = port;
msoro_io_params.baud = baud;
msoro_io_params.timeout = timeout;
locom_mgr.initialize(msoro_io_params); 


% [3] == Begin robot locomotion control/management
input('Press <Enter> to begin robot loocmotion management.');
locom_mgr.start();






