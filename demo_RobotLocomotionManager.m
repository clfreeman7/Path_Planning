% Demonstrate basic high-level flow of logic when managing robot trajectory
% execution.
%
% Pre-requisite(s): 
%   TBD
%   

% [0] == Script setup
% Add dependencies to classpath
addpath('./');

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');


% [1] == Script usage parameter(s):
%     TBD


% [2] == Instantiate Matlab-ROS framework for trajectory planning and tracking
% Control of Wedged sidewinding gait
locom_mgr_params.robo_pose_update_period = 1;   % sec. (periodicity at which to retrieve updated robot pose)
locom_mgr_params.cntrl_update_period = 5;       % sec. (periodicity at which to compute update control)
locom_mgr = robomgmt.RobotLocomotionManager_Impl( locom_mgr_params );

% Set logging parameters & logging level
logging_params.logging_level = 3;
logging_params.traj_fig_hdl = 17001;
logging_params.cntrl_fig_hdl = 17002;
logging_params.record_name = 'msoro_exp01';
locom_mgr.init_logging( logging_params );
locom_mgr.set_logging_level( 1 );

% Set gait motion model(s)
gait_motion_models = [];
locom_mgr.set_gait_dynamics(gait_motion_models); 

% Set tracking/control parameters
cntrl_params = [];
locom_mgr.set_cntrl_params(cntrl_params);

% Set robot goal position
goal_pos = [ 100 ; 200 ];
locom_mgr.set_goal_position(goal_pos);


% [3] == Begin robot locomotion control/management
input('Press <Enter> to begin robot loocmotion management.');
locom_mgr.start();






