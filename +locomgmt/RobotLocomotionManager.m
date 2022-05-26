%============================= RobotLocomotionManager =================
%
%  Utilities and framework supporting discrete-time trajectory planning and 
%  closed-loop tracking for (gait-constrained) mobile robots.
%
%  RobotLocomotionManager()
%
%  INPUTS:
%    TBD
%
%
%  OUTPUTS:
%
%
%  ============================= RobotLocomotionManager =================
classdef RobotLocomotionManager < handle  
  properties  (Access = public)   % temporarily leaving all properties public
    % Scenario properties
    goal_pos;           % robot goal position (currently unused)
    
    % Path/trajectory planning
    trajectory_plan;
    
    % Robot (locomotive) dynamics
    gait_models;

    % Robot configuration
    robo_pose;            % robot pose
        
    % Timer/thread periodicities & timing
    robo_cntrl_update_period;
    robo_pose_update_period;
    
    % Timestamps
    last_robo_pose_timestamp;
    last_cntrl_timestamp;
    poll_start_timestamp;
    
    % Figure & plot handles
    logging_visual;
    logging_params;
        
    % Data recording
    data_record;
  end
  
  methods
    % Constructor
    function this = RobotLocomotionManager( params )
      if ( nargin < 1 )
        params = [];
      end
      
      % == Set input parameters
      this.set_property(params, 'robo_pose_update_period', 0.25);  % sec.
      this.set_property(params, 'robo_cntrl_update_period', 0.5);  % sec.

      % == Internal state      
      %   Path/Trajectory Planning
      this.trajectory_plan = [];

      %   Initialize timestamps
      this.poll_start_timestamp = -1;
      this.last_cntrl_timestamp = -1;
      this.last_robo_pose_timestamp = -1;
      
      %   Robot pose
      this.robo_pose = [];
    end

    % Retrieve & compute current timestamp
    function [ cur_time_sec ] = get_cur_time( this )
      cur_time = datevec(now);
      cur_time_sec = cur_time(4)*3600 + cur_time(5)*60 + cur_time(6);
    end
    
    % Set parameter value for class-instance, based on user specified 
    % values (or default if property doesn't exist as struct field)
    function set_property(this, source_struct, param_name, def_val)
      if ( isfield(source_struct, param_name) )
        this.(param_name) = source_struct.(param_name);
      else
        this.(param_name) = def_val;
      end
    end
    
  end     % methods

  methods (Abstract)

    % Main method (manage indefinite robot control & execution)
    start( this )

    % Start-up/initialize/shutdown (methods designed for one-time execution)
    initialize( this )                % initialization procedures (for class instance and/or robot)
    shutdown( this )                  % shutdown procedures (for class instance and/or robot)

    % Logging/data recording
    set_logging( this )               % setup associated with data recording or visualization
    save_scenario_results( this )     % export/save run data (e.g. .mat archive)

    % Periodic methods
    update_robo_pose( this )  % update current robot pose
    compute_robo_cntrl( this ) % compute updated control command(s)
    send_cntrl( this )        % send control command(s) to robot
    goal_reached( this )      % evaluate whether goal reached

    % Planning & control
    set_goal( this )                % set locomotion goal (pose)
    set_trajectory_plan( this )     % set robot trajectory plan
    set_gait_models( this )         % set gait motion models
    set_cntrl_params( this )        % set control parameters

  end     % methods (Abstract)
end     % class








