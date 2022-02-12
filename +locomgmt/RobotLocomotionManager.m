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
  properties  (Access = public)
    % Scenario properties
    goal_pos;           % robot goal position (currently unused)
    
    % Path/trajectory planning
    traj_plan;
    
    % Robot (locomotive) dynamics
    gait_dynamics;

    % Robot configuration
    robo_pose;            % robot pose
        
    % Timer/thread periodicities & timing
    cntrl_update_period;
    robo_pose_update_period;
    
    % Timestamps
    last_robo_pose_timestamp;
    last_cntrl_timestamp;
    poll_start_timestamp;
    
    % Figure & plot handles
    logging_visual;
    logging_params;
        
    % Data recording
    record;    
  end
  
  methods
    % Constructor
    function this = RobotLocomotionManager( params )
      if ( nargin < 1 )
        params = [];
      end
      
      % == Set input parameters
      this.set_property(params, 'robo_pose_update_period', 0.25);  % sec.
      this.set_property(params, 'cntrl_update_period', 0.5);  % sec.

      % == Internal state      
      %   Path/Trajectory Planning
      this.traj_plan = [];

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
    % Main class loop facilitating indefinite closed-loop control of robot
    start( this )

    set_cntrl_params( this )
    
    set_goal_position( this )
    
    set_logging_level( this )

    update_robo_pose( this )

    goal_reached( this )

    initialize_robot( this )

    shutdown_robot( this )
    
    init_logging( this )

    compute_cntrl( this )

    send_cntrl( this )

    generate_trajectory_plan( this )
        
    set_gait_dynamics( this )
    
    save_scenario_state( this )

  end     % methods (Abstract)
end     % class








