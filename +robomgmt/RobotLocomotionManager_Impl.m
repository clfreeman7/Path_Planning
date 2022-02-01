%============================= RobotLocomotionManager_Impl =================
%
%  Implementation of abstract RobotLocomotionManager class, for 
%  demonstration purposes.
%
%  RobotLocomotionManager_Impl()
%
%  INPUTS:
%    TBD
%
%
%  OUTPUTS:
%
%
%  ============================= RobotLocomotionManager_Impl =================
classdef RobotLocomotionManager_Impl < robomgmt.RobotLocomotionManager
  properties (Constant)
    
  end
  
  properties  (Access = public)
    % Subclass-specific properties

  end
  
  methods
    % Constructor
    function this = RobotLocomotionManager_Impl( params )
      if ( nargin < 1 )
        params = [];
      end
      
      this@robomgmt.RobotLocomotionManager( params );    % super class constructor

      % == Set input parameters
      % this.set_property(params, 'subclass_specific_param', 0.25);

    end
    
    % Main loop managing indefinite locomotion control of robot
    % NOTE(s): Overload this method, in a sub-class, if need different implementation
    function start( this )
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::start()] Entering main polling loop!\n', ...
                this.get_cur_time());
      
      
      % [0] == Any initialization procedures needed priorto polling start
      this.poll_start_timestamp = this.get_cur_time();
      
      this.initialize_robot();          % pre-scenario robot initialization procedures
      this.update_robo_pose();          % get initial robot pose
      this.generate_trajectory_plan();  % plan trajectory & control


      % [1] == Main polling loop
      while( ~this.goal_reached() )
        cur_time = this.get_cur_time();
        
        % Retrieve and update robot pose
        if ( cur_time-this.last_robo_pose_timestamp >= this.robo_pose_update_period )
          this.update_robo_pose();
            
          this.last_robo_pose_timestamp = cur_time;
        end
        
        % Periodic feedback tracking control update
        if ( cur_time-this.last_cntrl_timestamp >= this.cntrl_update_period )
          % Compute feedback (tracking) control commands
          robo_cntrl = this.compute_cntrl();
          
          % Command initial/updated gait parameters to track trajectory
          this.send_cntrl( robo_cntrl );

          this.last_cntrl_timestamp = this.get_cur_time();
        end        
        
        pause(0.01);    % polling delay
      end
      

      % [2] == Clean-up (scenario complete)
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::poll()] Locomotion control completed.\n', this.get_cur_time());
      this.shutdown_robot();  % robot clean-up procedures
      
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::poll()] Exiting.\n', this.get_cur_time());
      
      this.save_scenario_state();
    end

    function set_cntrl_params( this, a_cntrl_params )
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::set_clcntrl_params()] Set control parameters.\n', ...
                this.get_cur_time());
    end
    
    function set_goal_position( this, a_goal_pos )
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::set_goal_position()] Set goal position.\n', ...
                this.get_cur_time());
    end
    
    function set_logging_level( this, a_logging_level )
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::set_logging()] Set logging level.\n', ...
                this.get_cur_time());
    end

    function init_logging( this, a_logging_params )
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::init_logging()] Setup visualization aids (e.g. plots) and data recording.\n', ...
                this.get_cur_time());
    end

    function update_robo_pose( this )
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::update_robo_pose()] Retrieve updated robot pose.\n', ...
                this.get_cur_time());
    end

    function [ result ] = goal_reached( this )
%       fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::goal_reached()] Check whether goal reached.\n', ...
%                 this.get_cur_time());

      result = false;
    end

    function initialize_robot( this )
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::initialize_robot()] Command initial robot configuration.\n', ...
                this.get_cur_time());
    end

    function shutdown_robot( this )
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::shutdown_robot()] Perform any final robot commands prior to terminating scenario.\n', ...
                this.get_cur_time());
    end
    
    function [ robo_cntrl ] = compute_cntrl( this )
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::compute_cntrl()] Compute updated control.\n', ...
                this.get_cur_time());

      robo_cntrl = [];
    end

    function send_cntrl( this, a_robo_cntrl )
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::send_cntrl()] Command updated control.\n', ...
                this.get_cur_time());
    end

    % Path/trajectory planning
    function generate_trajectory_plan( this )
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::generate_trajectory_plan()] Generate trajectory plan.\n', ...
                this.get_cur_time());
    end
        
    % Robot locomotive dynamic/kinematic models
    function set_gait_dynamics( this, a_gait_motion_models )
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::set_gait_dynamics()] Set gait motion models.\n', ...
                this.get_cur_time());
    end
    
    function save_scenario_state( this )
      fprintf('[%.2f sec.] [RobotLocomotionManager_Impl::save_scenario_state()] Save scenario state to .mat file.\n', ...
                this.get_cur_time());
    end
    
  end     % methods 
end     % class








