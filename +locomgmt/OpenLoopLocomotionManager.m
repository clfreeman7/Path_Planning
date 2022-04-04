%============================= OpenLoopLocomotionManager =================
%
%  Implementation of abstract RobotLocomotionManager class, for open-loop 
%   trajectory execution/management.
%
%  OpenLoopLocomotionManager()
%
%  INPUTS:
%    TBD
%
%
%  OUTPUTS:
%
%
%  ============================= OpenLoopLocomotionManager =================
classdef OpenLoopLocomotionManager < locomgmt.RobotLocomotionManager
  properties  (Access = public)
    % Subclass-specific properties

  end
  
  methods
    % Constructor
    function this = OpenLoopLocomotionManager( params )
      if ( nargin < 1 )
        params = [];
      end
      
      this@locomgmt.RobotLocomotionManager( params );    % super class constructor

      % == Set input parameters
      % this.set_property(params, 'subclass_specific_param', 0.25);

    end
    
    % Main loop managing indefinite locomotion control of robot
    % NOTE(s): Overload this method, in a sub-class, if need different implementation
    function start( this )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::start()] Entering main polling loop!\n', ...
                this.get_cur_time());
      
      
      % [0] == Any initialization procedures needed priorto polling start      
      this.initialize();                % pre-scenario robot initialization procedures
      this.update_robo_pose();          % get initial robot pose
      this.generate_trajectory_plan();  % plan trajectory & control


      % [1] == Main polling loop
      this.poll_start_timestamp = this.get_cur_time();

      while( ~this.goal_reached() )
        cur_time = this.get_cur_time();
        
        % Retrieve and update robot pose
        if ( cur_time-this.last_robo_pose_timestamp >= this.robo_pose_update_period )
          this.update_robo_pose();
            
          this.last_robo_pose_timestamp = cur_time;
        end
        
        % Periodic feedback tracking control update
        if ( cur_time-this.last_cntrl_timestamp >= this.robo_cntrl_update_period )
          % Compute feedback (tracking) control commands
          robo_cntrl = this.update_robo_cntrl();
          
          % Command initial/updated gait parameters to track trajectory
          this.send_cntrl( robo_cntrl );

          this.last_cntrl_timestamp = this.get_cur_time();
        end        
        
        pause(0.01);    % polling delay
      end
      

      % [2] == Clean-up (scenario complete)
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::poll()] Locomotion control completed.\n', this.get_cur_time());
      this.shutdown();  % robot clean-up procedures
      
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::poll()] Exiting.\n', this.get_cur_time());
      
      this.save_scenario_results();
    end


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Periodic execution
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function update_robo_pose( this )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::update_robo_pose()] Retrieve updated robot pose.\n', ...
                this.get_cur_time());

      % TODO: not needed for open-loop??
    end

    function [ robo_cntrl ] = update_robo_cntrl( this )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::update_robo_cntrl()] Compute updated control.\n', ...
                this.get_cur_time());

      robo_cntrl = [];
    end

    function send_cntrl( this, a_robo_cntrl )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::send_cntrl()] Command updated control.\n', ...
                this.get_cur_time());
    end


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % One-time execution 
    %       (e.g. start-up/initialize/shutdown)
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function initialize( this )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::initialize()] Command initial robot configuration.\n', ...
                this.get_cur_time());

      % TODO: 
      %   Initialize logging
      %   MSoRo API init
    end

    function init_logging( this, a_logging_params )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::init_logging()] Setup visualization aids (e.g. plots) and data recording.\n', ...
                this.get_cur_time());

      % TODO:
      %   Set-up data recording variables/structure
      %   Set-up visualization (e.g. plots and handles)
    end

    function shutdown( this )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::shutdown()] Perform any final robot commands prior to terminating scenario.\n', ...
                this.get_cur_time());
    end

    function save_scenario_results( this )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::save_scenario_results()] Save scenario state to .mat file.\n', ...
                this.get_cur_time());
    end
    
    % Path/trajectory planning
    function generate_trajectory_plan( this, a_start_pose, a_goal_pos )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::generate_trajectory_plan()] Generate trajectory plan from start pose (%.2f, %.2f, %.2f) to goal position (%.2f, %.2f).\n', ...
                this.get_cur_time(), a_start_pose(1), a_start_pose(2), a_start_pose(3), a_goal_pos(1), a_goal_pos(2));

      % TODO: load scenario from file (class input) and generate trajectory
      %         between (user input) start pose and goal position
    end


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Setters
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function set_cntrl_params( this, a_cntrl_params )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::set_clcntrl_params()] Set control parameters.\n', ...
                this.get_cur_time());
    end
    
    function set_goal( this, a_goal_pos )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::set_goal()] Set goal position.\n', ...
                this.get_cur_time());

      this.goal_pos = a_goal_pos;
    end
    
    function set_logging_level( this, a_logging_level )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::set_logging_level()] Set logging level.\n', ...
                this.get_cur_time());

      this.logging_params.logging_level = a_logging_level;
    end
     
    % Robot locomotive dynamic/kinematic models
    function set_gait_dynamics( this, a_gait_motion_models )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::set_gait_dynamics()] Set gait motion models.\n', ...
                this.get_cur_time());
    end

  end     % methods 
end     % class








