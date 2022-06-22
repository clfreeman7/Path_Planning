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
%  ============================= OpenLoopLocomotionManager =================
classdef OpenLoopLocomotionManager < locomgmt.RobotLocomotionManager
  properties  (Access = public)
 
    msoro_intfc;

  end
  
  methods
    % Constructor
    function this = OpenLoopLocomotionManager( params )
      if ( nargin < 1 )
        params = [];
      end
      
      this@locomgmt.RobotLocomotionManager( params );    % super class constructor

      % == Set input parameters
      % this.set_property(params, 'example_subclass_parameter_1', 0.25);

      % == Other class properties
      this.robo_cntrl_update_period = 0;   % init. to 0 -> immediate gait execution on start()

      %   Init. logging configuration (default no logging/data collection)
      this.logging_params.log_level = 0;
      this.logging_params.data_collect_level = 0;

      %   Init. data record fields
      this.data_record.poll_start_timestamp = [];     % (timestamp when start() polling begins)
      this.data_record.robo_pose_timestamp = [];      % (sequence of timestamps when robot pose is updated)
      this.data_record.robo_pose = [];                % (sequence of robot 3-DoF poses)
      this.data_record.robo_control_timestamp = [];   % (sequence of timestamps when updated control is commanded)
      this.data_record.robo_control = [];             % (sequence of robot commands; custom-defined structs)
      this.data_record.gait_models = [];              % (sequence of gaitdef.Gait instances)
      this.data_record.trajectory_plan = [];          % (sequence of locomgmt.locotraj.LocomotionTrajectory instances)

      this.data_record.robo_pose_update_period = this.robo_pose_update_period;  % (pose update rate)      
    end
    

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Main
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Main polling loop, managing indefinite locomotion control of robot
    % 
    % Input(s):
    %   a_robo_traj  (class instance of locomgmt.locotraj.LocomotionTrajectory)
    %       a_robo_traj.timestamps         (array of timestamps at which to command gaits)
    %       a_robo_traj.poses              (array of 3-DoF destination poses)
    %       a_robo_traj.gait_names         (array of gait names)
    %       a_robo_traj.gait_types         (array of gait types: "ROTATE" or "TRANSLATE")
    %       a_robo_traj.gait_durations     (array of gait durations in # gait periods)
    %       a_robo_traj.gait_directions    (array of gait directions: "CCW", "SE", "NE", "NW", "SW")
    % 
    function start( this, a_robo_traj )
      assert( isa(a_robo_traj, 'locomgmt.locotraj.LocomotionTrajectory'), ...
              '[OpenLoopLocomotionManager::start()] Input trajectory plan  must be of type: locomgmt.locotraj.LocomotionTrajectory\n' );

      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::start()] Beginning trajectory execution ...\n', ...
                this.get_cur_time());


      % [0] == Any initialization procedures needed prior to polling
      this.set_trajectory_plan(a_robo_traj);  % set trajectory plan to follow
      this.update_robo_pose();                % get initial robot pose


      % [1] == Main polling loop
      this.poll_start_timestamp = this.get_cur_time();
      if ( this.logging_params.data_collect_level > 0 )
        this.data_record.poll_start_timestamp = this.poll_start_timestamp;
      end

      complete = false;
      trajectory_index = 1;   % trajectory plan index to act on
      while( ~complete )
        cur_time = this.get_cur_time();
        
        % Retrieve and update robot pose
        if ( cur_time-this.last_robo_pose_timestamp >= this.robo_pose_update_period )
          this.update_robo_pose();

          if ( this.logging_params.data_collect_level > 0 )
            this.data_record.robo_pose = [this.data_record.robo_pose, this.robo_pose];
            this.data_record.robo_pose_timestamp = [this.data_record.robo_pose_timestamp, cur_time];
          end

          this.last_robo_pose_timestamp = cur_time;
        end
        
        % Periodic feedback tracking control update
        if ( cur_time-this.last_cntrl_timestamp >= this.robo_cntrl_update_period )
          % Compute feedback (tracking) control commands
          robo_cntrl = this.compute_robo_cntrl( trajectory_index );
          
          if ( isempty(robo_cntrl) )  % exit when no more gaits to run
            complete = true;
            continue;
          end

          if ( this.logging_params.data_collect_level > 0 )
            this.data_record.robo_control = [this.data_record.robo_control, robo_cntrl];
            this.data_record.robo_control_timestamp = [this.data_record.robo_control_timestamp, cur_time];
          end

          % Command initial/updated gait parameters to track trajectory
          % [ARL TODO]: for translational gaits, this is running Gait E
          % regardless of direction (NE, NW, SW, SE) -> code needs to be
          % updated to distinguish different gait for each direction and
          % run appropriate one
          this.send_cntrl( robo_cntrl );

          this.robo_cntrl_update_period = robo_cntrl.gait_duration*robo_cntrl.gait_period - 0.5;    % time to next scheduled gait command (minus  0.5 sec. to maintain conitnuous gait execution); TODO -> parametrize 0.5 sec.
          trajectory_index = trajectory_index + 1;
          this.last_cntrl_timestamp = this.get_cur_time();
        end        
        
        pause(0.01);    % polling delay
      end
      

      % [2] == Clean-up (scenario complete)
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::poll()] Locomotion control completed.\n', this.get_cur_time());
      this.shutdown();  % robot clean-up procedures
      
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::poll()] Exiting.\n', this.get_cur_time());      
    end


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Start-up/Shutdown
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Initialization procedure(s)
    % 
    % Input(s):
    %   a_msoro_io_params  (struct)
    %     a_msoro_io_params.port         (port ID, e.g. "COM6")
    %     a_msoro_io_params.baud         (baud rate, e.g. 9600)
    %     a_msoro_io_params.timeout      (timeout duration, sec.)
    % 
    function initialize( this, a_msoro_io_params )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::initialize()] Command initial robot configuration.\n', ...
                this.get_cur_time());

      %  MSoRo serial port config.
      port = a_msoro_io_params.port;
      baud = a_msoro_io_params.baud;
      timeout = a_msoro_io_params.timeout;

      % Instantiate MSoRo interface and connect to port
      this.msoro_intfc = msoro.api.MSoRo_IO();

      this.msoro_intfc.connect(port, baud, timeout);
      fprintf('[OpenLoopLocomotionManager::initialize()] MSoRo serial port opened.\n\n');
      pause(2);

      % Define gaits to be used
      gait_trans_seq = cell([1, length(this.gait_models)]);
      gait_names = cell([1, length(this.gait_models)]);
      for ii = 1:length(this.gait_models)
        gait_trans_seq{ii} = this.gait_models(ii).robo_states;
        gait_names{ii} = this.gait_models(ii).gait_name;
      end
      fprintf('[OpenLoopLocomotionManager::initialize()] Defining MSoRo gaits ...\n\n');
      this.msoro_intfc.define_gait(gait_trans_seq, gait_names);
      pause(2);

    end

    % Shutdown procedure(s)
    % 
    function shutdown( this )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::shutdown()] Perform any final robot commands prior to terminating scenario.\n', ...
                this.get_cur_time());

      this.msoro_intfc.disconnect();
      fprintf('[OpenLoopLocomotionManager::initialize()] MSoRo serial port closed.\n\n');
    end


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Periodic execution
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Retrieve current robot pose from visual tracking
    % 
    % Input(s):
    %   a_traj_index (index of this.trajectory_plan fields to retrieve)
    % 
    % Ouptut(s):
    %   robo_cntrl (struct)
    %     robo_cntrl.timestamp        (target timestamp)
    %     robo_cntrl.pose             (target pose)
    %     robo_cntrl.gait_name        (gait name to execute)
    %     robo_cntrl.gait_type        (gait type: "TRANSLATE" or "ROTATE")
    %     robo_cntrl.gait_duration    (gait duration, # of gait periods)
    %     robo_cntrl.gait_direction   (gait direction: "NE", "NW", "SE", "SW", "CCW")
    % 
    function update_robo_pose( this )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::update_robo_pose()] Retrieve updated robot pose.\n', ...
                this.get_cur_time());

      % [ARL TODO]: Replace code here to retrieve current robot pose (3-DoF) from
      % visual tracking
      this.robo_pose = [ 0 ; 0 ; 0 ];

    end

    % Compute robot control
    % 
    % Input(s):
    %   a_traj_index (index of this.trajectory_plan fields to retrieve)
    % 
    % Ouptut(s):
    %   robo_cntrl (struct)
    %     robo_cntrl.timestamp        (target timestamp)
    %     robo_cntrl.pose             (target pose)
    %     robo_cntrl.gait_name        (gait name to execute)
    %     robo_cntrl.gait_type        (gait type: "TRANSLATE" or "ROTATE")
    %     robo_cntrl.gait_duration    (gait duration, # of gait periods)
    %     robo_cntrl.gait_direction   (gait direction: "NE", "NW", "SE", "SW", "CCW")
    %     robo_cntrl.gait_period      (gait period, sec.)
    % 
    function [ robo_cntrl ] = compute_robo_cntrl( this, a_traj_index )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::compute_robo_cntrl()] Compute updated control.\n', ...
                this.get_cur_time());

      if (a_traj_index > length(this.trajectory_plan.timestamps))    % if index out of bounds, return empty control struct (implies trajectory completion)
        robo_cntrl = [];
      else
        % Extract control-relevant information
        robo_cntrl.timestamp = this.trajectory_plan.timestamps(a_traj_index);
        robo_cntrl.pose = this.trajectory_plan.poses(a_traj_index);
        robo_cntrl.gait_name = this.trajectory_plan.gait_names(a_traj_index);
        robo_cntrl.gait_type = this.trajectory_plan.gait_types(a_traj_index);
        robo_cntrl.gait_duration = this.trajectory_plan.gait_durations(a_traj_index);
        robo_cntrl.gait_direction = this.trajectory_plan.gait_directions(a_traj_index);
  
        % Compute gait period
        robo_cntrl.gait_period = -1;
        for ii = 1:length(this.gait_models)
          if ( this.gait_models(ii).gait_name == robo_cntrl.gait_name )
            robo_cntrl.gait_period = this.gait_models(ii).len_gait*this.gait_models(ii).transition_time;
            break;
          end
        end
        if ( robo_cntrl.gait_period <= 0 )
          %   TODO: other shutdown calls here, before error()
          error('[OpenLoopLocomotionManager::compute_robo_cntrl()] Invalid gait period computed!');
        end
      end
    end

    % Command MSoRo gait
    % 
    % Input(s):
    %   a_robo_cntrl 
    %     robo_cntrl.gait_name        (gait name to execute)
    %     robo_cntrl.gait_duration    (gait duration, # of gait periods)
    % 
    function send_cntrl( this, a_robo_cntrl )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::send_cntrl()] Commanding gait %s for %d cycles.\n', ...
                this.get_cur_time(), a_robo_cntrl.gait_name, a_robo_cntrl.gait_duration);

      % Send MSoRo 'start' command
      this.msoro_intfc.start_gait(a_robo_cntrl.gait_name, floor(a_robo_cntrl.gait_duration));
    end


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Logging & data recording
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Configure logging and data collection
    % 
    % Input(s):
    %   a_params (struct)
    %     a_params.log_level            (0 = no logging, 1 = logging)
    %     a_params.data_collect_level   (0 = no collection, 1 = collection)
    % 
    function set_logging( this, a_params )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::set_logging()] Setup visualization aids (e.g. plots) and data recording.\n', ...
                this.get_cur_time());

      if ( isfield(a_params, 'log_level') )
        this.logging_params.log_level = a_params.log_level;
      end

      if ( isfield(a_params, 'data_collect_level') )
        this.logging_params.data_collect_level = a_params.data_collect_level;
      end

      % TODO:
      %   Set-up data recording variables/structure
      %   Set-up visualization (e.g. plots and handles)
    end

    % Save data collected during trajectory execution
    % 
    % Input(s):
    %   a_output_mat_file (full path to output .mat file; string)
    % 
    function save_scenario_results( this, a_output_mat_file )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::save_scenario_results()] Save scenario state to .mat file.\n', ...
                this.get_cur_time());

      data_record = this.data_record;
      save(a_output_mat_file, 'data_record');
    end
    

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Planning & control
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Set robot goal position
    % 
    % Input(s):
    %   a_goal_pos (x-y coordinate)
    %   a_goal_thresh (threshold distance within which declare success)
    % 
    function set_goal( this, a_goal_pos, a_goal_thresh )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::set_goal()] Set goal position.\n', ...
                this.get_cur_time());

      this.goal_pos = a_goal_pos;
      this.goal_thresh = a_goal_thresh;
    end

    % Check if current robot position (x, y) within threshold distance of
    % goal position.
    % 
    % Output(s):
    %   result  (boolean value)
    % 
    function result = goal_reached( this )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::goal_reached()] Check goal reached.\n', ...
                this.get_cur_time());

      result = sqrt((this.goal_pos(1)-this.robo_pose(1))^2 + (this.goal_pos(2)-this.robo_pose(2))^2) < this.goal_thresh;
    end

    % Set robot gait motion models for use in planning & control
    % 
    % Input(s):
    %   a_trajectory  (class instance of locomgmt.locotraj.LocomotionTrajectory)
    %       a_trajectory.timestamps         (array of timestamps at which to command gaits)
    %       a_trajectory.poses              (array of 3-DOF destination poses)
    %       a_trajectory.gait_names         (array of gait names)
    %       a_trajectory.gait_types         (array of gait types: "ROTATE" or "TRANSLATE")
    %       a_trajectory.gait_durations     (array of gait durations in # gait periods)
    %       a_trajectory.gait_directions    (array of gait directions: "CCW", "SE", "NE", "NW", "SW")
    % 
    function set_trajectory_plan( this, a_trajectory )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::set_trajectory_plan()] Set trajectory plan.\n', ...
                this.get_cur_time());
      
      assert( isa(a_trajectory, 'locomgmt.locotraj.LocomotionTrajectory'), ...
              '[OpenLoopLocomotionManager::set_trajectory_plan()] Input trajectory plan  must be of type: locomgmt.locotraj.LocomotionTrajectory\n' );

      % Persist trajectory plan as class property
      this.trajectory_plan.timestamps = a_trajectory.timestamps(1:end-1);   % trim off final timestamp (no action needed)
      this.trajectory_plan.poses = a_trajectory.poses(2:end);               % trim off starting pose
      this.trajectory_plan.gait_names = a_trajectory.gait_names;
      this.trajectory_plan.gait_types = a_trajectory.gait_types;
      this.trajectory_plan.gait_durations = a_trajectory.gait_durations;
      this.trajectory_plan.gait_directions = a_trajectory.gait_directions;

      if ( this.logging_params.data_collect_level > 0 )
        this.data_record.trajectory_plan = this.trajectory_plan;
      end    
    end

    % Set robot gait motion models for use in planning & control
    % 
    % Input(s):
    %   a_gait_models  (boolean value)
    % 
    function set_gait_models( this, a_gait_models )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::set_gait_models()] Set gait motion models.\n', ...
                this.get_cur_time());

      assert( isa(a_gait_models, 'gaitdef.Gait'), ...
              '[OpenLoopLocomotionManager::set_gait_models()] Input is not of type: gaitdef.Gait.');

      this.gait_models = a_gait_models;

      if ( this.logging_params.data_collect_level > 0 )
        this.data_record.gait_models = this.gait_models;
      end      
    end

    function set_cntrl_params( this, a_cntrl_params )
      fprintf('[%.2f sec.] [OpenLoopLocomotionManager::set_clcntrl_params()] Set control parameters.\n', ...
                this.get_cur_time());

    end    

  end     % methods 
end     % class








