% ======================= MSoRoRTPlanner =======================
%
%  Motion primitive-based controlled trajectory planner. 
%  Trajectory planning uses a grid world representation and takes a 
%  'greedy' search-based approach, finding a sequence of 
%  rotation-translation motion primitives that, collectively, 
%  travel from start to goal. 
%
%  planning.MSoRoRTPlanner()
%
%  TODO:
%   (X) helper method -> cost map synthesis from binary scenario image
%   (X) set goal thresh/radius
%   (3) MSoRo layer + demo script
%   (4) visualization with MSoRo overlay
%
%  ====================== MSoRoRTPlanner ========================
classdef MSoRoRTPlanner < pathGen.RTGreedyPlanner

  properties (Access = protected)
    rotation_gait;            % gaitdef.Gait instance modeling rotationally-dominant MSoRo gait
    translation_gait;         % gaitdef.Gait instance modeling translationally-dominant MSoRo gait

    % Visualization/debug aids
%     vis_config;     % (struct) visualization configuration
%     vis_hdls;       % (struct) visualization figure and plot handles

    % Internal state
    gaits_set;
    scenario_set;
  end


  methods (Access = public)

    % Constructor
    function this = MSoRoRTPlanner( params )
      assert( isfield(params, 'gridS'), '[RTGreedyPlanner::RTGreedyPlanner() Missing gridS parameter!]');

      % == Super class constructor
      this = this@pathGen.RTGreedyPlanner( params );

      % == Subclass internal properties
      this.rotation_gait = [];
      this.translation_gait = [];

      % == Internal state
      this.gaits_set = false;
      this.scenario_set = false;

      % == Initialize visualization state
      this.vis_config.mode = 0;
      this.vis_hdls = [];
    end

    % Set rotationally-dominant and translationally-dominant gaits for trajectory planning
    %
    % Input(s):
    %   a_rot_gait:             Rotational gait model used for trajectory planning
    %                           (class instance: gaitdef.Gait) 
    %   a_trans_gait:           Translational gait model used for trajectory planning
    %                           (class instance: gaitdef.Gait) 
    % 
    function setGaits( this, a_rot_gait, a_trans_gait )
      assert ( isa(a_rot_gait, 'gaitdef.Gait'), ...
                  '[MSoRoRTPlanner::setRotationGait()] Added gait must be of type gaitdef.Gait.');
      assert ( isa(a_trans_gait, 'gaitdef.Gait'), ...
                  '[MSoRoRTPlanner::setTranslationGait()] Added gait must be of type gaitdef.Gait.');

      % Rotational gait model
      this.rotation_gait = a_rot_gait;

      rot_gait_period = a_rot_gait.len_gait*a_rot_gait.transition_time;

      % Transcribe rotate gait data as motion primitive
      rot_mp_params.name = a_rot_gait.gait_name;
      rot_mp_params.body_vel_twist = a_rot_gait.Twist;
      rot_mp_params.dt = rot_gait_period;
      rot_mp_params.max_moves = floor(pi/2 / abs(rot_mp_params.body_vel_twist(3)*rot_mp_params.dt) );    % number of time periods to rotate up to 90 deg.
      rotate_mp = pathGen.MotionPrimitive( rot_mp_params );
      this.addRotationMP( rotate_mp );

      % Translational gait model
      this.translation_gait = a_trans_gait;

      trans_speed = norm(a_trans_gait.Twist(1:2));
      trans_gait_period = a_trans_gait.len_gait*a_trans_gait.transition_time;
      grid_size = this.dg;
      trans_dt = trans_gait_period*ceil(grid_size/(trans_speed*trans_gait_period));
      trans_max_moves = floor(2*pi/a_rot_gait.Twist(3));

      % Transcribe translate gait data as symmetrically-permutated motion primitives
      %   Linear velocity rotated 0 deg.
      trans_mp_params.name = sprintf('%s_R000', a_trans_gait.gait_name);
      trans_mp_params.body_vel_twist = a_trans_gait.Twist;   % [ cm/s, cm/s, rad/s ]
      trans_mp_params.dt = trans_dt;
      trans_mp_params.max_moves = trans_max_moves;    % number of time periods to translate
      trans_mp = pathGen.MotionPrimitive( trans_mp_params );
      this.addTranslationMP( trans_mp );

      %   Linear velocity rotated 90 deg.
      trans_mp_params.name = sprintf('%s_R090', a_trans_gait.gait_name);
      trans_mp_params.body_vel_twist = [-a_trans_gait.Twist(2), a_trans_gait.Twist(1), a_trans_gait.Twist(3)];   % [ cm/s, cm/s, rad/s ]
      trans_mp_params.dt = trans_dt;
      trans_mp_params.max_moves = trans_max_moves;    % number of time periods to translate
      trans_mp = pathGen.MotionPrimitive( trans_mp_params );
      this.addTranslationMP( trans_mp );

      %   Linear velocity rotated 180 deg.
      trans_mp_params.name = sprintf('%s_R180', a_trans_gait.gait_name);
      trans_mp_params.body_vel_twist = [-a_trans_gait.Twist(1), -a_trans_gait.Twist(2), a_trans_gait.Twist(3)];   % [ cm/s, cm/s, rad/s ]
      trans_mp_params.dt = trans_dt;
      trans_mp_params.max_moves = trans_max_moves;    % number of time periods to translate
      trans_mp = pathGen.MotionPrimitive( trans_mp_params );
      this.addTranslationMP( trans_mp );

      %   Linear velocity rotated 270 deg.
      trans_mp_params.name = sprintf('%s_R270', a_trans_gait.gait_name);
      trans_mp_params.body_vel_twist = [a_trans_gait.Twist(2), -a_trans_gait.Twist(1), a_trans_gait.Twist(3)];   % [ cm/s, cm/s, rad/s ]
      trans_mp_params.dt = trans_dt;
      trans_mp_params.max_moves = trans_max_moves;    % number of time periods to translate
      trans_mp = pathGen.MotionPrimitive( trans_mp_params );
      this.addTranslationMP( trans_mp );

      this.gaits_set = true;
    end

    % Set (binary) scenario image (1 = obstacles, 0 = empty space) and
    % associated cost map
    %
    % Input(s):
    %   a_bin_scenario_img     Binary scenario image ef.Gait)
    %   a_rad_falloff          Radial decay constant for obstacle costs (grid units)
    % 
    % Output(s):
    %   cf                     Cost map
    % 
    function [ cf ] = setScenario( this, a_bin_scenario_img, a_rad_falloff )
      assert ( size(a_bin_scenario_img, 3) == 1, ...
                  '[MSoRoRTPlanner::setScenario()] Input scenario must be a binary image.');

      % Convert binary image to cost map
      img_scaling = max(this.gridS.size)/max(size(a_bin_scenario_img));    % (binary) scenario image -> grid world scaling
      cf = pathGen.RTGreedyPlanner.binImg2CostMap(a_bin_scenario_img, img_scaling, a_rad_falloff);

      this.setCostMap(cf);  % cost map encodes obstacle configuration

      this.scenario_set = true;
    end

    % Plan controlled gait trajectory from starting SE(2) pose, 
    % to goal position (i.e. E(2))
    %
    % Input(s):
    %   a_start_pose:           planar starting pose
    %                           (class instance: SE2) 
    %   a_goal_position:        2-D goal position, [x, y]
    % 
    % Output(s):
    %   gait_trajectory_plan:        gait-based trajectory plan (locomgmt.locotraj.LocomotionTrajectory() instance) 
    % 
    function [ gait_trajectory_plan ] = planTrajectory( this, a_start_pose, a_goal_position )
      assert ( isa(this.start_pose, 'SE2'), ...
                  '[MSoRoRTPlanner::planTrajectory()] Specified start pose is not of type SE2.');
      assert ( length(a_goal_position) == 2, ...
                  '[MSoRoRTPlanner::planTrajectory()] Specified goal position is not length 2.');

      % Check planner is properly setup
      planner_primed = true;
      if ( ~this.gaits_set )
        warning('[MSoRoRTPlanner::planTrajectory()] Gaits not setup.');
        planner_primed = false;
      end
      if ( ~this.scenario_set )
        warning('[MSoRoRTPlanner::planTrajectory()] Scenario not set.');
        planner_primed = false;
      end
      if ( ~planner_primed )
        error('[MSoRoRTPlanner::planTrajectory()] Planner not properly setup.');
      end

      % Plan the underlying MP-based controlled trajectory
      this.pose2point( a_start_pose, a_goal_position );   % call super class trajectory planner
      mp_trajectory_plan = this.getTrajectoryPlan();  % retrieve MP controlled trajectory plan from super class

      % Transcribe MP-based controlled trajectory outcome as a MSoRo gait-based controlled trajectory
      gait_trajectory_plan = this.mp2GaitTrajectory( mp_trajectory_plan );
    end

    % Enable/disable and configure visualization (debug aid)
    %
    % Input(s):
    %   a_vis_config:           visualization config.
    %     a_vis_config.mode     visualization mode (0 = none, 1 = animate)
    %   a_fig_hdl:              (optional) figure handle
    % 
    function configureVisualization( this, a_vis_config, a_fig_hdl )
      if ( nargin < 3 )
        this.vis_hdls.fig_hdl = figure;
      else
        this.vis_hdls.fig_hdl = a_fig_hdl;
      end

      this.vis_config.mode = a_vis_config.mode;

      % Animate search space exploration
      if ( this.vis_config.mode == 1 )
        figure(this.vis_hdls.fig_hdl);
        hold on;
          % Cost map
          if ( ~isempty(this.costMap) )  % logic in case method called prior to cost map assignment
            this.vis_hdls.costmap_img_hdl = imagesc(this.costMap);
            this.vis_hdls.costmap_visualized = true;
          else
            this.vis_hdls.costmap_img_hdl = imagesc(zeros(1, 1));
            this.vis_hdls.costmap_visualized = false;
          end

          % Explored trajectory segment end positions
          this.vis_hdls.cand_traj_plt_hdl = plot(0, 0, 'ro');
          this.vis_data.cand_traj_pnts = [];

          % Identified optimal trajectory segment end positions
          this.vis_hdls.opt_traj_plt_hdl = plot(0, 0, 'go', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');
          this.vis_data.opt_traj_pnts = [];

          % Start and goal positions
          this.vis_hdls.start_pos_hdl = plot(0, 0, 'gd', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');
          this.vis_hdls.start_visualized = false;
          this.vis_hdls.goal_pos_hdl = plot(0, 0, 'kd', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
          this.vis_hdls.goal_rad_hdl = plot(0, 0, 'k-');
          this.vis_hdls.goal_visualized = false;
        hold off;
        axis equal;
        xlabel('X (grid units)'); ylabel('Y (grid units)');
        this.vis_hdls.title_hdl = title(sprintf('Grid unit = %.4f cm', this.dg));  % TODO: units should be set per user
        drawnow;
      end
    end

  end %  methods (public)


  methods (Access = private)

    % Transcribe MP-based controlled trajectory outcome as a MSoRo gait-based controlled trajectory
    %
    % Input(s):
    %   a_mp_trajectory:                MP-based controlled trajectory (custom struct) 
    %     a_mp_trajectory.g_poses         array (sequence) of poses (SE2() instances)
    %     a_mp_trajectory.mps             array (sequence) of MPs (pathGen.MotionPrimitive() instances)   
    %     a_mp_trajectory.mp_moves        array (sequence) of MP moves (integers)  
    %     a_mp_trajectory.mp_type         array (sequence) of MP classifications (strings): 
    %                                               'T' for translation or 'R' for rotation 
    %
    % Output(s):
    %   gait_trajectory:                gait-based controlled trajectory (locomgmt.locotraj.LocomotionTrajectory instance) 
    %
    function [ gait_trajectory ] = mp2GaitTrajectory( this, a_mp_trajectory )
      % Break-out input struct fields
      mp_plan_poses = a_mp_trajectory.g_poses;
      mp_plan_mps = a_mp_trajectory.mps;
      mp_plan_moves = a_mp_trajectory.mp_moves;
      mp_plan_mp_types = a_mp_trajectory.mp_type;

      % Output gait-based trajectory data type
      gait_trajectory = locomgmt.locotraj.LocomotionTrajectory();

      % Transcribe MP-based trajectory -> gait-based trajectory
      trans_gait_period = this.translation_gait.len_gait*this.translation_gait.transition_time;
      rot_gait_period = this.rotation_gait.len_gait*this.rotation_gait.transition_time;

      out_timestamps = zeros(1, length(mp_plan_mps)+1);
      out_poses = zeros(3, length(mp_plan_mps)+1);
      out_gait_names = cell(1, length(mp_plan_mps));
      out_gait_types = gaitdef.GaitType.empty(1, 0); 
      out_gait_durations = zeros(1, length(mp_plan_mps));
      out_gait_directions = gaitdef.GaitDir.empty(1, 0);

      out_timestamps(1) = 0;
      out_poses(:, 1) = [ mp_plan_poses(1).getTranslation() ; mp_plan_poses(1).getAngle() ];
      for ii = 1:length(mp_plan_mps)
        out_poses(:, ii+1) = [ mp_plan_poses(ii).getTranslation() ; mp_plan_poses(ii).getAngle() ];

        if ( mp_plan_mp_types(ii) == 'T' )  % translate gait
          out_gait_types(ii) = gaitdef.GaitType.TRANSLATE;
          out_gait_durations(ii) = mp_plan_moves(ii)*round(mp_plan_mps(ii).dt/trans_gait_period);
          out_timestamps(ii+1) = out_timestamps(ii) + out_gait_durations(ii)*trans_gait_period;

          trans_mp_dir_str = mp_plan_mps(ii).name(end-2:end);     % translate symmetrically permutated gait (retain permutation indicator) 
          out_gait_names{ii} = mp_plan_mps(ii).name(1:end-5);     % translate symmetrically permutated gait (remove permutation indicator)
          if ( strcmp(trans_mp_dir_str, '000') )
            out_gait_directions(ii) = gaitdef.GaitDir.NE;     % [TODO] no longer seems to be a relevant enumeration value
          elseif ( strcmp(trans_mp_dir_str, '090') )
            out_gait_directions(ii) = gaitdef.GaitDir.NW;
          elseif ( strcmp(trans_mp_dir_str, '180') )
            out_gait_directions(ii) = gaitdef.GaitDir.SW;
          elseif ( strcmp(trans_mp_dir_str, '270') )
            out_gait_directions(ii) = gaitdef.GaitDir.SE;
          else
            error('[MSoRoRTPlanner::mp2GaitTrajectory()] Encountered translational gait name with invalid symmetric permutation indicator: %s.', mp_plan_mps(ii).name);
          end

        elseif ( mp_plan_mp_types(ii) == 'R' ) % rotate gait
          out_gait_types(ii) = gaitdef.GaitType.ROTATE;
          out_gait_durations(ii) = mp_plan_moves(ii);
          out_timestamps(ii+1) = out_timestamps(ii) + out_gait_durations(ii)*rot_gait_period;

          out_gait_names{ii} = mp_plan_mps(ii).name;
          if ( mp_plan_mps(ii).body_vel_twist(3) > 0 )
            out_gait_directions(ii) = gaitdef.GaitDir.CCW;
          else
            out_gait_directions(ii) = gaitdef.GaitDir.CW;
          end
        else
          error('[MSoRoRTPlanner::mp2GaitTrajectory()] Encountered invalid mp_type value generated MP controlled trajectory plan: %s.', mp_plan_mp_types(ii));
        end
      end

      gait_trajectory.timestamps = out_timestamps;            % sequence of timestamps
      gait_trajectory.poses = out_poses;                      % sequence of robot poses [x ; y ; theta]
      gait_trajectory.gait_names = out_gait_names;            % cell array (or array) of chars representing
      gait_trajectory.gait_types = out_gait_types;            % sequence of +gaitdef.GaitType enumerations (TRANSLATE, ROTATE)
      gait_trajectory.gait_durations = out_gait_durations;    % corresponding number of gait periods
      gait_trajectory.gait_directions = out_gait_directions;  % sequence of +gaitdef.GaitDir enumerations (NE, NW, SW, SE, CW, CCW)
    end

  end %  methods (private)


  methods (Static)

    % Plot rotation-translation-paradigmed trajectory plan [TODO]
    %
    % Input(s):
    %   a_motion_plan:                visualization config.
    %     motion_plan.cost_map          cost map
    %     motion_plan.start_pose        starting SE2 pose  
    %     motion_plan.goal_position     goal position, [x ; y]
    %     motion_plan.g_poses           discrete-time trajectory poses
    %     motion_plan.optimal_rt_seq    rotation-translation sequences to
    %                                   acccomplishing planned trajectory poses
    %     motion_plan.goal_thresh       goal radius
    %     motion_plan.dg                cost map grid size
    %   a_fig_hdl:                    (optional) figure handle
    % 
    function plotTrajectory( a_motion_plan, a_fig_hdl )
      if ( nargin < 3 )
        figure;
      else
        figure(a_fig_hdl);
      end

      a_cost_map = a_motion_plan.cost_map;
      a_gridS = a_motion_plan.gridS;
      a_dg = a_gridS.dg;
      a_start_pose = a_motion_plan.start_pose;
      a_goal_position = a_motion_plan.goal_position;
      a_goal_thresh = a_motion_plan.goal_thresh;
      a_g_poses = a_motion_plan.g_poses;
      a_mp_type = a_motion_plan.mp_type;
      a_mp_list = a_motion_plan.mps;            % MotionPrimitive()
      a_mp_durations = a_motion_plan.mp_durations;

      % Compute world2grid mapping
      for ii=1:length(a_gridS.size)
        a_grids{ii} = a_gridS.cmin(ii) + a_dg*[0:(a_gridS.size(ii)-1)];
      end
      for ii=1:length(a_gridS.size)
        map_w2g{ii} = griddedInterpolant(a_grids{ii},[1:a_gridS.size(ii)]);
      end
      map_world2grid = @(x) [map_w2g{1}(x(1,:)); map_w2g{2}(x(2,:))];

      hold on;
        % Cost map
        imagesc(a_cost_map);

        % Start and goal positions
        start_grid_position = map_world2grid(a_start_pose.getTranslation());
        plot(start_grid_position(1), start_grid_position(2), 'md', 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'm');

        goal_grid_position = map_world2grid(a_goal_position);
        plot(goal_grid_position(1), goal_grid_position(2), 'kd', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');

        theta_rad_samp = linspace(0, 2*pi, 60);     % Goal threshold radius/boundary
        goal_boundary = [ cos(theta_rad_samp) ; sin(theta_rad_samp) ]*a_goal_thresh/a_dg + goal_grid_position;
        plot(goal_boundary(1, :), goal_boundary(2, :), 'k--', 'LineWidth', 1.5);

        % Optimal trajectory segments end positions
        traj_grid_positions = zeros(2, length(a_g_poses)-1);
        marker_clrs = zeros(length(a_g_poses)-1, 3);
        for ii = 2:length(a_g_poses)
          if ( a_mp_type(ii-1) == 'R' )
            marker_clrs(ii-1, :) = [1, 0, 0];   % rotation = red
          elseif ( a_mp_type(ii-1) == 'T' )
            marker_clrs(ii-1, :) = [0, 1, 0];   % translation = green
          else
            error('[RTGreedyPlanner::plot_trajectory()] Invalid MP type in input motion plan: %s.', a_mp_type(ii));
          end
          traj_grid_positions(:, ii-1) = map_world2grid(a_g_poses(ii).getTranslation());
        end
        scatter(traj_grid_positions(1,:), traj_grid_positions(2, :), 40, marker_clrs, 'filled');

        % Plot trajectory segments for each MP
        num_samples = 50;
        mp_grid_traj = zeros(2, num_samples+1); % plus starting position
        for ii = 1:length(a_mp_type)
          mp_twist = a_mp_list(ii).body_vel_twist;
          mp_dt = a_mp_list(ii).dt;
          mp_dur = a_mp_durations(ii);
          g_0 = a_g_poses(ii);
          mp_grid_traj(:, 1) = map_world2grid(g_0.getTranslation());
          for jj = 1:num_samples
            g_twist = SE2.exp(mp_twist', mp_dur*mp_dt/num_samples);
            g_next = g_0*g_twist;
            mp_grid_traj(:, jj+1) = map_world2grid(g_next.getTranslation());
            g_0 = g_next;
          end
          if ( a_mp_type(ii) == 'R' )       % rotation MP (red)
            plot(mp_grid_traj(1, :), mp_grid_traj(2, :), 'r-');
          elseif ( a_mp_type(ii) == 'T' )   % translation MP (green)
            plot(mp_grid_traj(1, :), mp_grid_traj(2, :), 'g-');
          else
            error('[RTGreedyPlanner::plot_trajectory()] Invalid MP type in input motion plan: %s.', a_mp_type(ii));
          end
        end
  
      hold off;
      axis equal;
      xlabel('X (grid units)'); ylabel('Y (grid units)');
      title(sprintf('Grid unit = %.4f cm', a_dg));  % TODO: units should be set per user
      drawnow;
    end

  end % methods (static)

end % classdef
