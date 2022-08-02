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
%  Dependencies:
%     ivaMatlibs/control/+pathGen (RTGreedyPlanner and MotionPrimitive classes)
%                                 ivaMatlibs/control repository: 
%                                     Web page: https://github.gatech.edu/ivaMatlibs/control/tree/gm_Mwork
%                                     Git URL: git@github.gatech.edu:ivaMatlibs/control.git (branch: gm_Mwork) 
%
%     ivaMatlibs/groups           (SE2 class)
%                                 ivaMatlibs/groups repository: 
%                                     Web page: https://github.gatech.edu/ivaMatlibs/groups
%                                     Git URL: git@github.gatech.edu:ivaMatlibs/groups.git (branch: master) 
%
%     ivalibs/fastmarch           (fastmarch class)
%                                 ivaMatlibs/fastmarch repository: 
%                                     Web page: https://github.gatech.edu/ivalibs/fastmarch
%                                     Git URL: git@github.gatech.edu:ivalibs/fastmarch.git (branch: master) 
%
%  ====================== MSoRoRTPlanner ========================
classdef MSoRoRTPlanner < pathGen.RTGreedyPlanner

  properties (Access = protected)
    rotation_gait;        % gaitdef.Gait instance modeling rotationally-dominant MSoRo gait
    translation_gait;     % gaitdef.Gait instance modeling translationally-dominant MSoRo gait

    % Internal state
    gaits_set;            % boolean flag - signals gaits have been set
    scenario_set;         % boolean flag - signals scenario has been set
  end


  methods (Access = public)

    % Constructor
    function this = MSoRoRTPlanner( params )
      assert( isfield(params, 'gridS'), '[MSoRoRTPlanner::MSoRoRTPlanner()] Missing gridS parameter!');

      % == Super class constructor
      this = this@pathGen.RTGreedyPlanner( params );

      % == Subclass internal properties
      this.rotation_gait = [];
      this.translation_gait = [];

      % == Internal state
      this.gaits_set = false;
      this.scenario_set = false;
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
      trans_max_moves = floor(3/2*pi/a_rot_gait.Twist(3));

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

  end %  methods (public)


  methods (Access = private)

    % Transcribe MP-based controlled trajectory as a MSoRo gait-based controlled trajectory
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
        out_poses(:, ii+1) = [ mp_plan_poses(ii+1).getTranslation() ; mp_plan_poses(ii+1).getAngle() ];

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

    % Plot MSoRo rotation-translation gait-based trajectory plan
    %
    % Input(s):
    %   a_gait_trajectory:              gait-based controlled trajectory (locomgmt.locotraj.LocomotionTrajectory instance) 
    %   a_gait_library:                 array of gaitdef.Gait instances
    %   a_scen_props:                   scenario image (e.g. color, binary or cost map image)
    %     a_scen_props.image              scenario image (color, binary, cost map image) 
    %     a_scen_props.px2cm              pixel-to-cm conversion for image
    %     a_scen_props.radStop            goal threshold (radius in world units)
    %     a_scen_props.startPose          starting pose ([x ; y ; \theta], world coordinates) 
    %     a_scen_props.goalPosition       goal position ([x ; y], world coordinates) 
    %   a_fig_hdl:                      (optional) figure handle
    %   a_msoro_outline                 MSoRo outline coordinates
    % 
    function plotTrajectory( a_gait_trajectory, a_scen_props, a_gait_library, a_fig_hdl, a_msoro_outline )
%       if ( nargin < 5 )
%         a_msoro_outline;
%       else
%         a_msoro_outline = [];
%       end
      if ( nargin < 4 )
        figure;
      else
        figure(a_fig_hdl);
      end

      % Breakout inputs
      scen_img = a_scen_props.image;
      px2cm = a_scen_props.px2cm;
      scen_lims = fliplr(size(scen_img)*px2cm);
      rad_stop = a_scen_props.radStop;
      start_pose = a_scen_props.startPose;
      goal_position = a_scen_props.goalPosition;

      gait_lib = a_gait_library;

      traj_timestamps = a_gait_trajectory.timestamps;           
      traj_poses = a_gait_trajectory.poses;                     
      traj_gait_names = a_gait_trajectory.gait_names;            
      traj_gait_types = a_gait_trajectory.gait_types;           
      traj_gait_durs = a_gait_trajectory.gait_durations;    
      traj_gait_dirs = a_gait_trajectory.gait_directions; 

      % Goal threshold radius/boundary
      theta_rad_samp = linspace(0, 2*pi, 60);
      goal_boundary = [ cos(theta_rad_samp) ; sin(theta_rad_samp) ]*rad_stop + goal_position;

      % Optimal trajectory segments end positions
      marker_clrs = zeros(length(traj_gait_types), 3);
      for ii = 1:length(traj_gait_types)
        if ( traj_gait_types(ii) == gaitdef.GaitType.ROTATE )
          marker_clrs(ii, :) = [1, 0, 0];   % rotation = red
        elseif ( traj_gait_types(ii) == gaitdef.GaitType.TRANSLATE )
          marker_clrs(ii, :) = [0, 1, 0];   % translation = green
        else
          error('[MSoRoRTPlanner::plotTrajectory()] Invalid gait type encountered in trajectory plan: %s.', traj_gait_types(ii));
        end
      end

      % Map gait names to gait library entries
      traj_gait_lib_inds = zeros(1, length(traj_gait_names));
      for ii = 1:length(traj_gait_names)
        for jj = 1:length(gait_lib)
          if ( strcmp(gait_lib(jj).gait_name, traj_gait_names{ii}) )
            traj_gait_lib_inds(ii) = jj;
            break;
          end

          if ( jj == length(gait_lib) )
            error('[MSoRoRTPlanner::plotTrajectory()] Gait name planned in trajectory; not found in gait library: %s', traj_gait_names(ii));
          end
        end
      end

      % Plot trajectory segments for each MP
      num_samples = 50;
      gait_traj = zeros(2, num_samples+1, length(traj_gait_types)); % plus starting position
      for ii = 1:length(traj_gait_types)          
        gait_dur = traj_gait_durs(ii)*(gait_lib(traj_gait_lib_inds(ii)).len_gait*gait_lib(traj_gait_lib_inds(ii)).transition_time);
        gait_twist = gait_lib(traj_gait_lib_inds(ii)).Twist;
        
        if ( traj_gait_dirs(ii) == gaitdef.GaitDir.NW )
          gait_dir_twist = [-gait_twist(2), gait_twist(1), gait_twist(3)];   % [ cm/s, cm/s, rad/s ]
        elseif ( traj_gait_dirs(ii) == gaitdef.GaitDir.SW )
          gait_dir_twist = [-gait_twist(1), -gait_twist(2), gait_twist(3)];  % [ cm/s, cm/s, rad/s ]
        elseif ( traj_gait_dirs(ii) == gaitdef.GaitDir.SE )
          gait_dir_twist = [gait_twist(2), -gait_twist(1), gait_twist(3)];   % [ cm/s, cm/s, rad/s ]
        else
          gait_dir_twist = gait_twist;    % no change for NE, CW or CCW
        end

        g_0 = SE2(traj_poses(1:2, ii), traj_poses(3, ii));
        gait_traj(:, 1, ii) = g_0.getTranslation();
        for jj = 1:num_samples
          g_twist = SE2.exp(gait_dir_twist', gait_dur/num_samples);
          g_next = g_0*g_twist;
          gait_traj(:, jj+1, ii) = g_next.getTranslation();
          
          g_0 = g_next;
        end
      end

      % MSoRo overlay poses
      traj_sample_inds = floor(linspace(1, size(traj_poses, 2), floor(size(traj_poses, 2)/4)));
      msoro_sample_poses = traj_poses(:, traj_sample_inds);
      msoro_overlays = zeros(2, size(a_msoro_outline, 2), length(traj_sample_inds));
      for ii = 1:length(traj_sample_inds)
        msoro_pose = SE2(msoro_sample_poses(1:2, ii), msoro_sample_poses(3, ii));
        msoro_overlays(:, :, ii) = msoro_pose.leftact(a_msoro_outline);
      end

      % Convert coordinates to grid world units (to overlay on scenario
      % image)
      if ( ~isempty(scen_img) )
        start_pose(1:2) = start_pose(1:2)/px2cm;
        goal_position = goal_position/px2cm;
        goal_boundary = goal_boundary/px2cm;
        traj_poses(1:2, :) = traj_poses(1:2, :)/px2cm;
        gait_traj(1:2, :, :) = gait_traj(1:2, :, :)/px2cm;

        msoro_sample_poses(1:2, :) = msoro_sample_poses(1:2, :)/px2cm;
        msoro_overlays = msoro_overlays/px2cm;
      end

      % Plot trajectory plan
      hold on;
        % Cost map
        if ( ~isempty(scen_img) )
          imagesc(scen_img);
        end

        % Start pose and goal position
        start_mrkr_hdl = plot(start_pose(1), start_pose(2), 'md', 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'm', 'MarkerSize', 9);
        goal_mrkr_hdl = plot(goal_position(1), goal_position(2), 'kd', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'MarkerSize', 9);

        % Goal threshold radius/boundary
        plot(goal_boundary(1, :), goal_boundary(2, :), 'k--', 'LineWidth', 1.5);

        for ii = 1:length(traj_sample_inds)
          % Plot MSoRo overlay
          plot(msoro_overlays(1, :, ii), msoro_overlays(2, :, ii), 'y-', 'LineWidth', 2.0);
          bdy_frm_xaxis_hdl = quiver(msoro_sample_poses(1, ii), msoro_sample_poses(2, ii), cos(msoro_sample_poses(3, ii)), sin(msoro_sample_poses(3, ii)), ...
                  'Color', [1,1,0]*0.8, 'LineWidth', 3.5, 'MaxHeadSize', 0.7, ...
                  'AutoScale', 'on', 'AutoScaleFactor', 5);
          quiver(msoro_sample_poses(1, ii), msoro_sample_poses(2, ii), -sin(msoro_sample_poses(3, ii)), cos(msoro_sample_poses(3, ii)), ...
                  'Color', [1,1,0]*0.5, 'LineWidth', 2, 'MaxHeadSize', 0.7, ...
                  'AutoScale', 'on', 'AutoScaleFactor', 5);
        end
        
        % Optimal trajectory segments end positions
        scatter(traj_poses(1, 2:end), traj_poses(2, 2:end), 40, marker_clrs, 'Filled');

        for ii = 1:length(traj_gait_types)          
          % Plot trajectory segments for each MP
          if ( traj_gait_types(ii) == gaitdef.GaitType.ROTATE )           % rotation MP (red)
            rot_traj_plt_hdl = plot(gait_traj(1, :, ii), gait_traj(2, :, ii), 'r-');
          elseif ( traj_gait_types(ii) == gaitdef.GaitType.TRANSLATE )    % translation MP (green)
            trans_traj_plt_hdl = plot(gait_traj(1, :, ii), gait_traj(2, :, ii), 'g-');
          end
        end
 
      hold off;
      axis equal;
      xlabel('X (cm)'); ylabel('Y (cm)');
      disp_xtick_max_cm = scen_lims(1) - mod(scen_lims(1), 100);
      disp_ytick_max_cm = scen_lims(2) - mod(scen_lims(2), 100);
      disp_xticks_cm = 0:100:disp_xtick_max_cm; disp_yticks_cm = 0:100:disp_ytick_max_cm; 
      disp_xticks_grid = disp_xticks_cm/px2cm; disp_yticks_grid = disp_yticks_cm/px2cm; 
      xlim([0, scen_lims(2)]/px2cm); ylim([0, scen_lims(1)]/px2cm); 
      xticks(disp_xticks_grid); yticks(disp_yticks_grid);
      xticklabels(disp_xticks_cm); yticklabels(disp_yticks_cm);
      legend([start_mrkr_hdl, goal_mrkr_hdl, rot_traj_plt_hdl, trans_traj_plt_hdl, bdy_frm_xaxis_hdl], ...
              {'Start', 'Goal', 'Rotate', 'Translate', 'Body Frame $x$-axis'}, ...
              'Location', 'NorthEast', 'Interpreter', 'latex');
      drawnow;
    end

    % Extract robot outline from image
    %
    % Input(s):
    %   a_img_file:              robot image file
    %   a_px2cm:                 image scaling factor
    %   a_num_samps:             number of robot outline points
    %
    % Output(s):
    %   result                   [x ; y], sequence of coordinates
    % 
    function [ result ] = img2msoroOutline( a_img_file, a_px2cm, a_num_samps )
      img = imread(a_img_file);
      bw_img = imbinarize(flipud(img));
      bw_outline_img = bwperim(~bw_img);

      % get outline pixel coordinates
      [y, x] = find(bw_outline_img);
      [min_dist, min_dist_ind] = min(sqrt(x.^2 + y.^2));
      min_x = x(min_dist_ind); min_y = y(min_dist_ind);   % NW-most corner of perimeter

      outline_coords = bwtraceboundary(bw_outline_img, [min_y, min_x], 'E');

      % re-center about outline's centroid
      x = outline_coords(:, 2) - mean(outline_coords(:, 2));        
      y = outline_coords(:, 1) - mean(outline_coords(:, 1));

      x = x*a_px2cm;
      y = y*a_px2cm;

      result = [x' ; y'];
      result = result(:, floor(linspace(1, size(result, 2), a_num_samps)));
    end

  end % methods (static)

end % classdef
