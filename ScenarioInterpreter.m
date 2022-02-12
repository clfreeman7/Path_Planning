% ======================= ScenarioInterpreter =======================
%
%  Image processing utilities collected and applied to intepret an 
%     (overhead) locomotion scenario. Objective is to extract obstacle
%     poses/geometry and return a minimalized scenario description (e.g. 
%     for follow-on robot trajectory planning)
%
%  ScenarioInterpreter()
%
%  INPUTS:
%    N/A
%
%  OUTPUTS:
%
%
%  ====================== ScenarioInterpreter ========================
classdef ScenarioInterpreter < handle
  properties  (Access = public)
    % Scenario properties

    
  end
  
  methods
    % Constructor
    function this = ScenarioInterpreter( params )
      if ( nargin < 1 )
        params = [];
      end
      
    end

    % Segment obstacle poses/geometry from binary image
    %
    % Input(s):
    %   a_bin_img - binary image of scenario; obstacle pixels are non-zero
    %   a_bwareaopen_size - [optional] bwareaopen min. area to retain
    %   a_strel_disk_size - [optional] disk size for morphological expansion
    %
    % Output(s):
    %   obst_map - binary image/map (with dim. = size(a_depth_img))
    %
    function [ obst_config, scenario_img_cleaned ] = get_obstacles_from_image( this, a_bin_img, a_num_obst, a_img_preproc_params )
      % Default parameter values
      if ( nargin < 4 )
        a_img_preproc_params.strel_disk_size = 0;
        a_img_preproc_params.bwareaopen_size = 0;
        a_img_preproc_params.region_ignore = [];
      else
        if ( ~isfield(a_img_preproc_params, 'strel_disk_size') )
          a_img_preproc_params.strel_disk_size = 0;
        end
        if ( ~isfield(a_img_preproc_params, 'bwareaopen_size') )
          a_img_preproc_params.bwareaopen_size = 0;
        end
        if ( ~isfield(a_img_preproc_params, 'region_ignore') )
          a_img_preproc_params.region_ignore = [];
        end
      end
      
      % Output
      obst_config = cell(a_num_obst, 1);

      % Input binary image
      obst_img = uint8(a_bin_img);
      
      % remove small areas of 'noise', then morphologically expand remnants
      if ( a_img_preproc_params.bwareaopen_size > 0 )
        obst_img = bwareaopen(obst_img, a_img_preproc_params.bwareaopen_size);
      end
      
      if ( a_img_preproc_params.strel_disk_size > 0 )
        se = strel('disk', a_img_preproc_params.strel_disk_size);
        obst_img = imclose(obst_img, se);
      end
      
      % ignore specified image regions 
      for ii = 1:size(a_img_preproc_params.region_ignore, 1)
        region_lims = a_img_preproc_params.region_ignore(ii, :);
        
        obst_img(region_lims(1):region_lims(2), region_lims(3):region_lims(4)) = 0;
      end
      
      % filter out largest binary blobs
      obst_img = imfill(obst_img, 'holes');
      obst_img = bwareafilt(obst_img, a_num_obst);

      % Fit bounding boxes to obstacle blobs
      obst_props = regionprops(obst_img, 'basic');
      for ii = 1:length(obst_props)
        obstacle = [];    % reset obstacle properties

        centroid = obst_props(ii).Centroid;
        bbox = obst_props(ii).BoundingBox;
        
        bbox_ul = ceil(bbox(1:2));      % protect against indexing 0
        bbox_size = floor(bbox(3:4));
        
        [Y_obst, X_obst] = find(uint8(obst_img(bbox_ul(2):floor(bbox_ul(2)+bbox_size(2))-1, bbox_ul(1):floor(bbox_ul(1)+bbox_size(1)-1))));   % osbtacle pixel coordinates
        Y_obst_cent = centroid(2)-bbox_ul(2);   % centroid in the bounding box
        X_obst_cent = centroid(1)-bbox_ul(1);
        
        % PCA-based obstacle body frame & pose estimate
        [R_obst, ~, eig_val] = pca( [X_obst Y_obst] );
        
        % Enforce right-handedness of frame/rotation
        if ( det(R_obst) < 0 )
          R_tmp = R_obst;
          R_obst = [R_tmp(:, 2), R_tmp(:, 1)];
        end
        
        % Obstacle body frame SE(2) pose
        obstacle.pose = [ centroid(1) ; centroid(2) ; atan2(R_obst(2, 1), R_obst(1, 1)) ];  % [x ; y ; theta]
        
        % Find rectangular obstacle length (x-) / width (y-)
        obst_g_bb = [ R_obst [X_obst_cent ; Y_obst_cent] ;
                        0 0 1];
        
        obst_pos_bb_local = inv(obst_g_bb)*[ [X_obst, Y_obst]' ; ones(1, length(X_obst)) ];
        obst_max_x = max(abs(obst_pos_bb_local(1, :)));
        obst_max_y = max(abs(obst_pos_bb_local(2, :)));
        obstacle.rect.x = obst_max_x;
        obstacle.rect.y = obst_max_y;

        % Obstacle results
        obst_config{ii} = obstacle;
      end
      
      scenario_img_cleaned = uint8(obst_img);
    end

    % Estimate (rectangular) obstacle dimensions and poses
    function [ obstacle_scenario ] = extract_obstacle_config( this, a_scenario_img, a_scenario_params, a_img_preproc_params )
      % Validate input arguments
      if ( ~isfield(a_scenario_params, 'num_obstacles') )
        error('[ScenarioInterpreter::model_obstacles()] Scenario parameters missing num_obstacles!');
      elseif ( ~isfield(a_scenario_params, 'px2mm') )
        error('[ScenarioInterpreter::model_obstacles()] Scenario parameters missing px2mm!');
      end
      num_obst = a_scenario_params.num_obstacles;
      PX2MM = a_scenario_params.px2mm;

      if ( ~isfield(a_scenario_params, 'obstacle_dilation') )
        obst_dilation = 0;
      else
        obst_dilation = a_scenario_params.obstacle_dilation;
      end

      % Extract obstacle configuration from scenario image (image space representation)
      [ obst_config_px, scenario_img_cleaned ] = ...
            this.get_obstacles_from_image( a_scenario_img, num_obst, a_img_preproc_params );
      
      % Convert obstacle data from image space (x-left, y-down, pixels) to conventional spatial
      % frame (x-left, y-up, mm)
      img_size_x = size(a_scenario_img, 2);
      img_size_y = size(a_scenario_img, 1);

      obstacle_scenario = [];
      obstacle_scenario.obstacles = cell(size(obst_config_px));
      for ii = 1:length(obst_config_px)
        % image space/frame representation
        obstacle_px = obst_config_px{ii};
        obst_pose_px = obstacle_px.pose;
        obst_xdim_px = obstacle_px.rect.x;
        obst_ydim_px = obstacle_px.rect.y;

        % Convert to conventional spatial frame
        obst_pose_mm = [ obst_pose_px(1)*PX2MM ; (img_size_y-obst_pose_px(2))*PX2MM ; -obst_pose_px(3) ];

        obst_pose_homog_mm = [ cos(obst_pose_mm(3)), -sin(obst_pose_mm(3)), obst_pose_mm(1) ; ...
                                sin(obst_pose_mm(3)), cos(obst_pose_mm(3)), obst_pose_mm(2) ; ...
                                0, 0, 1 ];
        
        obst_xdim_mm = obst_xdim_px*PX2MM;
        obst_ydim_mm = obst_ydim_px*PX2MM;
        dilated_obst_xdim_mm = obst_xdim_mm+obst_dilation;    % dilated dimensions
        dilated_obst_ydim_mm = obst_ydim_mm+obst_dilation;

        % Organize results
        obstacle = [];
        %   Coventional spatial frame
        obstacle.pose_mm = obst_pose_mm;
        obstacle.pose_homog_mm = obst_pose_homog_mm;
        obstacle.rect_dims_mm.x = obst_xdim_mm;
        obstacle.rect_dims_mm.y = obst_ydim_mm;
        obstacle.dilated_rect_dims_mm.x = dilated_obst_xdim_mm;
        obstacle.dilated_rect_dims_mm.y = dilated_obst_ydim_mm;
        %   Image frame
        obstacle.pose_px = obst_pose_px;
        obstacle.pose_homog_px = [ cos(obst_pose_px(3)), -sin(obst_pose_px(3)), obst_pose_px(1) ; ...
                                    sin(obst_pose_px(3)), cos(obst_pose_px(3)), obst_pose_px(2) ; ...
                                    0, 0, 1 ];
        obstacle.rect_dims_px.x = obst_xdim_px;
        obstacle.rect_dims_px.y = obst_ydim_px;
        obstacle.dilated_rect_dims_px.x = dilated_obst_xdim_mm/PX2MM;
        obstacle.dilated_rect_dims_px.y = dilated_obst_ydim_mm/PX2MM;

        obstacle_scenario.obstacles{ii} = obstacle;
      end
      
      % Organize obstacle configuration results
      obstacle_scenario.scenario_img_clean = scenario_img_cleaned;
      obstacle_scenario.num_obst = num_obst;
      obstacle_scenario.px2mm = PX2MM;
      obstacle_scenario.obstacle_dilation = obst_dilation;
    end

    % Plot obstacle scenario
    function plot_obstacle_config( this, a_obstacle_scenario, a_vis_hdl, a_plot_img_space )
      if (nargin < 4)
        a_plot_img_space = false;     % Plot w.r.t. conventional spatial frame
      end

      PX2MM = a_obstacle_scenario.px2mm;
      obstacles = a_obstacle_scenario.obstacles;

      % Plot virtual world (spatial y-axis flipped; obstacle data 
      % transformed to accomodate)
      if ( isa(a_vis_hdl, 'matlab.ui.Figure') )
        figure(a_vis_hdl);
      elseif ( isa(a_vis_hdl, 'matlab.graphics.axis.Axes') )
        subplot(a_vis_hdl);
      end
      x_lims_px = [0 size(a_obstacle_scenario.scenario_img_clean, 2)];
      y_lims_px = [0 size(a_obstacle_scenario.scenario_img_clean, 1)];
      x_lims_mm = [0 x_lims_px(2)]*PX2MM;
      y_lims_mm = [0 y_lims_px(2)]*PX2MM;
      
      hold on;
        for ii = 1:length(obstacles)
          obstacle = obstacles{ii};

          g_obst_pose_homog_mm = obstacle.pose_homog_mm;
          d_obst_mm = g_obst_pose_homog_mm(1:2, 3);
          R_obst_mm = g_obst_pose_homog_mm(1:2, 1:2);
          obst_xdim_mm = obstacle.rect_dims_mm.x;
          obst_ydim_mm = obstacle.rect_dims_mm.y;
          dilated_obst_xdim_mm = obstacle.dilated_rect_dims_mm.x;
          dilated_obst_ydim_mm = obstacle.dilated_rect_dims_mm.y;
  
          g_obst_pose_homog_px = obstacle.pose_homog_px;
          d_obst_px = g_obst_pose_homog_px(1:2, 3);
          R_obst_px = g_obst_pose_homog_px(1:2, 1:2);
          obst_xdim_px = obstacle.rect_dims_px.x;
          obst_ydim_px = obstacle.rect_dims_px.y;
          dilated_obst_xdim_px = obstacle.dilated_rect_dims_px.x;
          dilated_obst_ydim_px = obstacle.dilated_rect_dims_px.y;
          
          if ( a_plot_img_space )
            % Plot obstacle frames
            quiver(d_obst_px(1), d_obst_px(2), R_obst_px(1, 1), R_obst_px(2, 1), ...
                    'Color', 'r', 'LineWidth', 1, 'MaxHeadSize', 5, ...
                    'AutoScale', 'on', 'AutoScaleFactor', 50);
            quiver(d_obst_px(1), d_obst_px(2), -R_obst_px(1, 2), -R_obst_px(2, 2), ...    % flip y-axis for display
                    'Color', 'g', 'LineWidth', 1, 'MaxHeadSize', 5, ...
                    'AutoScale', 'on', 'AutoScaleFactor', 50);
            
            % Plot obstacle boundaries
            rect_corners = [-obst_xdim_px, obst_xdim_px, obst_xdim_px, -obst_xdim_px, -obst_xdim_px ; ...
                            obst_ydim_px, obst_ydim_px, -obst_ydim_px, -obst_ydim_px, obst_ydim_px ; ...
                            ones(1, 5) ];
            rect_corners = g_obst_pose_homog_px*rect_corners;
            plot(rect_corners(1, :), rect_corners(2, :), 'r-', 'LineWidth', 1);
                  
            rect_corners = [-dilated_obst_xdim_px, dilated_obst_xdim_px, dilated_obst_xdim_px, -dilated_obst_xdim_px, -dilated_obst_xdim_px ; ...
                            dilated_obst_ydim_px, dilated_obst_ydim_px, -dilated_obst_ydim_px, -dilated_obst_ydim_px, dilated_obst_ydim_px ; ...
                            ones(1, 5) ];
            rect_corners = g_obst_pose_homog_px*rect_corners;
            plot(rect_corners(1, :), rect_corners(2, :), 'g--', 'LineWidth', 1); 
          else
            % Plot obstacle frames
            quiver(d_obst_mm(1), d_obst_mm(2), R_obst_mm(1, 1), R_obst_mm(2, 1), ...
                    'Color', 'r', 'LineWidth', 1, 'MaxHeadSize', 5, ...
                    'AutoScale', 'on', 'AutoScaleFactor', 50);
            quiver(d_obst_mm(1), d_obst_mm(2), R_obst_mm(1, 2), R_obst_mm(2, 2), ...
                    'Color', 'g', 'LineWidth', 1, 'MaxHeadSize', 5, ...
                    'AutoScale', 'on', 'AutoScaleFactor', 50);
            
            % Plot obstacle boundaries
            rect_corners = [-obst_xdim_mm, obst_xdim_mm, obst_xdim_mm, -obst_xdim_mm, -obst_xdim_mm ; ...
                            obst_ydim_mm, obst_ydim_mm, -obst_ydim_mm, -obst_ydim_mm, obst_ydim_mm ; ...
                            ones(1, 5) ];
            rect_corners = g_obst_pose_homog_mm*rect_corners;
            plot(rect_corners(1, :), rect_corners(2, :), 'r-', 'LineWidth', 1);
                  
            rect_corners = [-dilated_obst_xdim_mm, dilated_obst_xdim_mm, dilated_obst_xdim_mm, -dilated_obst_xdim_mm, -dilated_obst_xdim_mm ; ...
                            dilated_obst_ydim_mm, dilated_obst_ydim_mm, -dilated_obst_ydim_mm, -dilated_obst_ydim_mm, dilated_obst_ydim_mm ; ...
                            ones(1, 5) ];
            rect_corners = g_obst_pose_homog_mm*rect_corners;
            plot(rect_corners(1, :), rect_corners(2, :), 'g--', 'LineWidth', 1); 
          end
        end
      hold off;
      axis equal;
      xlabel('X (m)'); ylabel('Y (m)'); 
      if ( a_plot_img_space )   % image space-adjusted axis labels
        xlim(x_lims_px); ylim(y_lims_px);
        xticks(x_lims_px); yticks(y_lims_px);
        xticklabels(x_lims_mm/1000); yticklabels(fliplr(y_lims_mm)/1000);
%         set(gca,'ydir','normal');
      else                      % conventional axis labels 
        xlim(x_lims_mm); ylim(y_lims_mm);
        xticks(x_lims_mm); yticks(y_lims_mm);
        xticklabels(x_lims_mm/1000); yticklabels(fliplr(y_lims_mm)/1000);
      end
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

end     % class


