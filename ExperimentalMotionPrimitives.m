% ==================== ExperimentalMotionPrimitives =======================
%
%  Data structure encapsulating experimental motion primitive exploration 
%  data for the MSoRo mobile robot.
%
% 
%
%  ExperimentalMotionPrimitives()
%
%
% ==================== ExperimentalMotionPrimitives =======================
classdef ExperimentalMotionPrimitives
    properties (Access = public)
      % Experimental Setup
      framerate;       % frames per second for visual tracking
      
      pixel_length;    % cm per pixel 
      
      robo_states;     % sequence of robot states e.g., [1, 7, 16, ...]
                       % This will be a [1 x 241] vector corresponding to a
                       % randomized Euler tour.
      
      n_markers;       % number of markers
      
      transition_time  % time constant for motion primitives in seconds
      
      % Raw Experimental Data
      marker_x_pos;    % x position of all markers in pixels 
                       % [n_frames x n_markers]
      marker_y_pos;    % y position of all markers in pixels
      marker_z_pos;    % z position of all markers in pixels 
                       % Assume all z positions are zero for SE(2) motion
                       %(i.e., planar).
                       
      n_frames;        % number of total frames 
      
      timestamps;      % sequence of timestamps (s)

      % Processed Experimental Data
      frame_start;     % frame at which first motion primitive starts
      
      % Rotation matrix and translation vector w.r.t. the global initial
      % frame.
      rotm_global;     % [3 x 3 x n] rotation matrices for every frame n
      p_global;        % [3 x n] translation vectors for every frame n
      
      % Sequence of poses [x y theta]' in the global frame for every frame.
      poses;           % [3 x n] matrix
      
      % Frame numbers for the start of every motion primitive.
      keyframes;       % [k x 1] vector where k = n_frames - 1
      
      % Data for motion primitives reordered by transition number.
      % For a motion primitive = initial state --> final state,
      % translations are w.r.t. the initial state frame. 
      translations;    % [2 x k] matrix of [x y]' translations
      
      rotations;       % [1 x k] vector of heading (yaw) rotations
    end
    
    methods
        % Constructor
        function this = ExperimentalMotionPrimtives( raw_data, Euler_tour, params )
            if (nargin < 3)
                params = [];
            end
            
            % Set default values for experimental setup.
            this.set_property(params, 'framerate', 30);      % frames/sec
            this.set_property(params, 'pixel_length', .2426);  % cm 
            this.set_property(params, 'n_markers', 8);
            this.set_property(params, 'transition_time', .45); % sec
            this.set_property(params, 'frame_start', 207);
            
            % Define order of robot states in the experiment.
            this.robo_states = Euler_tour;
            
            % Extract marker positions from raw data.
            this.marker_x_pos = raw_data(:, 1:3:22);
            this.marker_y_pos = raw_data(:, 2:3:23);
            this.marker_z_pos = raw_data(:, 3:3:24);
            
            % Extract video information.
            this.n_frames = length(raw_data);
            this.timestamps = [0:n_frames -1] / this.framerate;
            
            % Placeholders
            this.rotm_global = [];
            this.p_global = [];
            this.poses = [];
            this.keyframes = [];
            this.translations = [];
            this.rotations = [];
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
      
    end
end