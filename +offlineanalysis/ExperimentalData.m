% ==================== ExperimentalData =======================
%
%  Data structure encapsulating experimental visual tracking 
%  data for the MSoRo mobile robot.
%
% 
%
%  ExperimentalData()
%
%
% ==================== ExperimentalData =======================
classdef ExperimentalData < handle
    properties (Access = public)
      % Experimental Setup
      robot_name;      % label for robot (either blue or orange)  
      
      substrate;       % name for locomotion surface (e.g., black mat)
      
      framerate;       % frames per second for visual tracking
      
      pixel_length;    % cm per pixel 
      
      n_markers;       % number of markers
      
      transition_time;  % time constant for motion primitives in seconds
      
      % Raw Experimental Data
      marker_x_pos;    % x position of all markers in pixels 
                       % [n_frames x n_markers]
      marker_y_pos;    % y position of all markers in pixels
      marker_z_pos;    % z position of all markers in pixels 
                       % Assume all z positions are zero for SE(2) motion
                       %(i.e., planar).
                       
      n_frames;        % number of total frames 
      
      timestamps;      % sequence of timestamps for each frame (s)
      
      marker_order;    % order of the markers where marker 1 is closest to
                       % main limb marker and the labels increase CW.
                       
      frame_1;         % frame at which motion 
      timestamp_1;     % timestamp at which motion starts
      
      R_1;             % initial rotation matrix compared to trial 1
      
      % Processed Experimental Data `   
      
      % Rotation matrix and translation vector w.r.t. the global initial
      % frame.
      rotm_global;     % [3 x 3 x n] rotation matrices for every frame n
      t_global;        % [3 x n] translation vectors for every frame n
      
      % Sequence of poses [x y theta]' in the global frame for every frame.
      poses;           % [3 x n] matrix
    end 
    
    methods
        % Constructor
        function this = ExperimentalData( raw_data, params )
            if ( nargin < 2 )
                params = [];
            end
            
            % Set default/ input values for experimental setup.
            this.set_property(params, 'robot_name', 'undefined');
            this.set_property(params, 'substrate', 'undefined');
            this.set_property(params, 'framerate', 30);      % frames/sec
            this.set_property(params, 'pixel_length', .2426);  % cm/pixel
            this.set_property(params, 'n_markers', 8);
            this.set_property(params, 'transition_time', .45); % sec   
            this.set_property(params, 'marker_order', 1:this.n_markers);
            this.set_property(params, 'frame_1', 1);
            this.set_property(params, 'R_1', eye(3));
            
            % Extract marker positions from raw data.
            this.marker_x_pos = raw_data(:, 1:3:22);
            this.marker_y_pos = raw_data(:, 2:3:23);
            this.marker_z_pos = raw_data(:, 3:3:24);
            
            % Reoraganize markers if the marker order is not default.
            this.marker_x_pos = this.marker_x_pos(:, this.marker_order);
            this.marker_y_pos = this.marker_y_pos(:, this.marker_order);
            this.marker_z_pos = this.marker_z_pos(:, this.marker_order);
            
            % Extract video information.
            this.n_frames = length(raw_data);
            this.timestamps = [0: (length(raw_data)-1)] / this.framerate; 
            this.timestamp_1 = this.timestamps(this.frame_1);
            
            % Placeholders
            this.process_data(raw_data);
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
        
        % Find the rotation matrix, translation vector, and pose for each 
        % frame w.r.t. the global coordinate system (GCS).
        function process_data(this, raw_data)
            for i = 1:length(raw_data)
                % Rotation matrices and translation vectors have already
                % been calculated and stored in raw data file. Extract:
                this.rotm_global(:, :, i) = reshape(raw_data(i, 37:45), [3,3]);
                % Multiply by initial rotation matrix:
                this.rotm_global(:, :, i) = this.R_1' * this.rotm_global(:, :, i);
                this.t_global(:, i) = raw_data(i, 46:48)';
                % Use these to calculate the pose of the robot center.
                if i == 1
                    pos_global(:, i) = [mean(this.marker_x_pos(1, :)); 
                                       mean(this.marker_y_pos(1,:));
                                       mean(this.marker_z_pos(1,:))];
                else
                    pos_global(:, i) = this.rotm_global(:, :, i)*pos_global(:, 1) + ...
                                 this.t_global(:, i);
                end
                % Use built-in MATLAB command to convert rotation matrix
                % to Euler angles.
                eul_angles = rotm2eul(this.rotm_global(:, :, i));
                this.poses(:, i) = [pos_global(1, i);      % x position in GCS
                                    pos_global(2, i);      % y position in GCS
                                    eul_angles(1)];      % heading / yaw
            end
            % Convert from pixels to cm. 
            this.poses(1:2, :) = this.poses(1:2, :) * this.pixel_length;
            this.t_global = this.t_global * this.pixel_length;
        end
    end
end
