% ==================== PathTest =======================
%
%  Subclass of the ExperimentalData class specific to experiments
%  testing the gaits output from the gait synthesizer. 
% 
%
%  PathTest()
%
%  The constructor accepts raw_data as an argument, which is the raw output
%  from visual tracking, defined as an array tracking_data structured as
%  follows, whose size depends on the number of n markers: 
%  Columns                       Respective Data
%  1  to 3  (1x3)                Marker 1 [x,y,z]
%  4  to 6  (1x3)                Marker 2 [x,y,z]
%            . . .
%  3*(n-1)+1 to 3*n (1x3)        Marker n [x,y,z]
%
%  3*n+1 to 3*n+9 (1x9)          Rotation matrix Intermediate frames 
%                                  [Reshaped from 3x3 to 1x9]
%  3*n+10 to 3*n+12 (1x3)        Translation Matrix Intermediate frames 
%                                  [[x,y,z]Reshaped from 3x1 to 1x3] 
%  3*n+13 to 3*n+21 (1x9)        Rotation matrix Global frame 
%                                  [Reshaped from 3x3 to 1x9]
%  3*n+22 to 3*n+24 (1x3)        Translation Matrix Global frame 
%                                  [[x,y,z]Reshaped from 3x1 to 1x3]
%  3*n+ + 2*9 + 2*3 + 1 (1x1)    Timestamps (s)
%
% ==================== PathTest =======================

classdef PathTest < handle
    properties (Access = public)
      % Subclass-specific properties
      gait_names;         % should be cell array (or array) of chars representing 
                          %     sequence of gaits comprising trajectory (e.g. 'A', 'B', 'A', 'B')


      gait_durations;   % corresponding gait durations (time or number of gait periods)
      gt_params;
      robo_states;
      gait_tests;
      keyframes;
      switch_frames;
      path_name;
      poses;
      pause_time;
      % Change in robot pose information w.r.t. local frame of tail state
      delta_x;         % change in x position (cm)
    
      delta_y;         % change in y position (cm)
      
      delta_theta;     % change in rotation (CCW is positive)
      
    end 
    
    methods
        % Constructor
        function this = PathTest( raw_data, path_sequence, gait_library,  params)
          if nargin < 4
              params = [];
          end
          % Use super class constructor for basic data analysis
          %this@offlineanalysis.ExperimentalData( raw_data, params ); 
          this.set_property(params, 'pause_time', 0);

          this.gait_names = path_sequence.gait_names;
          this.gait_durations = path_sequence.gait_durations;
          if length(this.pause_time) == 1 % if the pause-time is constant
              this.pause_time = repmat(this.pause_time,1,length(this.gait_names)-1);
          end
          gait_library_names = '';
          for i = 1:length(gait_library)
              gait_library_names(i) = gait_library(i).gait_name;
          end
          this.gt_params = params;
          this.gt_params.frame_1 = params.frame_1(1);
          this.keyframes = [];
          this.switch_frames = [];
          % Extract parameters to use the GaitTest class.
          for i = 1:length(this.gait_names)*2-1
              if i>1
                  this.gt_params.frame_1 = this.gait_tests{i-1}.keyframes(end)+1;
              end
              if mod(i,2) == 0 % if the index is even
                  this.gt_params.n_cycles = 1;     % transition between gaits
                  this.robo_states{i} = [(this.robo_states{i-1}(end)) 1 (this.robo_states{i}(1))];
              else
                  this.gt_params.n_cycles = this.gait_durations((i+1)/2);
                  this.robo_states{i} = gait_library(gait_library_names == this.gait_names((i+1)/2)).robo_states;
                  if length(params.frame_1) > 1 
                      if params.frame_1((i+1)/2) ~= 0
                        this.gt_params.frame_1 = params.frame_1((i+1)/2);
                      end
                  end
              end

              this.switch_frames(i) = this.gt_params.frame_1-1;
              this.gait_tests{i} = offlineanalysis.GaitTest( raw_data, this.robo_states{i}, this.gt_params);
              this.delta_x{i} = this.gait_tests{i}.delta_x;
              this.delta_y{i} = this.gait_tests{i}.delta_y;
              this.delta_theta{i} = this.gait_tests{i}.delta_theta;
              this.keyframes = [this.keyframes; this.gait_tests{i}.keyframes];
          end
         this.keyframes = unique(this.keyframes);
          all_data = offlineanalysis.ExperimentalData(raw_data, params);
          this.poses = all_data.poses;

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
        
        

        function plot(this)
            
            % Extract the global poses for every keyframe.
            key_poses = this.poses(:, this.keyframes);

            Poses = this.poses(:, this.switch_frames);

            w = ((max(Poses(1,:))-min(Poses(1,:))).^2+(max(Poses(2,:))-min(Poses(2,:))).^2).^.5/10;
            hold on
            plot(this.poses(1,:), this.poses(2,:))
            xlabel('x (cm)')
            ylabel('y (cm)')
           title("Path")
            hold on
            sz = 30;
            c1 = linspace(1,10,length(this.switch_frames));
            c2 = [0.6350 0.0780 0.1840];

            scatter(key_poses(1,:), key_poses(2,:), sz, c1, 'filled');
             %scatter(pose_check(1, :), pose_check(2, :),sz, c2)
             scatter(key_poses(1, :), key_poses(2, :),sz, c2)
             quiver(Poses(1,:), Poses(2,:), w*cos(Poses(3,:)), w*sin(Poses(3,:)),0)
            %legend('Continuous Robot Position', 'Actual Keyframe Positions', ...
            %       'Keyframe Positions Reconstructed from Motion Primitives')
             daspect([1 1 1]);
             grid on;
        end
        
    end  %methods

end  %class