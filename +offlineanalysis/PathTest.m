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

classdef PathTest < offlineanalysis.ExperimentalData 
    properties (Access = public)
      % Subclass-specific properties
      gait_names;         % should be cell array (or array) of chars representing 
                          %     sequence of gaits comprising trajectory (e.g. 'A', 'B', 'A', 'B')


      gait_durations;   % corresponding gait durations (time or number of gait periods)
      gt_params;
      robo_states;
      gait_tests;
      keyframes;
      % Change in robot pose information w.r.t. local frame of tail state
      delta_x;         % change in x position (cm)
    
      delta_y;         % change in y position (cm)
      
      delta_theta;     % change in rotation (CCW is positive)
      
    end 
    
    methods
        % Constructor
        function this = PathTest( raw_data, path_sequence, gait_library,  params)
          if nargin < 3
              params = [];
          end
          % Use super class constructor for basic data analysis
          this@offlineanalysis.ExperimentalData( raw_data, params ); 

          this.gait_names = path_sequence.gait_names;
          this.gait_durations = path_sequence.gait_durations;
          this.keyframes = [];
          % Extract parameters to use the GaitTest class.
          for i = 1:length(this.gait_names)*2-1
              if mod(i,2) == 0 % if the index is even
                  this.gt_params(i).n_cycles = 1;     % transition between gaits
                  this.robo_states{i} = [(this.robo_states{i-1}(end)) 1];
              else
                  this.gt_params(i).n_cycles = this.gait_durations(i);
                  this.robo_states{i} = gait_library(gait_library.gait_names == this.gait_names(i)).robo_states;
              end
              if i ==1
                  this.gt_params(i).frame_1 = params.frame_1;
              else
                  this.gt_params(i).frame_1 = this.gait_tests{i-1}.keyframes(end)+1;
              end
              this.gait_tests{i} = offlineanalysis.GaitTest( raw_data, this.robo_states{i}, this.gt_params(i));
              this.delta_x{i} = this.gait_tests{i}.delta_x;
              this.delta_y{i} = this.gait_tests{i}.delta_y;
              this.delta_theta{i} = this.gait_tests{i}.delta_theta;
              this.keyframes = [this.keyframes this.gait_tests{i}.keyframes];
          end
         this.keyframes = unique(this.keyframes);
          
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
            % Reconstruct the keyframe positions from motion primitives.
            pose_check(:, 1) = this.poses(:, this.keyframes(2));
            %R = [cos(pose_check(3, 1)) -sin(pose_check(3, 1));
             %   sin(pose_check(3, 1)) cos(pose_check(3, 1))];
            R =  eul2rotm([pose_check(3,1) 0 0]);
            pos = [pose_check(1:2, 1); 0];
            k=0;
            for i = 1:this.n_cycles
                for j = 1:this.len_gait
                    k = k+1;
                    delta_x_local = this.delta_x(i, j);
                    delta_y_local = this.delta_y(i, j);
                    R_local = eul2rotm([this.delta_theta(i,j) 0 0]);
                    % Use body-frame transformation formula:
                    pos = R*[delta_x_local; delta_y_local; 0] + pos;
                    % Post-multiply for intrinsic rotations.
                    R = R*R_local;
                    % Store global position data for verification.
                    pose_check(:, k+1) = pos;
                end
            end
            Poses = key_poses(:,1:this.len_gait:end);

            w = ((max(Poses(1,:))-min(Poses(1,:))).^2+(max(Poses(2,:))-min(Poses(2,:))).^2).^.5/10;
            hold on
            plot(this.poses(1,:), this.poses(2,:))
            xlabel('x (cm)')
            ylabel('y (cm)')
           title("Gait [" + num2str(this.robo_states) + "]")
            hold on
            sz = 30;
            c1 = linspace(1,this.n_cycles,length(this.keyframes));
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