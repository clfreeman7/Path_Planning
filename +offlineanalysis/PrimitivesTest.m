% ==================== PrimitivesTest =======================
%
%  Subclass of the ExperimentalData class specific to experiments
%  exhuastively exploring motion primitives using Euler tours.
% 
%
%  PrimitivesTest()
%
%  The constructor accepts raw_data as an argument, which is the raw output
%  from visual tracking, defined as an array all_pts structured as
%  follows: 
%  Columns             Respective Data
%  1  to 3  (1x3)      Marker 1 [x,y,z]
%  4  to 6  (1x3)      Marker 2 [x,y,z]
%  7  to 9  (1x3)      Marker 3 [x,y,z]
%  10 to 12 (1x3)      Marker 4 [x,y,z]
%  13 to 15 (1x3)      Marker 5 [x,y,z]
%  16 to 18 (1x3)      Marker 6 [x,y,z]
%  19 to 21 (1x3)      Marker 7 [x,y,z]
%  22 to 24 (1x3)      Marker 8 [x,y,z]
%  25 to 33 (1x9)      Rotation matrix Intermediate frames 
%                         [Reshaped from 3x3 to 1x9]
%  34 to 36 (1x3)      Translation Matrix Intermediate frames 
%                         [[x,y,z]Reshaped from 3x1 to 1x3] 
%  37 to 45 (1x9)      Rotation matrix Global frame 
%                         [Reshaped from 3x3 to 1x9]
%  46 to 48 (1x3)      Translation Matrix Global frame 
%                         [[x,y,z]Reshaped from 3x1 to 1x3]

% ==================== PrimitivesTest =======================
classdef PrimitivesTest < offlineanalysis.ExperimentalData 
    properties (Access = public)
      % Subclass-specific properties
      n_unique_states;     % number of unique robot states
      
      robo_states;         % sequence of robot states e.g., [1, 7, 16, ...]
                           % This will be a [1 x 241] vector corresponding to
                           % randomized Euler tour.
                           
      primitive_labels;    % sequence of robot motion primitive labels, 
                           % e.g., [161, 23, 1, ...] for = {1,2, ..., 240}
                       
      keyframes;
      
      % Change in robot pose information w.r.t. local frame of tail state
      delta_x;         % change in x position (cm)
    
      delta_y;         % change in y position (cm)
      
      delta_theta;     % change in rotation (CCW is positive)
      
    end 
    
    methods
        % Constructor
        function this = PrimitivesTest( raw_data, params, Euler_tour)
            
          % Use super class constructor for basic data analysis
          this@offlineanalysis.ExperimentalData( raw_data, params ); 
          this.set_property(params, 'n_unique_states', 16);
          this.robo_states = Euler_tour;
          
          % Define the sequence of motion primitive labels.
          this.states_to_primitives;
          
          % Find the local rotation matrices for each motion primitive. 
          this.extract_keyframes;
          this.calculate_local_motions;
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
        
        % Convert the 241 roobot state sequence into a sequence of motion 
        % primitive labels.
        function states_to_primitives(this)
            states = this.robo_states;
            n_states = this.n_unique_states;
            for i = 1:length(states)-1
                this.primitive_labels(i) = inverse_map(states(i), states(i+1), n_states);
            end
            
            function edgeNum = inverse_map(from,to,n_states)
            % n_s = number of states (in our case, 16)
             for m = 2:n_states-1
               h(m-1) = H(f(from,to,n_states)-f(m,m,n_states));
             end
             
             edgeNum = f(from,to,n_states)-sum(h);
           
             % Nested helper function 1
             function F = f(i,j,n_states)
                F = n_states*(i-1)+(j-1);
             end
             
             % Nested helper function 2
             function h = H(x)     % discrete Heaviside step function
               if x <0
                   h = 0;
               else
                   h = 1;
               end
             end
             
             end
        end
        
        % Extract keyframes.
        function extract_keyframes(this)
            frames_per_trans = this.framerate * this.transition_time; 
            if (frames_per_trans - floor(frames_per_trans)) < 0.05
                % If # frames per transition is close to an integer: 
                frames_per_trans = floor(frames_per_trans);    % make integer
                unorderedPoses(1:2, :) = p(initialFrame - 1:frames_per_trans:end);
            else 
                this.keyframes = zeros(242, 1);
                this.keyframes(1) = this.frame_1 - 1;
                for k = 2:242
                    if mod(k, 2) == 0 % if even index
                        this.keyframes(k) = this.keyframes(k-1) + ceil(frames_per_trans);
                    else
                        this.keyframes(k) = this.keyframes(k-1) + floor(frames_per_trans);
                    end
                end
            end
        end
        
        
        function calculate_local_motions(this)
            % Extract the global poses for every keyframe.
            key_poses = this.poses(:, this.keyframes);
            
            % Define local changes in these poses w.r.t. to each tail pose. 
            unordered_deltas = zeros(3, 240);
            
            for k = 1:240     % total number of transitions in Euler tour.
                % Calculate change in robot heading.
                unordered_deltas(3, k) = key_poses(3, k+2) - key_poses(3, k+1);
                % Calculate change in robot position.
                delta_X_global = key_poses(1:2, k+2) - key_poses(1:2, k+1);
                % Orientation of initial pose w.r.t. global frame.
                R_Global = this.rotm_global(:, :, this.keyframes(k+1));
                delta_X_local = R_Global' * [delta_X_global; 0];
                unordered_deltas(1:2, k) = delta_X_local(1:2);
            end
            [~, orderIndex] = sort(this.primitive_labels);
            % Sort position and heading data. 
            ordered_deltas = unordered_deltas(:, orderIndex);
            this.delta_x = ordered_deltas(1,:);
            this.delta_y = ordered_deltas(2,:);
            this.delta_theta = ordered_deltas(3,:);
            % normalize to between -pi and pi
            this.delta_theta(this.delta_theta > pi)  = this.delta_theta(this.delta_theta > pi) - 2*pi;
            this.delta_theta(this.delta_theta < -pi)  = this.delta_theta(this.delta_theta < -pi) +2*pi;
        end

        function plot(this)
            % Extract the global poses for every keyframe.
            key_poses = this.poses(:, this.keyframes);
            % Reconstruct the keyframe positions from motion primitives.
            pose_check(:, 1) = this.poses(:, this.keyframes(2));
            R =  eul2rotm([pose_check(3,1) 0 0]);
            pos = [pose_check(1:2, 1); 0];
            for i = 1:240
                k = this.primitive_labels(i);
                delta_x_local = this.delta_x(k);
                delta_y_local = this.delta_y(k);
                R_local = eul2rotm([this.delta_theta(k) 0 0]);
                % Use body-frame transformation formula:
                pos = R*[delta_x_local; delta_y_local; 0] + pos;
                % Post-multiply for intrinsic rotations.
                R = R*R_local;
                % Store global position data for verification.
                pose_check(:, i+1) = pos;
            end
            

            hold on
            plot(this.poses(1,:), this.poses(2,:))
            xlabel('x (cm)')
            ylabel('y (cm)')
            title('Verification of Motion Primitive Calculations')
            hold on
            sz = 30;
            c1 = linspace(1,10,length(this.keyframes));
            c2 = [0.6350 0.0780 0.1840];
            scatter(key_poses(1,:), key_poses(2,:), sz, c1, 'filled')
            scatter(pose_check(1, :), pose_check(2, :),sz, c2)
            legend('Continuous Robot Position', 'Actual Keyframe Positions', ...
                   'Keyframe Positions Reconstructed from Motion Primitives')
        end
    end  %methods

end  %class
