% ============================= Gait =================================
%
%  Handle class used to instantiate gait objects to form a gait library /
%  database using one of three inputs:
%  
%       (1) a GaitTest() object which analyzes the data from visual
%           tracking,
%
%  (future)     (2) an OldGaitTest() object which analyses the data from manual
%           visual tracking , or
%
%  (future)    (3) a GaitPredict() object which uses motion primitive data to
%           predict the behavior of gaits before testing them
%
%  Gait()
%

% ============================ Gait ==============================

classdef Gait < handle
    properties (Access = public)
      % Robot / envrionment  information
      robot_name;          % label for robot (either blue or orange)  

      transition_time;     % time constant for motion primitives in seconds
      
      substrate;           % surface that the robot moves on

      tether;              % tether  

      tether_protocol;
      
      n_unique_states;     % number of unique robot states
      
      
      % Gait information
      gait_name;           % character (i.e., A, B, etc) name for the gait
      
      robo_states;         % gait sequence
                           
      primitive_labels;    % sequence of robot motion primitive labels, 
                           % e.g., [161, 23, 1, ...] for = {1,2, ..., 240}
                           

      len_gait;            % [n] length of the gait (number of robot states)
      
      category;            % category of data: scalar
                           % GaitTest - tested gait with automatic tracking
                           % OldGaitTest - tested gait with manual tracking
                           % GaitPredict - predicted gait

      n_experiments;       % number of GaitTest() objects being analyzed if 
                           % category 1

      % Change in robot pose information w.r.t. local frame of tail state
      % for each motion primitive where m (number of motion primitives) = 
      % n x n_cycles
      
      delta_poses;         % [3 x m] vector denoting average change in 
                           % [x y theta]' for theta in radians
                           
      var_delta_poses;     % variance 
      
      twists;              % [3 x m] body twists in SE(2)
      
      % Change in robot pose information w.r.t. local frame of tail state
      % for each Gait. 
      
      Delta_Pose;          % [3 x 1] vector denoting average change in 
                           % [x y theta]' for theta in radians
      
      Var_Delta_Pose; 
      
      Twist;
    end 
    
    methods 
        % Constructor
        function this = Gait( gait_data_object,  params)
            if nargin < 2
                params = [];
            end
            this.set_property(params, 'gait_name', 'undefined');
            this.set_property(params, 'robot_name', 'undefined');
            this.set_property(params, 'transition_time', .45);  % s
            this.set_property(params, 'substrate', 'mat');  % s
            this.set_property(params, 'n_unique_states', 16);

            this.robo_states = gait_data_object(1).robo_states;
            this.primitive_labels = gait_data_object(1).primitive_labels;
            this.len_gait = gait_data_object(1).len_gait;

            if isa(gait_data_object, 'offlineanalysis.GaitTest')
                this.category = 'GaitTest';
            end
            if isa(gait_data_object, 'offlineanalysis.GaitPredict') 
                this.category = 'GaitPredict';
            end
            % Find average change in robot pose for each motion primitive.
            
            this.n_experiments = length(gait_data_object);
            delta_x = [];
            delta_y = [];
            delta_theta = [];

            for i = 1: this.n_experiments
                delta_x = [delta_x; gait_data_object(i).delta_x];
                delta_y = [delta_y; gait_data_object(i).delta_y];
                delta_theta = [delta_theta; gait_data_object(i).delta_theta];
            end
            this.delta_poses(1,:) = mean(delta_x);
            this.delta_poses(2,:) = mean(delta_y);
            this.delta_poses(3,:) = mean(delta_theta);

            this.var_delta_poses(1,:) = var(delta_x);
            this.var_delta_poses(2,:) = var(delta_y);
            this.var_delta_poses(3,:) = var(delta_theta);
            this.calculate_total_motion;

            % Convert change in robot pose to body twists.
            for i = 1:this.len_gait
                this.twists(:, i) = this.delta_pose_2_twist(this.delta_poses(:, i), this.transition_time);
            end
            this.Twist = this.delta_pose_2_twist(this.Delta_Pose, this.transition_time*(this.len_gait));

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
        
        % Use the delta_pose of each motion to calculate the total average
        % movement (Delta_Pose) over an entire gait cycle. 
        function calculate_total_motion(this)
            pos = zeros(3,1);      % [x y theta]' at start
            pose = pos;
            R = eye(3);
            for i = 1:this.len_gait
                delta_x_local = this.delta_poses(1, i);
                delta_y_local = this.delta_poses(2, i);
                R_local = eul2rotm([this.delta_poses(3, i) 0 0]);
                % Use body-frame transformation formula:
                pos = R*[delta_x_local; delta_y_local; 0] + pos;
                % Post-multiply for intrinsic rotations.
                R = R*R_local;
                % Store global position data for verification.
                pose(:, i+1) = pos;
            end
            this.Delta_Pose = [pose(1:2, end); sum(this.delta_poses(3,:))];
        end
        



        function pose = plot(this, n_cycles)
            if nargin<2
                n_cycles = 1;
            end 
            % Reconstruct the keyframe positions from motion primitives.
            pose(:, 1) = zeros(3,1);
            pos = pose;
            R =  eye(3);
            k=0;
            % Find the poses using the delta_poses for each motion
            % primitive. 
            for i = 1:n_cycles
                for j = 1:this.len_gait
                    k = k+1;
                    delta_x_local = this.delta_poses(1, j);
                    delta_y_local = this.delta_poses(2, j);
                    R_local = eul2rotm([this.delta_poses(3, j) 0 0]);
                    % Use body-frame transformation formula:
                    pos = R*[delta_x_local; delta_y_local; 0] + pos;
                    % Post-multiply for intrinsic rotations.
                    R = R*R_local;
                    % Store global position data for verification.
                    pose(1:2, k+1) = pos(1:2);
                    pose(3,k+1) = pose(3,k) + this.delta_poses(3,j);
                end
            end
            
            % Check the math in the methods by using the Delta_Pose of
            % the entire gait.
            pos = zeros(3,1);
            R = eye(3);
            for i = 1:n_cycles
                    delta_x_local = this.Delta_Pose(1);
                    delta_y_local = this.Delta_Pose(2);
                    R_local = eul2rotm([this.Delta_Pose(3) 0 0]);
                    % Use body-frame transformation formula:
                    pos = R*[delta_x_local; delta_y_local; 0] + pos;
                    % Post-multiply for intrinsic rotations.
                    R = R*R_local;
                    % Store global position data for verification.
                    Pose(:, i+1) = pos;
                    Pose(3,i+1) = Pose(3,i) + this.Delta_Pose(3);
            end

            % Check the math of the methods by using the Twist of
            % the entire gait. 
            w = ((max(Pose(1,:))-min(Pose(1,:))).^2+(max(Pose(2,:))-min(Pose(2,:))).^2).^.5/10;
            hold on
            plot(pose(1,:), pose(2,:))
            xlabel('x (cm)')
            ylabel('y (cm)')
%             if all(this.gait_name=='undefined')
%                 title("Average Gait [" + num2str(this.robo_states) + "]")
%             else
%                 title("Average Gait " + this.gait_name + ": [" +num2str(this.robo_states) + "]")
%             end
            grid on
            hold on
            sz = 30;
            c1 = linspace(1,n_cycles,this.len_gait*n_cycles+1);
            c2 = [0.6350 0.0780 0.1840];

            scatter(pose(1,:), pose(2,:), sz, c1, 'filled')
            scatter(Pose(1, :), Pose(2, :),sz, c2)
            daspect([1 1 1]);
           
            quiver(Pose(1,:), Pose(2,:), w*cos(Pose(3,:)), w*sin(Pose(3,:)),0)
        end
        
    end  %methods
    
    methods (Static)
        % Calculate the average body twist for each motion primitive or
        % gait.
        function twist = delta_pose_2_twist(delta_pose, delta_time)
             delta_pose_mat = [ cos(delta_pose(3)), -sin(delta_pose(3)), delta_pose(1) ; ...
                                sin(delta_pose(3)), cos(delta_pose(3)), delta_pose(2) ; ...
                                0, 0, 1 ];
             twist_mat = logm(delta_pose_mat)/delta_time;
             twist = [ twist_mat(1, 3), twist_mat(2, 3), twist_mat(2, 1) ];
%             twist = 1/delta_time*delta_pose;
         end
    end

end  %class
