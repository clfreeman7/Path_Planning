% ==================== GaitPredict =======================
%
%  Handle class used to predict gait motion based on motion primitive data.
% 
%
%  GaitTest()
%
%  The constructor accepts motion_primitive_data as an argument, which is
%  the concatenated data from the PrimitivesTest() object:
%  motion_primitive_data = cat(3, delta_x_data, delta_y_data, delta_theta_data);

% ==================== GaitPredict =======================

classdef GaitPredict<handle
    properties (Access = public)
      % Subclass-specific properties
      n_unique_states;     % number of unique robot states
      
      robo_states;         % sequence of robot states e.g., [1, 7, 16, ...] 
                           % that defines a gait

      primitive_labels;    % sequence of robot motion primitive labels, 
                           % e.g., [161, 23, 1, ...] 
                           
      
      len_gait;            % length of the gait (number of robot states)
     
      
      % Change in robot pose information w.r.t. local frame of tail state
      delta_x;         % change in x position (cm)
    
      delta_y;         % change in y position (cm)
      
      delta_theta;     % change in rotation (CCW is positive)
      
      var_x;

      var_y;

      var_theta;
    end 
    methods 
         % Constructor
         function this = GaitPredict( gait_sequence,  motion_primitive_data, params )
          if nargin < 3
              params = [];
          end
          this.set_property(params, 'n_unique_states', 16);
          
          % Define sequence of robot states.
          this.robo_states = gait_sequence;
          
          this.len_gait = length(gait_sequence);
          
          % Analyze data to find average and variance in motion.
          avg_motions = squeeze(mean(motion_primitive_data, 1));
          var_motions = squeeze(var(motion_primitive_data, 0, 1));

          % Define the sequence of motion primitive labels.
          this.primitive_labels = this.states_to_primitives;

          this.delta_x = avg_motions(this.primitive_labels,1)';
          this.delta_y = avg_motions(this.primitive_labels,2)';
          this.delta_theta = avg_motions(this.primitive_labels,3)';

          this.var_x = var_motions(this.primitive_labels,1)';
          this.var_y = var_motions(this.primitive_labels,2)';
          this.var_theta = var_motions(this.primitive_labels,3)';
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
        function primitive_labels = states_to_primitives(this)
            states = [this.robo_states this.robo_states(1)];
            n_states = this.n_unique_states;
            for i = 1:length(states)-1
                primitive_labels(i) = inverse_map(states(i), states(i+1), n_states);
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

        function pose = plot(this, n_cycles)
            
            R =  eye(3);
            pos = [0; 0; 0];
            k=0;
            pose = zeros(3, n_cycles*this.len_gait+1);
            for i = 1:n_cycles
                for j = 1:this.len_gait
                    k = k+1;
                    delta_x_local = this.delta_x(j);
                    delta_y_local = this.delta_y(j);
                    R_local = eul2rotm([this.delta_theta(j) 0 0]);
                    % Use body-frame transformation formula:
                    pos = R*[delta_x_local; delta_y_local; 0] + pos;
                    % Post-multiply for intrinsic rotations.
                    R = R*R_local;
                    % Store global position data for verification.
                    pose(:, k+1) = pos;
                    pose(3,k+1) = pose(3,k) + this.delta_theta(j);
                end
            end
            Poses = pose(:, 1:this.len_gait:end);
            w = ((max(pose(1,:))-min(pose(1,:))).^2+(max(pose(2,:))-min(pose(2,:))).^2).^.5/10;
            hold on
            plot(pose(1,:), pose(2,:))
            xlabel('x (cm)')
            ylabel('y (cm)')
            title("Predicted Gait [" + num2str(this.robo_states) + "]")
            hold on
            sz = 30;
            c1 = linspace(1,n_cycles,length(pose));
            c2 = [0.6350 0.0780 0.1840];

            scatter(pose(1,:), pose(2,:), sz, c1, 'filled');
             %scatter(pose_check(1, :), pose_check(2, :),sz, c2)
             scatter(Poses(1, :), Poses(2, :),sz, c2)
             quiver(Poses(1,:), Poses(2,:), w*cos(Poses(3,:)), w*sin(Poses(3,:)),0)
            %legend('Continuous Robot Position', 'Actual Keyframe Positions', ...
            %       'Keyframe Positions Reconstructed from Motion Primitives')
             daspect([1 1 1]);
             grid on;
        end
        

    end % methods
end %classdef
