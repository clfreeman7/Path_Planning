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

classdef GaitPredict
    properties (Access = public)
      % Subclass-specific properties
      n_unique_states = 16;     % number of unique robot states
      
      robo_states;         % sequence of robot states e.g., [1, 7, 16, ...] 
                           % that defines a gait

      primitive_labels;    % sequence of robot motion primitive labels, 
                           % e.g., [161, 23, 1, ...] 
                           
      
      len_gait;            % length of the gait (number of robot states)
     
      
      % Change in robot pose information w.r.t. local frame of tail state
      delta_x;         % change in x position (cm)
    
      delta_y;         % change in y position (cm)
      
      delta_theta;     % change in rotation (CCW is positive)
      
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
          
          % Define the sequence of motion primitive labels.
          this.states_to_primitives;
          
          this.delta_x = squeeze(motion_primitive_data(:,gait_sequence,1));
          this.delta_y = squeeze(motion_primitive_data(:,gait_sequence,2));
          this.delta_theta = squeeze(motion_primitive_data(:,gait_sequence,3));

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


    end % methods
end %classdef
