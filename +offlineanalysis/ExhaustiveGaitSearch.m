% ==================== GaitSynthesizer =======================
%
%  Handle class used for synthesizing gaits from the PrimitivesTest() data.
% 
%
%  GaitSynthesizer()
%
%  The constructor accepts motion_primitive_data as an argument, which is
%  the concatenated data from the PrimitivesTest() object:
%  motion_primitive_data = cat(3, delta_x_data, delta_y_data, delta_theta_data);
%

% ==================== GaitSynthesizer =======================

classdef ExhaustiveGaitSearch < handle
    properties (Access = public)
      % Experimental Setup
      robot_name;          % label for robot (either blue or orange)  
      
      substrate;           % name for substrate (e.g., black mat)
      
      n_unique_states;     % number of unique robot states
      
      % Optimization Parameters
      alpha_motion;        % [3 x 1] hyperparameter for scaling the 
                           % reward of the robot motion term
                           % [delta_x, delta_y, delta_theta]. This can be
                           % set as a constant or varying value.
                    
      alpha_var;           % [3 x 1] hyperparameter for penalizing
                           % the variance in robot motion term
                           % [var_x, var_y, var_theta]. This can be set as
                           % a constant or varying value.
      
      alpha_len;           % [1 x 1] hyperparameter for penalizing the 
                           % length of the synthesized gait. This is a
                           % constant value.
      
      n_variations;        % number of latin hypercube sampling (LHS) 
                           % variations
                           
      
      MAX_ITER;            % maximum number of iterations to search for 
                           % simple cycle output
      
                           
      goal;                % number denoting the goal of the gait synthesis.
                           % This can be set to 1 for translation, 2 for
                           % rotation, or 3 for mixed
                           
      MAX_TRANSLATION;     % max translation per cycle allowed in rotation 
                           % gait
                           
      MAX_ROTATION;        % max rotation per cycle allowed in translation 
                           % gait
                           
      % Mixed Goal Optimization Parameters
      % These will be defined if the user wants a mixed goal instead of the
      % standard 'translation' or 'rotation' goals.
      
      can_vary_alpha;      % [7 x 1] logical vector denoting which 
                           % hyperparameters are to be fixed [0] and which
                           % can be varied in the LHS [1]. The first three
                           % correspond to alpha_motion, the second three
                           % correspond to alpha_var, and the last 
                           % corresponds to alpha_len.
                           
      % Synthesis Outputs
      avg_motions;         % [3 x n] mean translation and rotation for all
                           %  n motion primitives
      
      var_motions;         % [3 x n] variance in translation and rotation
                           % for all n motion primitives
                           
      solutions;           % structure encapsulating optimal gait information
      
    end 


    methods
        % Constructor
        function this = ExhaustiveGaitSearch( motion_primitive_data, params )
           % Parse params to set properties.
           if nargin < 2
               params = [];
           end
           
           this.set_property(params, 'robot_name', 'undefined');
           this.set_property(params, 'substrate', 'undefined');
           this.set_property(params, 'n_unique_states', 16);
           this.set_property(params, 'alpha_motion', 2*[1 1 1]');
           this.set_property(params, 'alpha_var', 0.5*[1 1 1]');
           this.set_property(params, 'alpha_len', 1);
           this.set_property(params, 'n_variations', 100);
           this.set_property(params, 'MAX_ITER', 100);
           this.set_property(params, 'goal', 1);
           this.set_property(params, 'MAX_TRANSLATION', 1);
           this.set_property(params, 'MAX_ROTATION', deg2rad(5));
           
%            if this.goal == 3
%                this.set_property(params, 'can_vary_alpha', [1 1 0 1 1 1 1])
%            end
            
           % Analyze data to find average and variance in motion.
           this.avg_motions = squeeze(mean(motion_primitive_data, 1))';
           this.var_motions = squeeze(var(motion_primitive_data, 0, 1))';
           
           % Set up optimization problem.
           
           n_states = this.n_unique_states;
           % Define number of motion primitives.
           n_primitives = length(motion_primitive_data);
        end    % constructor
       




        
    end % methods
end % classdef