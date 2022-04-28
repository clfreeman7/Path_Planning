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

classdef GaitSynthesizer < handle
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
        function this = GaitSynthesizer( motion_primitive_data, params )
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
           
           % Build incidence matrices for simple cycle constraint.
           B = zeros(n_states, n_primitives); % vertex-arc incidence matrix
           B_tail = B; % tail incidence matric
           B_head = B; % head incidence matric
           for iEdges = 1:n_primitives
             [tail_state,head_state] = forwardmap(iEdges, this.n_unique_states);
             B_tail(tail_state, iEdges) = 1;
             B_head(head_state, iEdges)= 1;
           end
           B = B_head-B_tail;
           
           % constant for equality constraint to ensure indegree = outdegree
           beq = zeros(n_states , 1); % Bx = beq
           % constant and matrix for inequality constraint to ensure no repeated vertices
           b = ones(n_states , 1); % A x <= b
           A = B_head; 
           % add inequality constraint for length of cycle >= 2
           b = [b; -2];
           A = [A; -ones(1, n_primitives)]; 
           if this.goal == 1
              % add inequality constraint to ensure no rotation 
              A = [A; this.avg_motions(3,:); -this.avg_motions(3,:)];
              b = [b; this.MAX_ROTATION*ones(2, 1)];
              n_parameters = 2;
              alpha_motion_nom = mean(this.alpha_motion);
           elseif this.goal == 2
              % add inequality constraint to ensure no translation 
              A = [A; this.avg_motions(1:2,:); -this.avg_motions(1:2,:)];
              b = [b; this.MAX_TRANSLATION*ones(4, 1)];
              n_parameters = 1;
              alpha_motion_nom = mean(this.alpha_motion);
           end
          
           % define bounds to convert integer programming to binary programming 
           lb = zeros(n_primitives, 1);   % lower bound
           ub = ones(n_primitives, 1);   % upper bound
           intcon = 1:n_primitives;   % define all variables as integer (binary)      
          
           % define weights for cost function via LHS
           % rng default % For reproducibility
           cost_params = lhsdesign(this.n_variations, n_parameters);
           alpha_motion = zeros(3,1);
           alpha_var = zeros(3, 1);
           n_sols = 0;      % number of solutions (i.e., optimal gaits)
           solutions  = [];   % cell matrix of all solutions in the form of char vectors
           for iVariations = 1:this.n_variations
              % Vary the motion hyperparamter via LHS.
              if this.goal == 1
                alpha_motion(1:2) = (2*cost_params(iVariations,:)'-1)*alpha_motion_nom;
              elseif this.goal == 2
                alpha_motion(3) = (2*cost_params(iVariations,:)'-1)*alpha_motion_nom; 
              end
              
              % Define other hyperparameters. 
              alpha_var = this.alpha_var;
              alpha_len = this.alpha_len;
              
              % Define cost function.
              for i = 1:n_primitives
                  f(i) = alpha_motion' * this.avg_motions(:,i) + ...
                         alpha_var' * this.var_motions(:,i) + alpha_len;
              end
              
              
              n_iterations = 0;         % iteration counter
              isSingleCycle = false;    % boolean flag
              hasSolution = true;       % boolean flag
              intial_guess = [18, 47];
              x0 = zeros(n_primitives,1);
              x0([intial_guess]) = 1;
              options = optimoptions('intlinprog');
              options.Display = 'off';
              
              while n_iterations < this.MAX_ITER && isSingleCycle == false
                  [x1,fval1,exitflag1] = intlinprog(f,intcon,A,b,B,beq,lb,ub,x0,options);
                  cycle = find(x1>0.95);
                  hist{n_iterations + 1} = cycle;
                  if  length(cycle) == 1     % singleton solution
                    isSingleCycle = false;
                    newA = -ones(1, cycle_len);
                    newA(cycle) = 1;    
                    A = [A; newA];               % add cutting plane
                    b = [b; cycle_len-1];      
                  elseif isempty(cycle)      % no solution
                    hasSolution = false;
                    n_iterations = MAX_ITER;
                  else
                    cycle_len = length(cycle);
                    for i = 1:cycle_len
                        [tail(i), head(i)] = forwardmap(cycle(i), n_states);
                    end
                    count = 1;
                    % run DFS to check if resulting gait graph is connected (only one cycle
                    % present)
                    isComplete = false;  % DFS has not been completed yet
                    head_list = [];
                    tail_list = [];
                  
                  while ~isComplete 
                    if count ==1
                      edge_1 = tail(1);
                      j = head(1);
                      idx = 1;
                    else
                      j = head(idx);
                    end
                
                    % add to the history record
                    tail_list = [tail_list head(idx)];
                    head_list = [head_list tail(idx)];
                    % delete current edge
                    head(idx) = [];
                    tail(idx)= [];
                    % find new edge
                    idx = find(tail == j);
                    if length(idx)>1
                        idx(2:end) = [];
                    end
                    if isempty(idx)
                        if isempty(head) && j == edge_1
                            isSingleCycle = true;
                            isComplete = true;
                        else
                            isSingleCycle = false;
                            newA = -ones(1, cycle_len);
                            newA(cycle) = 1;
                            A = [A; newA];
                            b = [b; cycle_len-1];
                            isComplete = true;
                        end
                    end
                    count = count+1;
                end
            end
            n_iterations = n_iterations+1;

                  
              end    % while n_iterations++
              
              isNewCycle = false;
              % Check whether this new solution is unique or has already
              % been found.
              if ~isempty(cycle)
                  solutions{n_sols + 1} = num2str(cycle');
                  if isempty(solutions)
                      isNewCycle = true;
                  else
                    num_unique_sols = length(unique(solutions));
                    if num_unique_sols == n_sols + 1 
                      isNewCycle = true;
                    else
                       solutions{n_sols + 1} = [];     % delete repeated solution
                    end
                      
                  end
                  if isNewCycle      % if new solution is unique, store it
                    n_sols = n_sols +  1;
                    %this.solutions(n_sols).label = char('A' + n_sols -1);
                    this.solutions(n_sols).primitive_labels = cycle;
                    this.solutions(n_sols).distance = this.calc_gait_dist(cycle);
                    this.solutions(n_sols).total_rot_deg = rad2deg(sum(this.avg_motions(3, cycle)));
                    this.solutions(n_sols).lin_speed = this.solutions(n_sols).distance / length(cycle);
                    this.solutions(n_sols).rot_speed = this.solutions(n_sols).total_rot_deg / length(cycle);
                    solutions{n_sols} = num2str(cycle');
                    this.solutions(n_sols).tail_states = tail_list;
                    this.solutions(n_sols).head_states = head_list;
                  end
              end
                  
              
           end     %  for each LHS variation

        if isempty(this.solutions)
            error('No  gaits found. Try adjusting hyperparameters.')
        end
        
       
        end    % constructor
            
        % Set parameter value for class-instance, based on user specified 
        % values (or default if property doesn't exist as struct field)
        function set_property(this, source_struct, param_name, def_val)
          if ( isfield(source_struct, param_name) )
            this.(param_name) = source_struct.(param_name);
          else
            this.(param_name) = def_val;
          end
        end
        
        % Calculate the total distance travelled over one gait cycle.
        function gait_dist = calc_gait_dist( this, gait_primitive_labels )
            primitive_labels = gait_primitive_labels;
            pos = [0 0 0]';  % initial position
            R = eye(3);      % initial orientation
            for i = 1:length(primitive_labels)
                k = primitive_labels(i);
                trans_local = this.avg_motions(1:2, k);
                R_local = eul2rotm([this.avg_motions(3, k) 0 0]);
                % Use body-frame transformation formula:
                pos = R*[trans_local; 0] + pos;
                % Post-multiply for intrinsic rotations.
                R = R*R_local;
            end
            gait_dist = norm(pos(1), pos(2));     % total translation mag
        end
        
        
       
    end % methods
end % classdef