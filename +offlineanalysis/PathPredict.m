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

% ==================== PathPredict =======================

classdef PathPredict<handle
    properties (Access = public)
      % Subclass-specific properties
      gait_names;         % should be cell array (or array) of chars representing 
                          %     sequence of gaits comprising trajectory (e.g. 'A', 'B', 'A', 'B')


      gait_durations;   % corresponding gait durations (time or number of gait periods)
      robo_states;

      gait_idx;

      gait_library;

      path_name;

      switch_frames;
      
      include_transitions;

      R_1;
      pose_1;
      % Change in robot pose information w.r.t. local frame of tail state
      poses;
      Poses;

    end 
    methods 
         % Constructor
         function this = PathPredict( path_sequence, gait_library, params )
          if nargin < 3
              params = [];
          end
          this.gait_library = gait_library;
          this.set_property(params, 'path_name', 'undefined');
          this.set_property(params, 'R_1', eye(3));
          this.set_property(params, 'pose_1', [0 0 0]');
            this.set_property(params, 'include_transitions', false);
          %angles_1 = rotm2eul(this.R_1);
          %if abs(angles_1(1)) > 1e-6
           % this.pose_1(3) = angles_1(1);
         % end
          if isstring(path_sequence.gait_names)
              this.gait_names = char(path_sequence.gait_names);
          else
              this.gait_names = path_sequence.gait_names;
          end
        this.gait_durations = path_sequence.gait_durations;


          this.switch_frames = [];
          this.calculate_motions;
         end
        % Set parameter value for class-instance, based on user specified 
        % values (or default if property doesn't exist as struct field)
        function calculate_motions(this)
           gait_library_names = '';
          for i = 1:length(this.gait_library)
              gait_library_names(i) = this.gait_library(i).gait_name;
          end
            this.poses(:,1) = this.pose_1;
            this.Poses(:,1) = this.pose_1;
            if this.include_transitions
                for i = 1:length(this.gait_names)*2-1
                    if mod(i,2) == 0 % if the index is even
                        n_cycles = 1;     % transition between gaits
                        robo_states{i} = [(this.robo_states{i-1}(end)) 1];
                        robo_states{i} = [this.robo_states{i} this.gait_library(gait_library_names == this.gait_names((i/2)+1)).robo_states(1)];
                    else
                        n_cycles = this.gait_durations((i+1)/2);
                        this.gait_idx(i) = gait_library_names == this.gait_names((i+1)/2);
                        robo_states{i} = this.gait_library(this.gait_idx(i)).robo_states;
                    end
                end
            else
                for i = 1:length(this.gait_names)
                    pos = [this.poses(1:2,end); 0];
                    R = eul2rotm([this.poses(3,end) 0 0]);
                    this.switch_frames(2*i-1) = length(this.poses);
                    this.gait_idx(i) = find(gait_library_names == this.gait_names(i));
                    delta_poses = this.gait_library(this.gait_idx(i)).delta_poses;
                    for j = 1:this.gait_durations(i)
                        for k = 1:this.gait_library(this.gait_idx(i)).len_gait
                            delta_x_local = delta_poses(1, k);
                            delta_y_local = delta_poses(2, k);
                            R_local = eul2rotm([delta_poses(3, k) 0 0]);
                            % Use body-frame transformation formula:
                            pos = R*[delta_x_local; delta_y_local; 0] + pos;
                            % Post-multiply for intrinsic rotations.
                            R = R*R_local;
                            % Store global position data for verification.
                            this.poses(:, end+1) = pos;
                            this.poses(3,end) = this.poses(3,end-1) + delta_poses(3,k);
                        end
                    end
                  this.switch_frames(2*i) = length(this.poses);
                  this.Poses(:,end+1) = this.poses(:,end);
                end
            end

        end
        function set_property(this, source_struct, param_name, def_val)
          if ( isfield(source_struct, param_name) )
            this.(param_name) = source_struct.(param_name);
          else
            this.(param_name) = def_val;
          end
        end
        function plot(this)
                           this.path_name = '';
            Poses = this.poses(:, this.switch_frames);
            this.switch_frames(end+1) = length(this.poses);
            w = ((max(Poses(1,:))-min(Poses(1,:))).^2+(max(Poses(2,:))-min(Poses(2,:))).^2).^.5/10;
            hold on
            
            xlabel('x (cm)')
            ylabel('y (cm)')

            hold on
            [gaits_in_path, lgd_idx] = unique(this.gait_names);
            color_array = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30", "#4DBEEE"];
             %plot(key_poses(1,:), key_poses(2,:), 'o', 'LineStyle','none');
            for i = 1:length(this.gait_names)
                h(i) = plot(this.poses(1,this.switch_frames(2*i-1):this.switch_frames(2*i)), ...
                    this.poses(2,this.switch_frames(2*i-1):this.switch_frames(2*i)),...
                    'Color',color_array(this.gait_idx(i)));
                this.path_name = [this.path_name this.gait_names(i) num2str(this.gait_durations(i))];
                if i<length(this.gait_names)
                    this.path_name = [this.path_name ', '];
                end
             %scatter(pose_check(1, :), pose_check(2, :),sz, c2)

            end
                        quiver(Poses(1,:), Poses(2,:), w*cos(Poses(3,:)), w*sin(Poses(3,:)),0,'k')
            %legend('Continuous Robot Position', 'Actual Keyframe Positions', ...
            %       'Keyframe Positions Reconstructed from Motion Primitives')
             daspect([1 1 1]);
             title('Predicted Results', 'FontSize',36)
             subtitle(strjoin(["Path", this.path_name]))
             grid on;
             lgd = legend(h(lgd_idx),gaits_in_path,'Interpreter','latex','FontSize', 24);
             lgd.Orientation = 'horizontal';
             lgd.Location = 'northoutside';


        end
    end % methods
end %classdef
