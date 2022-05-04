% ======================= LocomotionTrajectory =======================
%
%  Data structure encapsulating locomotion trajectory information for the
%  MSoRo mobile robot.
%
%  LocomotionTrajectory()
%
%
%  ====================== LocomotionTrajectory ========================
classdef LocomotionTrajectory < handle
  properties  (Access = public)
    % Trajectory
    timestamps;       % sequence of timestamps

    poses;            % sequence of robot poses [x ; y ; theta]

    gait_names;       % should be cell array (or array) of chars representing 
                      %     sequence of gaits comprising trajectory (e.g. 'A', 'B', 'A', 'B')

    gait_types;       % sequence of +gaitdef.GaitType enumerations (TRANSLATE, ROTATE)

    gait_durations;   % corresponding gait durations (time or number of gait periods)

    gait_directions;  % sequence of +gaitdef.GaitDir enumerations (NE, NW, SW, SE, CW, CCW)
                      % Note: this is probably a temporary field until the
                      %         MSoRo trajectory execution/management is
                      %         better hashed out.
  end
  
  methods
    % Constructor
    function this = LocomotionTrajectory( params )
      if ( nargin < 1 )
        params = [];
      end
      
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
    
  end     % methods

end     % class


