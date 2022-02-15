%Trajectory Specification

% 
% Script: TrajSpec.m
% 
% Dependencies: 
%   +locomgmt.locotraj.LocomotionTrajectory 
%   +gaitdef.Gaits
%   +gaitdef.GaitDirs
%
% Description: 
%   Demonstrate how to populate an instance of 
%     locomgmt.locotraj.LocomotionTrajectory 
% 
% 
function TrajSpec(this)
% [0] == Script setup
% Add dependencies to classpath
addpath('../');

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');


% [1] == Create a LocomotionTrajectory() object
% Define a 9-pose trajectory
%   Note: this trajectory data is arbitrarily generated; it does not
%         represent the actual robot's motion capabilities

poses = zeros(1,length(this.overall_movement(1,:)));
gaits = strings(1,length(this.overall_movement(1,:)));
gait_durations = zeros(1,length(this.overall_movement(1,:)));
gait_directions = strings(1,length(this.overall_movement(1,:)));

for i = 1:length(this.overall_movement(1,:))
    poses(1,i) = this.overall_movement(1,i);
    poses(2,i) = this.overall_movement(2,i);
    poses(3,i) = this.overall_movement(3,i);
    
    if this.overall_movement(4,i) > 4
        gaits(1,i) = gaitdef.Gaits.ROTATE;
    else
        gaits(1,i) = gaitdef.Gaits.TRANSLATE;
    end
    
    gait_durations(1,i) = this.overall_movement(5,i);
    
    if  this.overall_movement(4,i) == 1
        gait_directions(1,i) = gaitdef.GaitDirs.NW;
    elseif this.overall_movement(4,i) == 2
        gait_directions(1,i) = gaitdef.GaitDirs.SW;
    elseif this.overall_movement(4,i) == 3
        gait_directions(1,i) = gaitdef.GaitDirs.SE;    
    elseif this.overall_movement(4,i) == 4
        gait_directions(1,i) = gaitdef.GaitDirs.NE; 
    elseif this.overall_movement(4,i) == 5
        gait_directions(1,i) = gaitdef.GaitDirs.CW; 
    elseif this.overall_movement(4,i) == 6
        gait_directions(1,i) = gaitdef.GaitDirs.CCW; 
    end
end

timestamps = [ 0, cumsum(gait_durations) ];

% Package trajectory data into organized data structure
robo_traj_plan = locomgmt.locotraj.LocomotionTrajectory();

robo_traj_plan.poses = poses;                       % sequence of robot poses
robo_traj_plan.gaits = gaits;                       % gait type to execute at each pose
robo_traj_plan.gait_durations = gait_durations;     % corresponding gait durations (e.g. number of gait cycles)
robo_traj_plan.gait_directions = gait_directions;   % direction for each gait execution
robo_traj_plan.timestamps = timestamps;             % timestamp of each trajectory node

% Visualize robot trajectory
figure(7);

  plot(robo_traj_plan.poses(1, :), robo_traj_plan.poses(2, :), 'bo-');
hold on;
  quiver(robo_traj_plan.poses(1, :), robo_traj_plan.poses(2, :), cos(robo_traj_plan.poses(3, :)), sin(robo_traj_plan.poses(3, :)), ...
          'Color', 'r', 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
          'AutoScale', 'on', 'AutoScaleFactor', 0.25);
  quiver(robo_traj_plan.poses(1, :), robo_traj_plan.poses(2, :), -sin(robo_traj_plan.poses(3, :)), cos(robo_traj_plan.poses(3, :)), ...    % flip y-axis for display
          'Color', 'g', 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
          'AutoScale', 'on', 'AutoScaleFactor', 0.25);
hold off;
axis equal; grid on;
%xlim([0.5, 2.5]); ylim([-1.25, 0.75]);
xlabel('X'); ylabel('Y');
title('Robot Trajectory');


end
