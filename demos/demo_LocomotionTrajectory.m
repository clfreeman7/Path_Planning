% 
% Script: demo_LocomotionTrajectory.m
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
poses = [ 1.0, 1.0, 1.5, 1.5, 2.0, 2.0, 1.5, 1.5, 2.0 ; ...
          0.0, 0.0, 0.5, 0.5, 0.0, 0.0, -0.5, -0.5, -1.0; ....
          10*180/pi, 0*180/pi, 15*180/pi, 3*180/pi, -5*180/pi, 20*180/pi, 30*180/pi, 45*180/pi, 25*180/pi ];   % [ x ; y ; theta ]

gaits = [ gaitdef.Gaits.ROTATE, gaitdef.Gaits.TRANSLATE, ...
          gaitdef.Gaits.ROTATE, gaitdef.Gaits.TRANSLATE, ...
          gaitdef.Gaits.ROTATE, gaitdef.Gaits.TRANSLATE, ...
          gaitdef.Gaits.ROTATE, gaitdef.Gaits.TRANSLATE ];
gait_durations = [ 3, 5, 3, 5, 3, 5, 3, 5 ];      % number of gait periods

gait_directions = [ gaitdef.GaitDirs.CW, gaitdef.GaitDirs.NE, ...
                    gaitdef.GaitDirs.CW, gaitdef.GaitDirs.SE, ...
                    gaitdef.GaitDirs.CW, gaitdef.GaitDirs.SW, ...
                    gaitdef.GaitDirs.CW, gaitdef.GaitDirs.SE ];

timestamps = [ 0, cumsum(gait_durations) ];

% Package trajectory data into organized data structure
robo_traj_plan = locomgmt.locotraj.LocomotionTrajectory();

robo_traj_plan.poses = poses;                       % sequence of robot poses
robo_traj_plan.gaits = gaits;                       % gait type to execute at each pose
robo_traj_plan.gait_durations = gait_durations;     % corresponding gait durations (e.g. number of gait cycles)
robo_traj_plan.gait_directions = gait_directions;   % direction for each gait execution
robo_traj_plan.timestamps = timestamps;             % timestamp of each trajectory node

% Visualize robot trajectory
figure;
hold on;
  plot(robo_traj_plan.poses(1, :), robo_traj_plan.poses(2, :), 'bo-');

  quiver(robo_traj_plan.poses(1, :), robo_traj_plan.poses(2, :), cos(robo_traj_plan.poses(3, :)), sin(robo_traj_plan.poses(3, :)), ...
          'Color', 'r', 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
          'AutoScale', 'on', 'AutoScaleFactor', 0.25);
  quiver(robo_traj_plan.poses(1, :), robo_traj_plan.poses(2, :), -sin(robo_traj_plan.poses(3, :)), cos(robo_traj_plan.poses(3, :)), ...    % flip y-axis for display
          'Color', 'g', 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
          'AutoScale', 'on', 'AutoScaleFactor', 0.25);
hold off;
axis equal; grid on;
xlim([0.5, 2.5]); ylim([-1.25, 0.75]);
xlabel('X'); ylabel('Y');
title('Robot Trajectory');

