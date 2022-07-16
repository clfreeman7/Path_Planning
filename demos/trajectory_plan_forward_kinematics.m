% 
% Script: trajectory_plan_forward_kinematics.m
% 
% Dependencies: 
%   +locomgmt.locotraj.LocomotionTrajectory 
%   +gaitdef.Gaits
%   +gaitdef.GaitDirs
%
% Description: 
%   Compute forward kinematic trajectory from planned gait commands and
%   compared to planned trajectory poses.
% 
% 

% [0] == Usage
TRAJECTORY_PLAN_MAT = 'data/msoro_openloop_trajectory.mat';
GAIT_LIBRARY_MAT = 'data/gait_library_2.mat';


% [1] == Script setup
% Add dependencies to classpath
addpath('../');
addpath('~/ivaMatlibs/groups');

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

robo_traj_plan = load(TRAJECTORY_PLAN_MAT).robo_traj_plan;
gait_library = load(GAIT_LIBRARY_MAT).gait_library_2;


% [2] == Unpack LocomotionTrajectory() fields and gata data
%   Locomotion trajectory plan
timestamps = robo_traj_plan.timestamps;
poses = robo_traj_plan.poses;   % 3xN matrix, [ x ; y ; theta ]

gait_names = robo_traj_plan.gait_names;
gait_types = robo_traj_plan.gait_types;
gait_durations  = robo_traj_plan.gait_durations;      % number of gait periods
gait_directions = robo_traj_plan.gait_directions;


% [3] == Compute forward kinematic trajectory from planned gait commands
% and gait library data.

g_0 = SE2(poses(1:2, 1), poses(3, 1));    % initial scenario (spatial) pose

g_traj = cell(1, length(gait_names)+1);   % FK-computed trajectory
g_traj{1} = g_0;

for ii = 1:length(gait_names)
  g_start = g_traj{ii};    % starting pose

  gait_twist = []; gait_period = [];
  for jj = 1:length(gait_library)
    % Note: Gait B -> rotation, Gait E -> translation
    if ( gait_library(jj).gait_name == gait_names(ii) )
      fprintf('FOUND gait: %s [gait_library(%d)]. \n', gait_names(ii), jj);
      gait_twist = gait_library(jj).Twist';
      gait_period = gait_library(jj).len_gait*gait_library(jj).transition_time;

      % Account for SE, NE, NW, SW symmetric gait permutations
      R = [0, -1 ; 1,  0];
      if ( gait_directions(ii) == "SE" )
        gait_twist(1:2) = R*R*gait_twist(1:2);
      elseif ( gait_directions(ii) == "NE" )
        gait_twist(1:2) = R*R*R*gait_twist(1:2);
      elseif ( gait_directions(ii) == "NW" )
        gait_twist(1:2) = gait_twist(1:2);
      elseif ( gait_directions(ii) == "SW" )
        gait_twist(1:2) = R*gait_twist(1:2);
      elseif ( gait_directions(ii) == "CCW" )
        gait_twist(1:2) = R*R*gait_twist(1:2);
      end
      break;
    end
  end
  if ( isempty(gait_twist) )
    error('Gait not found: %s!', gait_names(ii));
  end

  g_locom = SE2.exp(gait_twist, gait_period*gait_durations(ii));  % relative pose change from gait execution

  g_end = g_start*g_locom;  % final pose after gait execution
  g_traj{ii+1} = g_end;
end


% [4] == Visualize robot trajectories
%   Unpack FK-computed trajectory poses
traj_poses = zeros(3, length(g_traj));
for ii = 1:length(g_traj)
  g = g_traj{ii};

  traj_poses(1:2, ii) = g.getTranslation();
  traj_poses(3, ii) = g.getAngle();
end

%   Plot trajectory
figure;
hold on;
  % FK-computed
  fk_traj = plot(traj_poses(1, :), traj_poses(2, :), 'go-');

  quiver(traj_poses(1, :), traj_poses(2, :), cos(traj_poses(3, :)), sin(traj_poses(3, :)), ...
          'Color', [0, 0.5, 0], 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
          'AutoScale', 'on', 'AutoScaleFactor', 0.25);
  quiver(traj_poses(1, :), traj_poses(2, :), -sin(traj_poses(3, :)), cos(traj_poses(3, :)), ...    % flip y-axis for display
          'Color', [0, 0.5, 0], 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
          'AutoScale', 'on', 'AutoScaleFactor', 0.25);

  % Planned
  plan_traj = plot(robo_traj_plan.poses(1, :), robo_traj_plan.poses(2, :), 'b*-');

  quiver(robo_traj_plan.poses(1, :), robo_traj_plan.poses(2, :), cos(robo_traj_plan.poses(3, :)), sin(robo_traj_plan.poses(3, :)), ...
          'Color', [0, 0, 0.5], 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
          'AutoScale', 'on', 'AutoScaleFactor', 0.25);
  quiver(robo_traj_plan.poses(1, :), robo_traj_plan.poses(2, :), -sin(robo_traj_plan.poses(3, :)), cos(robo_traj_plan.poses(3, :)), ...    % flip y-axis for display
          'Color', [0, 0, 0.5], 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
          'AutoScale', 'on', 'AutoScaleFactor', 0.25);
hold off;
axis equal; grid on;
% xlim([0.5, 2.5]); ylim([-1.25, 0.75]);
xlabel('X'); ylabel('Y');
title('Robot Trajectory');

legend([fk_traj, plan_traj], {'FK', 'Plan'});


%   Animation
%     Setup figure
figure;
hold on;
  % FK-computed
  fk_traj = plot(0, 0, 'go-');
  fk_x_axis_hist = quiver(0, 0, 0, 0, ...
                    'Color', [0, 0.5, 0], 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
                    'AutoScale', 'off', 'AutoScaleFactor', 0.25);
  fk_y_axis_hist = quiver(0, 0, 0, 0, ...
                    'Color', [0, 0.5, 0], 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
                    'AutoScale', 'off', 'AutoScaleFactor', 0.25);
  fk_x_axis_cur = quiver(0, 0, 0, 0, ...
                    'Color', [0, 0.5, 0], 'LineWidth', 2, 'MaxHeadSize', 0.1, ...
                    'AutoScale', 'off', 'AutoScaleFactor', 0.25);
  fk_y_axis_cur = quiver(0, 0, 0, 0, ...
                    'Color', [0, 0.5, 0], 'LineWidth', 2, 'MaxHeadSize', 0.1, ...
                    'AutoScale', 'off', 'AutoScaleFactor', 0.25);

  % Planned
  plan_traj = plot(0, 0, 'b*-');
  plan_x_axis_hist = quiver(0, 0, 0, 0, ...
                    'Color', [0, 0, 0.5], 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
                    'AutoScale', 'off', 'AutoScaleFactor', 0.25);
  plan_y_axis_hist = quiver(0, 0, 0, 0, ...
                    'Color', [0, 0, 0.5], 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
                    'AutoScale', 'off', 'AutoScaleFactor', 0.25);
  plan_x_axis_cur = quiver(0, 0, 0, 0, ...
                    'Color', [0, 0, 0.5], 'LineWidth', 2, 'MaxHeadSize', 0.1, ...
                    'AutoScale', 'off', 'AutoScaleFactor', 0.25);
  plan_y_axis_cur = quiver(0, 0, 0, 0, ...
                    'Color', [0, 0, 0.5], 'LineWidth', 2, 'MaxHeadSize', 0.1, ...
                    'AutoScale', 'off', 'AutoScaleFactor', 0.25);

  % Starting pose
  plot(traj_poses(1, 1), traj_poses(2, 1), 'rx');
hold off;
axis equal; grid on;
xlim([0, 300]); ylim([-50, 160]);
xlabel('X'); ylabel('Y');
title_hdl = title('Robot Trajectory (Animation)');
legend([fk_traj, plan_traj], {'FK', 'Plan'});

quiver_len = 10;

for ii = 1:length(g_traj)
  if ( ii == 1 )
    title_hdl.String = 'Starting Pose';
  else
    title_hdl.String = sprintf('After: %s, %s, %d cycles', gait_types(ii-1), gait_directions(ii-1), gait_durations(ii-1));
  end

  set(fk_traj, 'XData', traj_poses(1, 1:ii), 'YData', traj_poses(2, 1:ii));
  set(fk_x_axis_hist, 'XData', traj_poses(1, 1:ii-1), 'YData', traj_poses(2, 1:ii-1), ...
                  'UData', quiver_len*cos(traj_poses(3, 1:ii-1)), 'VData', quiver_len*sin(traj_poses(3, 1:ii-1)));
  set(fk_y_axis_hist, 'XData', traj_poses(1, 1:ii-1), 'YData', traj_poses(2, 1:ii-1), ...
                  'UData', -quiver_len*sin(traj_poses(3, 1:ii-1)), 'VData', quiver_len*cos(traj_poses(3, 1:ii-1)));
  set(fk_x_axis_cur, 'XData', traj_poses(1, ii), 'YData', traj_poses(2, ii), ...
                  'UData', quiver_len*cos(traj_poses(3, ii)), 'VData', quiver_len*sin(traj_poses(3, ii)));
  set(fk_y_axis_cur, 'XData', traj_poses(1, ii), 'YData', traj_poses(2, ii), ...
                  'UData', -quiver_len*sin(traj_poses(3, ii)), 'VData', quiver_len*cos(traj_poses(3, ii)));

  % Planned
  set(plan_traj, 'XData', robo_traj_plan.poses(1, 1:ii), 'YData', robo_traj_plan.poses(2, 1:ii));
  set(plan_x_axis_hist, 'XData', robo_traj_plan.poses(1, 1:ii-1), 'YData', robo_traj_plan.poses(2, 1:ii-1), ...
                    'UData', quiver_len*cos(robo_traj_plan.poses(3, 1:ii-1)), 'VData', quiver_len*sin(robo_traj_plan.poses(3, 1:ii-1)));
  set(plan_y_axis_hist, 'XData', robo_traj_plan.poses(1, 1:ii-1), 'YData', robo_traj_plan.poses(2, 1:ii-1), ...
                    'UData', -quiver_len*sin(robo_traj_plan.poses(3, 1:ii-1)), 'VData', quiver_len*cos(robo_traj_plan.poses(3, 1:ii-1)));
  set(plan_x_axis_cur, 'XData', robo_traj_plan.poses(1, ii), 'YData', robo_traj_plan.poses(2, ii), ...
                    'UData', quiver_len*cos(robo_traj_plan.poses(3, ii)), 'VData', quiver_len*sin(robo_traj_plan.poses(3, ii)));
  set(plan_y_axis_cur, 'XData', robo_traj_plan.poses(1, ii), 'YData', robo_traj_plan.poses(2, ii), ...
                    'UData', -quiver_len*sin(robo_traj_plan.poses(3, ii)), 'VData', quiver_len*cos(robo_traj_plan.poses(3, ii)));

  input('Press <Enter>');
end


