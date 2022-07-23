clear all; clc; close all

GAIT_LIBRARY_MAT = 'data/gait_library_2_corrected.mat';

% [1] == Script setup
% Add dependencies to classpath
addpath('../');
addpath('./ivaMatlibs');

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

gait_library = load(GAIT_LIBRARY_MAT).gait_library_2;

% [3] == Compute forward kinematic trajectory from planned gait commands
% and gait library data.
theta_0 = 0;
n_gaits = 10;


g_0 = SE2([0,0], theta_0);    % initial scenario (spatial) pose
g_traj = cell(1, n_gaits+1);   % FK-computed trajectory
g_traj{1} = g_0;
gait_twist = gait_library(1).Twist';
gait_period = gait_library(1).len_gait*gait_library(1).transition_time;

for i = 1:n_gaits
  g_start = g_traj{i};    % starting pose


  g_locom = SE2.exp(gait_twist, gait_period);  % relative pose change from gait execution

  g_end = g_start*g_locom;  % final pose after gait execution
  g_traj{i+1} = g_end;
end
% [3] == Compute forward kinematic trajectory from planned gait commands
% and gait library data.
theta_0 = pi;
n_gaits = 10;


g_0 = SE2([0,0], theta_0);    % initial scenario (spatial) pose
g_traj2 = cell(1, n_gaits+1);   % FK-computed trajectory
g_traj2{1} = g_0;
gait_twist = gait_library(1).Twist';
gait_period = gait_library(1).len_gait*gait_library(1).transition_time;

for i = 1:n_gaits
  g_start = g_traj2{i};    % starting pose


  g_locom = SE2.exp(gait_twist, gait_period);  % relative pose change from gait execution

  g_end = g_start*g_locom;  % final pose after gait execution
  g_traj2{i+1} = g_end;
end
R = [cos(theta_0) -sin(theta_0); sin(theta_0) cos(theta_0)];
my_poses(1,:) = [0; 0; theta_0];
pos = [0 0]';
theta = theta_0;
            for i = 1:n_gaits
                
                    delta_x_local = gait_library(1).Delta_Pose(1);
                    delta_y_local = gait_library(1).Delta_Pose(2);
                    R_local = [cos(gait_library(1).Delta_Pose(3)) -sin(gait_library(1).Delta_Pose(3)); sin(gait_library(1).Delta_Pose(3)) cos(gait_library(1).Delta_Pose(3))];
                    % Use body-frame transformation formula:
                    pos = R*[delta_x_local; delta_y_local] + pos;
                    theta = theta+gait_library(1).Delta_Pose(3);
                    % Post-multiply for intrinsic rotations.
                    R = R*R_local;
                    % Store global position data for verification.
                    my_poses(i+1,:) = [pos; theta];
            end
% [4] == Visualize robot trajectories
%   Unpack FK-computed trajectory poses
traj_poses = zeros(3, length(g_traj));
for ii = 1:length(g_traj)
  g = g_traj{ii};

  traj_poses(1:2, ii) = g.getTranslation();
  traj_poses(3, ii) = g.getAngle();
end
traj_poses2 = zeros(3, length(g_traj));
for ii = 1:length(g_traj)
  g = g_traj2{ii};

  traj_poses2(1:2, ii) = g.getTranslation();
  traj_poses2(3, ii) = g.getAngle();
end

%   Plot trajectory
figure;
hold on;
  % FK-computed
  start1 = plot(traj_poses(1, :), traj_poses(2, :), 'go-');

  quiver(traj_poses(1, :), traj_poses(2, :), cos(traj_poses(3, :)), sin(traj_poses(3, :)), ...
          'Color', [0, 0.5, 0], 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
          'AutoScale', 'on', 'AutoScaleFactor', 0.25);
  quiver(traj_poses(1, :), traj_poses(2, :), -sin(traj_poses(3, :)), cos(traj_poses(3, :)), ...    % flip y-axis for display
          'Color', [0, 0.5, 0], 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
          'AutoScale', 'on', 'AutoScaleFactor', 0.25);

  % Planned
  start2 = plot(traj_poses2(1, :), traj_poses2(2, :), 'b*-');

  quiver(traj_poses2(1, :), traj_poses2(2, :), cos(traj_poses2(3, :)), sin(traj_poses2(3, :)), ...
          'Color', [0, 0, 0.5], 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
          'AutoScale', 'on', 'AutoScaleFactor', 0.25);
  quiver(traj_poses2(1, :), traj_poses2(2, :), -sin(traj_poses2(3, :)), cos(traj_poses2(3, :)), ...    % flip y-axis for display
          'Color', [0, 0, 0.5], 'LineWidth', 1, 'MaxHeadSize', 0.1, ...
          'AutoScale', 'on', 'AutoScaleFactor', 0.25);

  % delta_poses
start3 = plot(my_poses(:, 1), my_poses(:,2),'ro', 'LineWidth',5)
hold off;
axis equal; grid on;
% xlim([0.5, 2.5]); ylim([-1.25, 0.75]);
xlabel('X'); ylabel('Y');
title('Robot Trajectory');

lgd = legend([start1, start2, start3], {'\theta_0 = 0', '\theta_0 = \pi', 'my \theta_0 = \pi'},'FontSize',24);
lgd.Location = 'south'
