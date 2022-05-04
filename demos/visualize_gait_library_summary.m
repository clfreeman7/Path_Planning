% 
% Script: visualize_gait_library_summary.m
%  
% Dependencies:
%   +demos/data/gait_library_2.mat.mat
%
%   +offlineanalysis/Gait
%
% Description: 
%   Visual summary for library of gait motion data.


% [0] == Script setup
clear; clc
% Add dependencies to classpath
addpath('../');

gait_library = load('data/gait_library_2.mat').gait_library_2;%gait_library_2;

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');


% [1] == Summarize gait motion data

gait_lin_speed = zeros(1, length(gait_library));
gait_rot_speed = zeros(1, length(gait_library));
gait_name = cell(1, length(gait_library));
for ii = 1:length(gait_library)
  gait_data = gait_library(ii);

  gait_lin_speed(ii) = sqrt(gait_data.Twist(1)^2 + gait_data.Twist(2)^2);
  gait_rot_speed(ii) = abs(gait_data.Twist(3)*180/pi);
  gait_name{ii} = gait_data.gait_name;
end


% [2] == Visualize gait behaviors
figure(1);
gait_clrs = [ linspace(0, 1, length(gait_library))', ...
              linspace(1, 0, length(gait_library))', ...
              linspace(0.2, 0.8, length(gait_library))' ];

hold on;
  for ii = 1:length(gait_library)
    plot( gait_lin_speed(ii), gait_rot_speed(ii), 'o', ...
            'MarkerEdgeColor', gait_clrs(ii, :), ...
            'MarkerFaceColor', gait_clrs(ii, :) );
  
    text(gait_lin_speed(ii)+0.05, gait_rot_speed(ii), gait_name{ii}, ...
          'FontSize', 18, 'Color', gait_clrs(ii, :));
  end
hold off;
xlabel('Translational (cm/s)'); ylabel('Rotational (deg/s)');
title('Gait Behavior');
grid on;

xlim([0, max(gait_lin_speed)*1.2]); ylim([0, max(gait_rot_speed)*1.2]);
