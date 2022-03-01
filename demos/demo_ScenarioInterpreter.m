% 
% Script: demo_ScenarioInterpreter.m
% 
% Dependencies: 
%   ScenarioInterpreter
% 
% Description: 
%   Obstacle poses and geometries are estimated from a (overhead) binary 
%   image of a locomotion scenario.
% 
% 


% [0] == Script usage:
%   Scenario image
scenario_img_rgb = imread('data/snakey_scenario_rgb.png');     % original color
scenario_img_bin = imread('data/snakey_scenario_binary.png');  % binary (color filtered for obstacles)

%   Scenario properties
scenario_params.num_obstacles = 3;
scenario_params.px2mm = 2.4011;             % mm/pixel
scenario_params.obstacle_dilation = 150;    % mm

%   Image pre-processing (noise filtering)
img_preprocess_params.bwareaopen_size = 450;      % blobs less than this # px are removed
img_preprocess_params.strel_disk_size = 10;       % close holes using disc size

img_preprocess_params.region_ignore = [350, 1080, 1850:1920];   % image exclusion region -> 
                                                                %   rectangle: 
                                                                %     [row start, row end, column start, column end]


% [1] == Script setup
% Add dependencies to classpath
addpath('../');

% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');


% [2] == Interpret obstacle configuration from image
scen_interpreter = ScenarioInterpreter();

scenario_img_bin = uint8(~scenario_img_bin);  % obstacle pixels = 1, other pixels = 0

%   Visualization
figure;
subplot(2, 2, 1);
  imagesc(scenario_img_rgb); 
axis equal;
title('RGB Image');

subplot(2, 2, 2);
  imagesc(scenario_img_bin); 
axis equal;
title('Binary Image');

rgb_img_sp_hdl = subplot(2, 2, 3);
  imagesc(scenario_img_rgb);
axis equal;

clean_bin_sp_hdl = subplot(2, 2, 4);
  clean_bin_img_hdl = imagesc(scenario_img_bin);
axis equal;
title('Enhanced Binary Image');


% [2] == Model obstacle configuration
[ obstacle_scenario ] = scen_interpreter.extract_obstacle_config( scenario_img_bin, scenario_params, img_preprocess_params );

%  Update visualization
%   Overlay on (cleaned) binary scenario image
set(clean_bin_img_hdl, 'CData', obstacle_scenario.scenario_img_clean);

plot_img_space = true;
scen_interpreter.plot_obstacle_config( obstacle_scenario, clean_bin_sp_hdl, plot_img_space );

%   Overlay on RGB scenario image
plot_img_space = true;
scen_interpreter.plot_obstacle_config( obstacle_scenario, rgb_img_sp_hdl, plot_img_space );


