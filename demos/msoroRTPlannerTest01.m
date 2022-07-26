% 
% Demonstration of RTGreedyPlanner class.
% 
% 1 rotational motion primitive + 4 translational motion primitives are
% added to the planner's search space. Planner searches and finds sequence
% of rotation-then-translation movements that go from start pose to goal
% position.
%

% [0] == Script usage
ENABLE_VISUALIZATION = false;%true;%false;      %  Enable real-time visualization of planner exploration through grid world

% World selection
world = 4;                              % scenario selection: 1 through 4
world_construct_method = 'actual';      % 'ideal' or 'actual' (ideal planned world vs. actual captured binary world)

% Grid world properties
gridS.dg = 1.565430166666667;     % pixels-to-cm (cm/px)
gridS.cmin = [0, 0];
gridS.size = [336, 336];

% Planning parameters
obstacle_threshold = 10;      % cost value at which a location is identified as obstacle

% Cost map construction
rad_falloff = 20;       % rate of radial decay for obstacle costs (grid units)


% [1] == Script setup
addpath('~/ivalibs/fastmarch');
addpath('~/ivaMatlibs/groups');
addpath('~/ivaMatlibs/control');
addpath('../');

set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');


% [2] == Scenario setup
% Locomotion scenarios - set up grid world and construct cost map
switch world
  case 1 %orange
    if ( strcmp(world_construct_method, 'actual') )
      world_mat_file = '~/ivaMatlibs/control/testing/inputs/World1Dimensions.mat';
      world_img = load(world_mat_file).obstacle_scenario.scenario_img_clean;
    elseif ( strcmp(world_construct_method, 'ideal') )
      world_img = zeros(189, 336);
      world_img(1:124,80:88) = 1;
      world_img(76:189,160:168) = 1;
      world_img(1:112,240:248) = 1;
      world_img = imresize(double(world_img), 1/img_scaling);
    end

    start_pose = [36*gridS.dg ; 36*gridS.dg ; 0] ;
    goal_position = [290 ; 150]*gridS.dg;
  case 2 %black
    if ( strcmp(world_construct_method, 'actual') )
      world_mat_file = '~/ivaMatlibs/control/testing/inputs/World2Dimensions.mat';
      world_img = load(world_mat_file).obstacle_scenario.scenario_img_clean;
    elseif ( strcmp(world_construct_method, 'ideal') )
      world_img = zeros(189, 336);
      world_img(68:144,76:112) = 1;
      world_img(32:120,180:210) = 1;
      world_img = imresize(double(world_img), 1/img_scaling);
    end

    start_pose = [36*gridS.dg ; 30*gridS.dg ; 0] ;
    goal_position = [290 ; 150]*gridS.dg;
  case 3 %blue
    if ( strcmp(world_construct_method, 'actual') )
      world_mat_file = '~/ivaMatlibs/control/testing/inputs/World3Dimensions.mat';
      world_img = load(world_mat_file).obstacle_scenario.scenario_img_clean;
    elseif ( strcmp(world_construct_method, 'ideal') )
      world_img = zeros(189, 336);
      world_img(24:44,68:132) = 1;
      world_img(44:76,68:76) = 1;
      world_img(152:172,92:160) = 1;
      world_img(132:152,124:160) = 1;
      world_img(72:132,204:224) = 1;
      world_img = imresize(double(world_img), 1/img_scaling);
    end

    start_pose = [36*gridS.dg ; 36*gridS.dg ; 0] ;
    goal_position = [298 ; 150]*gridS.dg;
  case 4 %brown
    if ( strcmp(world_construct_method, 'actual') )
      world_mat_file = '~/ivaMatlibs/control/testing/inputs/World4Dimensions.mat';
      world_img = load(world_mat_file).obstacle_scenario.scenario_img_clean;
    elseif ( strcmp(world_construct_method, 'ideal') )
      world_img = zeros(189, 336);
      world_img(24:100,76:100) = 1;
      world_img(136:160,164:212) = 1;
      world_img = imresize(double(world_img), 1/img_scaling);
    end

    start_pose = [36*gridS.dg ; 36*gridS.dg ; 0] ;
    goal_position = [290 ; 75]*gridS.dg;
end


% [3] == Configure trajectory planner
params.gridS = gridS;
params.obstacle_threshold = obstacle_threshold;
msoroRTPlanner = planning.MSoRoRTPlanner(params);

% Set planning scenario (world)
cf = msoroRTPlanner.setScenario( world_img, rad_falloff );

%   Plot grid march world (cost map)
figure(1);
  imagesc(cf);
%   colormap('gray');
  ylim([0 200])
  axis xy;

% Set MSoRo gaits for planning
gait_library = load('data/gait_library_2_corrected.mat').gait_library_2;
rot_gait = gait_library(2);     % rotational gait
trans_gait = gait_library(5);   % translational gait

msoroRTPlanner.setGaits( rot_gait, trans_gait );


% Configure planner visualization (debug aid)
if ( ENABLE_VISUALIZATION )
  vis_config.mode = 1;  % 1 = true (enable real-time visualization of planner exploration)
  fig_hdl = figure;

  fprintf('[msoroRTPlannerTest01] Configuring planner visualization (debug aid).\n');
  msoroRTPlanner.configureVisualization( vis_config, fig_hdl );
end


% [4] == Plan discrete-time controlled trajectory
tic;
  trajectory_plan = msoroRTPlanner.planTrajectory(SE2(start_pose(1:2), start_pose(3)), goal_position);
toc

% Visualize result
% planning.MSoRoRTPlanner.plotTrajectory( trajectory_plan );

    
