% 
% Script: predicted_vs_actual.m
%  
% Dependencies:
%   +demos/data/experiment_2_motion_primitives_corrected.mat
%   +demos/data/NS_motion_primitives.mat
%   +demos/data/exp_S.mat
%   +demos/data/exp_NS.mat
%   +demos/data/gait_library_2_corrected.mat
%   +demos/data/gait_library_S.mat
%   +demos/data/gait_library_NS.mat
%
%   +offlineanalysis/GaitPredict
%   +offlineanalysis/Gait
%
% Description: 
%   Compare experimental and predicted gait trajectories / motion
%   primitives.

% [0] == Script setup
clear; clc; close all
% Add dependencies to classpath
addpath('../');
addpath('ivaMatlibs');
% Configure figure tex interpreters
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
set(0,'defaultAxesFontSize',16)

% Define gait robot states.
gait_B = [16,7,5,11,14];

% Load motion primitive (x y theta) data.
mps_H = load('data/experiment_2_motion_primitives_corrected.mat').motion_primitive_data;
mps_NS = load('data/NS_motion_primitives.mat').motion_primitive_data;

% Load experiment gait data. 
exp_S = load('data/exp_S.mat').exp_S;
exp_NS = load('data/exp_NS.mat').exp_NS;

% Load experimental gait library data. 
gait_library_H = load('data/gait_library_2_corrected.mat').gait_library_2;
gait_library_S = load('data/gait_library_S.mat').gait_library_S;
gait_library_NS = load('data/gait_library_NS.mat').gait_library_NS;


%% [1] == Instantiate GaitPredict() objects for each gait permutation.
% Define parameters.
exp.params.robot_name = 'orange';
exp.params.substrate = 'black mat';

% Calculate heavy sheath (H) tether predicted gait motions.
predicted_H = offlineanalysis.GaitPredict(gait_B, mps_H, exp.params);

% Calculate no sheath (NS) tether predicted gait motions.
predicted_NS = offlineanalysis.GaitPredict(gait_B, mps_NS, exp.params);

% Average the experimental motions for heavy sheath. 
for i = 1:6
    delta_poses_S((i-1)*59+1:i*59, :, :) = cat(3, exp_S(i).delta_x, exp_S(i).delta_y, exp_S(i).delta_theta);
end

gait_mps_S = zeros(length(delta_poses_S), 240,3);
gait_mps_S(:,exp_S(1).primitive_labels, :) = delta_poses_S;

averaged_S = offlineanalysis.GaitPredict(gait_B, gait_mps_S, exp.params);

for i = 1:6
    delta_poses_NS((i-1)*59+1:i*59, :, :) = cat(3, exp_NS(i).delta_x, exp_NS(i).delta_y, exp_NS(i).delta_theta);
end

gait_mps_NS = zeros(length(delta_poses_NS), 240,3);
gait_mps_NS(:,exp_NS(1).primitive_labels, :) = delta_poses_NS;

% Calculate no sheath (NS) tether averaged gait motions.
averaged_NS = offlineanalysis.GaitPredict(gait_B, gait_mps_NS, exp.params);
%averaged_NS = gaitdef.Gait(exp_NS, exp.params);

%% [2] == Plot all motion primitives with 95% confidence intervals.
figure(1)
[pt_H, avg_motions_H, stand_devs_H] = plot_all_deviations(mps_H);
%title(pt_H, 'Heavy sheath tether motion primitives with 95% confidence intervals', ...
%    'FontSize', 24)


figure(2)
[pt_NS, avg_motions_NS, stand_devs_NS] = plot_all_deviations(mps_NS);
%title(pt_NS, 'No sheath tether motion primitives with 95% confidence intervals', ...
 %   'FontSize', 24)

%% [3] == Plot experimental vs. predicted gait trajectories for no sheath tethers.

figure(3)
t=tiledlayout(2,3);
t.TileSpacing = 'tight';
t.Padding = 'compact';
title(t, 'No sheath tether experimental vs. predicted gait trajectory', 'FontSize', 24)
for i = 1:6
    nexttile;
    hold on
    averaged_poses(:,:,i) = averaged_NS.plot(59);
    ax = gca;
    ax.Children(1).Color = 	"black";
    ax.Children(2).MarkerEdgeColor = "none";

    ax.Children(3).MarkerEdgeColor = "black"; 

    predicted_poses(:,:,i) = predicted_NS.plot(59);
    ax = gca;
    ax.Children(1).Color = "blue";
    ax.Children(2).MarkerEdgeColor = "none";
    ax.Children(3).MarkerEdgeColor = "blue"; 
    ax.Children(3).MarkerEdgeAlpha = .8;

    exp_NS(i).plot;
    ax.Children(1).Color = "red";
    ax.Children(2).MarkerEdgeColor = "none";
    ax.Children(3).MarkerEdgeColor = "red"; 
    ax.Children(3).MarkerEdgeAlpha = .8;
    
    xlim([-55 10])
    ylim([-10 45])
    daspect([1 1 1]);
    colormap(flip(gray))
    title(['Experiment ', num2str(i)], 'FontWeight','bold')
    grid on
end

a = colorbar;
a.Label.String = 'Number of Gaits Executed';
a.Layout.Tile = 'east';
lgd = legend([ax.Children(1) ax.Children(5) ax.Children(9)],...
    {'Experimental', 'Predicted from motion primitives', 'Averaged from all experiments'});
lgd.Layout.Tile = 'north';
lgd.Orientation = 'horizontal';
color_array = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30", "#4DBEEE"];
%% [4] == Run Monte Carlo simulation to propogate uncertainty in motion primitives.

figure
hold on;
n_trials = 100;
rng('default')
for i = 1:predicted_NS.len_gait
    predicted_g{i} = SE2([predicted_NS.delta_x(i); predicted_NS.delta_x(i)], predicted_NS.delta_theta(i) );
end



for m = 1:n_trials
    % Initialize the Lie Group.
    g_0 = SE2([0,0], 0);
    g_traj = cell(1,59*5+1);
    k = 1;
    g_poses(:,1) = ones(3,1);
    g_traj{1} = g_0;
    g = g_0;

    % Choose the gait library.
    gait_lib = averaged_NS;

    % Generate random points for normally distributed delta poses. 
    std_x = gait_lib.var_x.^.5;
    mean_x = gait_lib.delta_x;
    for i = 1:5
        delta_x(i,:) = normrnd(mean_x(i), std_x(i), [1,59]);
    end

    std_y = gait_lib.var_y.^.5;
    mean_y = gait_lib.delta_y;
    for i = 1:5
        delta_y(i,:) = normrnd(mean_y(i), std_y(i), [1,59]);
    end

    std_theta = gait_lib.var_theta.^.5;
    mean_theta = gait_lib.delta_theta;
    for i = 1:5
        delta_theta(i,:) = normrnd(mean_theta(i), std_theta(i), [1,59]);
    end


    % Propogate random delta_poses at every motion primitive. 
    for j = 1:59
        for i = 1:predicted_NS.len_gait
            g_next = SE2([delta_x(i,j); delta_y(i,j)], delta_theta(i,j) );
            %g_next = SE2([predicted_NS.delta_x(i); predicted_NS.delta_y(i)], predicted_NS.delta_theta(i));
            g = g*g_next;
            g_poses(1:2,k+1) = g.getTranslation();
            g_poses(3,k+1) = g.getAngle();
            g_traj{k+1} = g;
            k = k+1;
        end
    end
    plot(g_poses(1,:), g_poses(2,:), 'Color',"#FFB8A0")
end
for i = 1:6
% Plot the experimental gait trajectory.
exp_NS(i).plot;

% Change the color scheme.
ax = gca;
ax.Children(1).Color = "none";
ax.Children(2).MarkerEdgeColor = "none";
ax.Children(3).MarkerEdgeColor = "red"; 
ax.Children(3).MarkerFaceColor = "red"; 
ax.Children(3).MarkerFaceAlpha = .5;
end

% Plot the averaged gait trajectory (from gait experiments).
averaged_NS.plot(59);
% Change the color scheme.
ax = gca;
ax.Children(1).Color = 	"black";
ax.Children(2).MarkerEdgeColor = "none";
ax.Children(3).MarkerEdgeColor = "black"; 

colormap(flip(gray))

a = colorbar;
a.Label.String = 'Number of Gaits Executed';
a.Location = 'eastoutside';
lgd = legend([ax.Children(1) ax.Children(7) ax.Children(100)],...
    {'Averaged from all experiments', 'Experimental', '100 Monte Carlo Simulations'});
lgd.Location = 'north';
lgd.Orientation = 'horizontal';
title('Monte Carlo simulations for gait B trajectory with IID assumption')


%% [4] == Run Monte Carlo simulation to propogate uncertainty in motion primitives.

figure
hold on;
n_trials = 100;
rng('default')
for i = 1:predicted_NS.len_gait
    predicted_g{i} = SE2([predicted_NS.delta_x(i); predicted_NS.delta_x(i)], predicted_NS.delta_theta(i) );
end



for m = 1:n_trials
    % Initialize the Lie Group.
    g_0 = SE2([0,0], 0);
    g_traj = cell(1,59*5+1);
    k = 1;
    g_poses(:,1) = ones(3,1);
    g_traj{1} = g_0;
    g = g_0;

    % Choose the gait library.
    gait_lib = averaged_NS;

    % Generate random points for normally distributed delta poses. 
    std_x = gait_lib.var_x.^.5;
    mean_x = gait_lib.delta_x;
    for i = 1:5
        delta_x(i,:) = normrnd(mean_x(i), std_x(i), [1,59]);
    end

    std_y = gait_lib.var_y.^.5;
    mean_y = gait_lib.delta_y;
    for i = 1:5
        delta_y(i,:) = normrnd(mean_y(i), std_y(i), [1,59]);
    end

    std_theta = gait_lib.var_theta.^.5;
    mean_theta = gait_lib.delta_theta;
    for i = 1:5
        delta_theta(i,:) = normrnd(mean_theta(i), std_theta(i), [1,59]);
    end


    % Propogate random delta_poses at every motion primitive. 
    for j = 1:59
        for i = 1:predicted_NS.len_gait
            g_next = SE2([delta_x(i,j); delta_y(i,j)], delta_theta(i,j) );
            %g_next = SE2([predicted_NS.delta_x(i); predicted_NS.delta_y(i)], predicted_NS.delta_theta(i));
            g = g*g_next;
            g_poses(1:2,k+1) = g.getTranslation();
            g_poses(3,k+1) = g.getAngle();
            g_traj{k+1} = g;
            k = k+1;
        end
    end
    plot(g_poses(1,:), g_poses(2,:), 'Color',"#FFB8A0")
end

% Plot the predicted gait trajectory with no noise. 
predicted_NS.plot(59);
% Change the color scheme.
ax = gca;
ax.Children(1).Color = "blue";
ax.Children(2).MarkerEdgeColor = "none";
ax.Children(3).MarkerEdgeColor = "blue"; 
ax.Children(3).MarkerEdgeAlpha = .8;

% Plot the averaged gait trajectory (from gait experiments).
averaged_NS.plot(59);
% Change the color scheme.
ax = gca;
ax.Children(1).Color = 	"black";
ax.Children(2).MarkerEdgeColor = "none";
ax.Children(3).MarkerEdgeColor = "black"; 

colormap(flip(gray))

a = colorbar;
a.Label.String = 'Number of Gaits Executed';
a.Location = 'eastoutside';
lgd = legend([ax.Children(1) ax.Children(5) ax.Children(9)],...
    {'Averaged from all experiments', 'Predicted from motion primitives (no noise)', '100 Monte Carlo Simulations'});
lgd.Location = 'north';
lgd.Orientation = 'horizontal';
title('Monte Carlo simulations for gait B trajectory with IID assumption')


figure
hold on;
for m = 1:n_trials
    % Initialize the Lie Group.
    g_0 = SE2([0,0], 0);
    g_traj = cell(1,59*5+1);
    k = 1;
    g_poses(:,1) = ones(3,1);
    g_traj{1} = g_0;
    g = g_0;

    % Choose initial gait library.
    gait_lib = predicted_NS;
    % Generate random points for normally distributed delta poses. 
    std_x = gait_lib.var_x.^.5;
    mean_x = gait_lib.delta_x;
    new_mean_x = normrnd(mean_x, std_x);

    std_y = gait_lib.var_y.^.5;
    mean_y = gait_lib.delta_y;
    new_mean_y = normrnd(mean_y, std_y);

    std_theta = gait_lib.var_theta.^.5;
    mean_theta = gait_lib.delta_theta;
    new_mean_theta = normrnd(mean_theta, std_theta);

    % Now, propogate with another library's standard deviation. 

    % Choose initial gait library.
    gait_lib = averaged_NS;
    % Generate random points for normally distributed delta poses. 
    std_x = gait_lib.var_x.^.5;
    for i = 1:5
        delta_x(i,:) = normrnd(new_mean_x(i), std_x(i), [1,59]);
    end

    std_y = gait_lib.var_y.^.5;
    for i = 1:5
        delta_y(i,:) = normrnd(new_mean_y(i), std_y(i), [1,59]);
    end

    std_theta = gait_lib.var_theta.^.5;
    for i = 1:5
        delta_theta(i,:) = normrnd(new_mean_theta(i), std_theta(i), [1,59]);
    end



    % Propogate random delta_poses at every motion primitive. 
    for j = 1:59
        for i = 1:predicted_NS.len_gait
            g_next = SE2([delta_x(i,j); delta_y(i,j)], delta_theta(i,j) );
            %g_next = SE2([predicted_NS.delta_x(i); predicted_NS.delta_y(i)], predicted_NS.delta_theta(i));
            g = g*g_next;
            g_poses(1:2,k+1) = g.getTranslation();
            g_poses(3,k+1) = g.getAngle();
            g_traj{k+1} = g;
            k = k+1;
        end
    end
    plot(g_poses(1,:), g_poses(2,:), 'Color',"#FFB8A0")
end

% Plot the predicted gait trajectory with no noise. 
predicted_NS.plot(59);
% Change the color scheme.
ax = gca;
ax.Children(1).Color = "blue";
ax.Children(2).MarkerEdgeColor = "none";
ax.Children(3).MarkerEdgeColor = "blue"; 
ax.Children(3).MarkerEdgeAlpha = .8;

% Plot the averaged gait trajectory (from gait experiments).
averaged_NS.plot(59);
% Change the color scheme.
ax = gca;
ax.Children(1).Color = 	"black";
ax.Children(2).MarkerEdgeColor = "none";
ax.Children(3).MarkerEdgeColor = "black"; 

colormap(flip(gray))

a = colorbar;
a.Label.String = 'Number of Gaits Executed';
a.Location = 'eastoutside';
lgd = legend([ax.Children(1) ax.Children(5) ax.Children(9)],...
    {'Averaged from all experiments', 'Predicted from motion primitives (no noise)', '100 Monte Carlo Simulations'});
lgd.Location = 'north';
lgd.Orientation = 'horizontal';
title('Monte Carlo simulations for gait B trajectory with dependence assumption')


figure
t = tiledlayout(3, 2);
t.TileSpacing = "compact";
t.Padding = 'compact';
%%
% [4] == Plot the predicted behavior with variance for each motion
% primitive of the base gait. 
for j = 1:2
    mp_seq = gait_library_H(2).primitive_labels;
switch j
    case 1
        motion_primitive_gait_data= mps_H(:,mp_seq,:);
        gait_lib = gait_library_H(2);
        delta_poses = gait_lib.delta_poses;
        var_poses = gait_lib.var_delta_poses;
    case 2
        motion_primitive_gait_data= mps_NS(:,mp_seq,:);
        gait_lib = averaged_NS;
        delta_poses = [averaged_NS.delta_x'; averaged_NS.delta_y'; averaged_NS.delta_theta'];
        var_poses = squeeze(var(delta_poses_NS, 0, 1))';
end
% Find mean, variance, and standard deviation of data.
avg_motions = squeeze(mean(motion_primitive_gait_data, 1))';
avg_motions(3,:) = rad2deg(avg_motions(3,:));
var_motions = squeeze(var(motion_primitive_gait_data, 0, 1))';
stand_devs = var_motions.^(.5);
stand_devs(3,:) = rad2deg(stand_devs(3,:));
delta_poses(3,:) = rad2deg(delta_poses(3,:));
stand_dev_poses = var_poses.^(.5);
stand_dev_poses(3,:) = rad2deg(stand_dev_poses(3,:));

for i = 1:3
nexttile(2*(i-1)+j);
x = 1:size(avg_motions,2);
xticks(1:length(gait_B))
%xticklabels(string(predicted_gait_H(j).primitive_labels))
xlim([0.5, length(gait_B)+0.5])
switch i 
    case 1
        ylabel('X-axis translation (cm)', 'FontSize', 16)
        ylim([-2 2.5])
        if j == 1
            title('Heavy sheath (1 experiment)')
        else
            title('No sheath (average for all 6 experiments)')
        end
    case 2 
        ylabel('Y-axis translation (cm)', 'FontSize', 16)
        ylim([-2 1.5])
    case 3 
        ylabel('Rotation (degrees)', 'FontSize', 16)
        ylim([-10 20])
end

grid on 
hold on
plus_sd = avg_motions(i,:)+2*stand_devs(i,:);
minus_sd = avg_motions(i,:)-2*stand_devs(i,:);

p=fill([x fliplr(x)],[plus_sd fliplr(minus_sd)],'b','FaceAlpha',0.2, 'DisplayName',"\pm 2\sigma predicted");
p.EdgeColor = 'b';
stp = stem(x,avg_motions(i,:),'b', 'LineWidth',2,'MarkerFaceColor','b', 'MarkerSize',10,'DisplayName',"average predicted motion");
plus_sdp = delta_poses(i,:)+2*stand_dev_poses(i,:);
minus_sdp = delta_poses(i,:)-2*stand_dev_poses(i,:);
pa=fill([x fliplr(x)],[plus_sdp fliplr(minus_sdp)],'r','FaceAlpha',0.2,'DisplayName',"\pm 2\sigma actual");
pa.EdgeColor = 'r';
sta = stem(x,delta_poses(i,:),'r', 'LineWidth',2,'MarkerFaceColor','r', 'MarkerSize',8,'DisplayName',"average actual motion");

end
end
predd = avg_motions;
actt = delta_poses;


%title(t,'Predicted vs. Actual Motion for each Motion Primitive in a Gait', 'FontSize', 24)
t.XLabel.String = 'Motion primitive index';
t.XLabel.FontSize = 24;
lg = legend([stp p sta pa],'FontSize',24);
lg.Orientation = 'horizontal';
lg.NumColumns = 2;
lg.Layout.Tile = 'north';

figure
t = tiledlayout(3, 6);
t.TileSpacing = "tight";
t.Padding = 'tight';
for j = 1:6
    mp_seq = gait_library_H(2).primitive_labels;
        motion_primitive_gait_data= mps_NS(:,mp_seq,:);
        gait_lib = gait_library_NS(j);

% Find mean, variance, and standard deviation of data.
avg_motions = squeeze(mean(motion_primitive_gait_data, 1))';
avg_motions(3,:) = rad2deg(avg_motions(3,:));
var_motions = squeeze(var(motion_primitive_gait_data, 0, 1))';
stand_devs = var_motions.^(.5);
stand_devs(3,:) = rad2deg(stand_devs(3,:));
delta_poses = gait_lib.delta_poses;
delta_poses(3,:) = rad2deg(delta_poses(3,:));
var_poses = gait_lib.var_delta_poses;
stand_dev_poses = var_poses.^(.5);
stand_dev_poses(3,:) = rad2deg(stand_dev_poses(3,:));

for i = 1:3
nexttile(6*(i-1)+j);
x = 1:size(avg_motions,2);
xticks(1:length(gait_B))
%xticklabels(string(predicted_gait_H(j).primitive_labels))
xlim([0.5, length(gait_B)+0.5])
switch i 
    case 1
        title("No sheath experiment "+ num2str(j))
        ylim([-.5 1])
        set(gca,'YTickLabel',[]);

    case 2 
        ylim([-.5 1.5])
        set(gca,'YTickLabel',[]);
    case 3 
        ylim([-10 10])
        set(gca,'YTickLabel',[]);
end

grid on 
hold on
plus_sd = avg_motions(i,:)+2*stand_devs(i,:);
minus_sd = avg_motions(i,:)-2*stand_devs(i,:);
p=fill([x fliplr(x)],[plus_sd fliplr(minus_sd)],'b','FaceAlpha',0.2, 'DisplayName',"\pm 2\sigma predicted");
p.EdgeColor = 'b';
stp = stem(x,avg_motions(i,:),'b', 'LineWidth',2,'MarkerFaceColor','b', 'MarkerSize',10,'DisplayName',"average predicted motion");

plus_sdp = delta_poses(i,:)+2*stand_dev_poses(i,:);
minus_sdp = delta_poses(i,:)-2*stand_dev_poses(i,:);
pa=fill([x fliplr(x)],[plus_sdp fliplr(minus_sdp)],'r','FaceAlpha',0.2,'DisplayName',"\pm 2\sigma actual");
pa.EdgeColor = 'r';
sta = stem(x,delta_poses(i,:),'r', 'LineWidth',2,'MarkerFaceColor','r', 'MarkerSize',8,'DisplayName',"average actual motion");
end
end

%title(t,'Predicted vs. Actual Motion for each Motion Primitive in a Gait', 'FontSize', 24)
t.XLabel.String = 'Motion primitive index';
t.XLabel.FontSize = 24;
lg = legend([stp p sta pa],'FontSize',24);
lg.Orientation = 'horizontal';
lg.NumColumns = 2;
lg.Layout.Tile = 'north';
nexttile(1);
ylabel('X-axis translation (cm)')
set(gca,'YTick',[-.5 0 .5 1]);
set(gca,'YTickLabel',string([-.5 0 .5 1]));
nexttile(7);
ylabel('Y-axis translation (cm)')
set(gca,'YTick',[-.5 0 .5 1.5]);
set(gca,'YTickLabel',string([-.5 0 .5 1.5]));
nexttile(13);
ylabel('Rotation (degrees)')
set(gca,'YTick',[-10 -5 0 5 10]);
set(gca,'YTickLabel',string([-10 -5 0 5 10]));

function [pt, avg_motions, stand_devs] = plot_all_deviations(mp_data)
    % Calculate average motion primitives. 
    avg_motions = squeeze(mean(mp_data, 1))';
    avg_motions(3,:) = rad2deg(avg_motions(3,:));

    % Calculate standard deviations of motion primitives. 
    var_motions = squeeze(var(mp_data, 0, 1))';
    stand_devs = var_motions.^(.5);
    stand_devs(3,:) = rad2deg(stand_devs(3,:));

    % Plot the averages and standard deviations.
    pt = tiledlayout(3, 1);
    for i = 1:3
        nexttile;
        x = 1:length(avg_motions);

        switch i
            case 1
                ylim([-4 4])
                xlim([1 x(end)])
                ylabel('X-axis translation (cm)')
            case 2
                ylim([-3.5 4.5])
                xlim([1 x(end)])
                ylabel('Y-axis translation (cm)')
            case 3
                ylim([-25 25])
                xlim([1 x(end)])
                ylabel('Rotation (degrees)')
        end

        grid on
        hold on
        xlabel(pt,'Motion Primitive Index', 'FontSize',24) 

        % Plot plus/minus 2 standard deviations (95% confidence).
        plus_sd = avg_motions(i,:)+2*stand_devs(i,:);
        minus_sd = avg_motions(i,:)-2*stand_devs(i,:);
        p=fill([x fliplr(x)],[plus_sd fliplr(minus_sd)],'r','FaceAlpha',0.2);
        p.EdgeColor = 'r';
        plot(x,avg_motions(i,:),'b')
    end
end

% function pt = compare_gait_mps(mps_array, gait_lib_array, gait_numbers)
% figure
% n_plots = length(gait_lib_array);
% t = tiledlayout(3, n_plots);
% t.TileSpacing = "compact";
% t.Padding = 'compact';
% % Define the motion primitive sequence. 
% mp_seq = gait_lib_array{1}(1).primitive_labels;
% for j = 1:n_plots
%     if iscell(gait_lib_array)
%         mp_gait_data = gait_lib_array{j}(:,mp_seq,:);
%     else
%         mp_gait_data = gait_lib_array(gait_numbers(j))(:,mp_seq,:)
%         gait_lib = gait_library_H(2);
%         mp_gait_data= mps_NS(:,mp_seq,:);
%         gait_lib = gait_library_NS(1);
% 
% % Find mean, variance, and standard deviation of data.
% avg_motions = squeeze(mean(mp_gait_data, 1))';
% avg_motions(3,:) = rad2deg(avg_motions(3,:));
% var_motions = squeeze(var(mp_gait_data, 0, 1))';
% stand_devs = var_motions.^(.5);
% stand_devs(3,:) = rad2deg(stand_devs(3,:));
% delta_poses = gait_lib.delta_poses;
% delta_poses(3,:) = rad2deg(delta_poses(3,:));
% var_poses = gait_lib.var_delta_poses;
% stand_dev_poses = var_poses.^(.5);
% stand_dev_poses(3,:) = rad2deg(stand_dev_poses(3,:));
% 
% for i = 1:3
% nexttile(2*(i-1)+j);
% x = 1:size(avg_motions,2);
% xticks(1:length(gait_B))
% %xticklabels(string(predicted_gait_H(j).primitive_labels))
% xlim([0.5, length(gait_B)+0.5])
% switch i 
%     case 1
%         ylabel('X-axis translation (cm)', 'FontSize', 16)
%         if j == 1
%             title('Heavy sheath')
%         else
%             title('No sheath')
%         end
%     case 2 
%         ylabel('Y-axis translation (cm)', 'FontSize', 16)
%     case 3 
%         ylabel('Rotation (degrees)', 'FontSize', 16)
% end
% 
% grid on 
% hold on
% plus_sd = avg_motions(i,:)+2*stand_devs(i,:);
% minus_sd = avg_motions(i,:)-2*stand_devs(i,:);
% % plot(x,plus_sd,'r')
% % plot(x,minus_sd,'r')
% p=fill([x fliplr(x)],[plus_sd fliplr(minus_sd)],'b','FaceAlpha',0.2, 'DisplayName',"\pm 1\sigma predicted");
% p.EdgeColor = 'b';
% stp = stem(x,avg_motions(i,:),'b', 'LineWidth',2,'MarkerFaceColor','b', 'MarkerSize',10,'DisplayName',"average predicted motion");
% 
% plus_sdp = delta_poses(i,:)+2*stand_dev_poses(i,:);
% minus_sdp = delta_poses(i,:)-2*stand_dev_poses(i,:);
% pa=fill([x fliplr(x)],[plus_sdp fliplr(minus_sdp)],'r','FaceAlpha',0.2,'DisplayName',"\pm 1\sigma actual");
% pa.EdgeColor = 'r';
% sta = stem(x,delta_poses(i,:),'r', 'LineWidth',2,'MarkerFaceColor','r', 'MarkerSize',8,'DisplayName',"average actual motion");
% end
% end
% 
% 
% 
% %title(t,'Predicted vs. Actual Motion for each Motion Primitive in a Gait', 'FontSize', 24)
% t.XLabel.String = 'Motion primitive index';
% t.XLabel.FontSize = 24;
% lg = legend([stp p sta pa],'FontSize',24);
% lg.Orientation = 'horizontal';
% lg.NumColumns = 2;
% lg.Layout.Tile = 'north';
% end