clear all; clc; close all
load 'data/MTA3_motion_primitives_mat.mat';
% Add dependencies to classpath
addpath('../');
addpath('ivaMatlibs')
n_unique_states = 8;
% build adjacency matrix
A = ones(n_unique_states, n_unique_states) - diag(ones(n_unique_states,1));
% construct complete 8 node digraph
G = digraph(A);
% Find the state orders for every cycle. 
all_sequences = allcycles(G);
n_sequences = length(all_sequences);
params.robot_name = 'MTA3';
params.n_unique_states = 8;
params.transition_time = .55;
params.substrate = 'black_mat';
Sequence = cell(n_sequences, 1);
Length = zeros(n_sequences, 1);
Translation = zeros(n_sequences, 1);
Rotation = zeros(n_sequences, 1);
Rot_Speed = zeros(n_sequences, 1);
Linear_Speed = zeros(n_sequences, 1);
Velocity = zeros(n_sequences, 3);
TranslationCost = zeros(n_sequences, 1);
Variance_t = zeros(n_sequences, 1);
Variance_theta = zeros(n_sequences, 1);
for i = 1:length(all_sequences)
    % Instantiate a GaitPredict() object.
    predicted_gait = offlineanalysis.GaitPredict( all_sequences{i},  motion_primitive_data, params);
    % Instantiate a Gait() object.
    all_gaits(i) = gaitdef.Gait(predicted_gait, params);
    g{i} = SE2(all_gaits(i).Delta_Pose(1:2,:),all_gaits(i).Delta_Pose(3,:));
    Sequence{i} = all_gaits(i).robo_states;
    Length(i) = all_gaits(i).len_gait;
    Translation(i) = norm(g{i}.getTranslation);
    Rotation(i) = rad2deg(g{i}.getAngle);
    Rot_Speed(i) = abs(Rotation(i))/(all_gaits(i).len_gait*all_gaits(i).transition_time);
    Linear_Speed(i) = abs(Translation(i))/(all_gaits(i).len_gait*all_gaits(i).transition_time);
    Velocity(i,:) =all_gaits(i).Twist';
    TranslationCost(i) = abs(sum(all_gaits(i).delta_poses(1,:)))+ abs(sum(all_gaits(i).delta_poses(2,:)));
    Variance_t(i) = mean([predicted_gait.var_x predicted_gait.var_y]);
    Variance_theta(i) = mean(predicted_gait.var_theta);
end
    
comparison_table = table(Sequence, Length, Translation, Rotation, Velocity, Rot_Speed, Linear_Speed,TranslationCost);
%%
% Maximize translation.
translation_table = comparison_table(abs(comparison_table.Rot_Speed)<1,:);
translation_table = sortrows(translation_table, "Translation");
% Maximize rotation.
rotation_table = comparison_table(abs(comparison_table.Linear_Speed)<.2,:);
rotation_table = sortrows(rotation_table, "Rotation");

save 'exhaustive_MTA3'
opt_t = {[3,6]; [1,8,2]; [1,6,4,5,7];[1,3];[4,5,6,7];[3,8,5];[1,4,7,5,8,3];[3,5];[1,4,7,5,3]};
for i = 1:length(opt_t)
    cell_t = repmat(opt_t{i},length(Sequence),1);
    cell_t = num2cell(cell_t,2);
    idx_t(i) = find(cellfun(@isequal, Sequence, cell_t));
end
opt_theta = {[4,2,1], [3,4,2,6,7,1], [7,5,6,4,2,1]};

figure
scatter(1./comparison_table.Rot_Speed, comparison_table.Linear_Speed)
set(gca,'xscale','log')
hold on
scatter(1./comparison_table.Rot_Speed(idx_t), comparison_table.Linear_Speed(idx_t))
xlabel()
