clear all; clc; close all
load 'data/MTA3_motion_primitives_carpet.mat';
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
params.substrate = 'carpet';
Sequence = cell(n_sequences, 1);-
Length = zeros(n_sequences, 1);
Translation = zeros(n_sequences, 1);
Rotation = zeros(n_sequences, 1);
Velocity = zeros(n_sequences, 3);
TranslationCost = zeros(n_sequences, 1);

for i = 1:length(all_sequences)
    % Instantiate a GaitPredict() object.
    predicted_gait = offlineanalysis.GaitPredict( all_sequences{i},  motion_primitive_data, params);
    % Instantiate a Gait() object.
    all_gaits(i) = gaitdef.Gait(predicted_gait, params);
    g{i} = SE2.exp(all_gaits(i).Twist', all_gaits(i).len_gait*params.transition_time);
    Sequence{i} = all_gaits(i).robo_states;
    Length(i) = all_gaits(i).len_gait;
    Translation(i) = norm(g{i}.getTranslation);
    Rotation(i) = rad2deg(g{i}.getAngle);
    Velocity(i,:) =all_gaits(i).Twist';
    TranslationCost(i) = abs(sum(all_gaits(i).delta_poses(1,:)))+ abs(sum(all_gaits(i).delta_poses(2,:)));
end
    
comparison_table = table(Sequence, Length, Translation, Rotation, Velocity, TranslationCost);
%%
% Maximize translation.
translation_table = comparison_table(abs(comparison_table.Rotation)<1,:);
translation_table = sortrows(translation_table, "Translation");
% Maximize rotation.
rotation_table = comparison_table(abs(comparison_table.Translation)<.1,:);
rotation_table = sortrows(rotation_table, "Rotation");
