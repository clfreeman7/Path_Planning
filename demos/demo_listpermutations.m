% Script: DEMO_LISTPERMUTATIONS.m 
%
% Dependencies: 
%   +gaitdef.Gait
%   demos/data
% Description: 
%   Demonstrate the use of the listpermutations.m function and find the
%   symmetric permutations for two gaits. 

% [1] == Script setup
% Add dependencies to classpath
addpath('../');
load data/gait_library_3

% [2] == Extract the gaits from the gait library.
for unique_idx = 1:length(gait_library_3)
    gait_names(unique_idx) = gait_library_3(unique_idx).gait_name;
end

% Define the gaits' actuation sequences. 
gait_G = gait_library_3(gait_names == 'G').robo_states;    % translation gait
gait_B = gait_library_3(gait_names == 'B').robo_states;    % rotation gait


% [3] == Find all symmetric permutations for these gaits (i.e. gait permutations 
% that rotate the actuation sequence by 90 degrees):

% The translation gait has 4 symmetric permutations.
[G_states, n_G] = listpermutations(gait_G, 1, 0)


% [4] == Give these new gaits names based on the direction. 
Directions= {'NW'; 'SW'; 'SE'; 'NE'};
Permutation_Number = (1:4)';

% Label the permutations.
G_perms = table(Permutation_Number, G_states, Directions)




