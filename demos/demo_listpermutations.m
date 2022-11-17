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
% Find an actuation sequence in the gait that can be uniquely identified
% when performing permutations.
unique_idx = 1;
while ismember(gait_G(unique_idx), [1,16])  % if actuation sequence is [0000] or [1111] 
    unique_idx = unique_idx+1;
end
if unique_idx> length(gait_G)
    error('All symmetric permutations are identical.')
end

% Compare the current order of the allPerms to the "correct" order based on
% sequential 90 degree shifting.
for i = 1:4
    correct_order(i,:) = circshift(dec2bin(gait_G(unique_idx)-1,4), i-1);
    current_order(i,:) = dec2bin(G_states(i)-1,4);
end

[~,order_idx] = ismember(correct_order, current_order,'rows');

% Reorder allPerms_G accordingly:
G_states = G_states(order_idx,:);
Directions= {'NW'; 'SW'; 'SE'; 'NE'};
Permutation_Number = [1:4]';
% Label the permutations.
G_perms = table(Permutation_Number, G_states, Directions)




