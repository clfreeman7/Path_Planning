% FIND_PERMUTATIONS.m 
% This script is used to show the functionality of the listpermutations
% function.
%
%
% Caitlin Freeman, Agile Robotics Lab (UA)
% 1/27/22

gait1 = [2, 13];     % translation gait
gait2 = [2, 16, 13, 4, 12, 15];     % rotation gait

% Find all permutations for these gaits:

% The translation gait has 8 total permutations (cyclic and symmetric).
[allPerms1, n1] = listpermutations(gait1)

% The rotation gait has 24 total permutations (cyclic and symmetric).
[allPerms2, n2] = listpermutations(gait2)
