function [allPerms, nPerms] = listpermutations(gaitCycle, issymmetric, isCyclic)
% LISTPERMUTATIONS  
% listpermutations(gaitCycle) lists all m unique cyclic and symmetric
% permutations of the robot states in the n-vector gaitCycle as rows in an
% m x n matrix .
%
% For an assymmetric robot:
% listpermutations(gaitCycle, issymmetric) lists all m uniquen cyclic and
% symmetric permutations of the robot states in the n-vector gaitCycle as
% rows in an m x n matrix. Symmetric permutations will be excluded if
% issymmetric = false.
%
% isCylic toggles whether cyclic permutations are calculated. 
%
% [allPerms, nPerms] = listpermutations(gaitCycle) will also list the
% total number nPerms of cyclic and symmetric permutations. 
%
% [~, nPerms] = listpermutations(gaitCycle) will only list the
% total number nPerms of cyclic and symmetric permutations and will not
% list all of the permutations
%
% Caitlin Freeman, Agile Robotics Lab (UA)
% 1/27/22



    if nargin < 2
        issymmetric = true;
    end
   
    if ~isvector(gaitCycle)
        error('gaitCycle must be a vector of robot state labels.');
    end
   
    allPerms(1, :) = gaitCycle;
    lenCycle = length(gaitCycle);
    
    % Find symmetric permutation states
    if issymmetric
        origStates= dec2bin(0:15, 4);    % actuation states as 4 bit strings
        symStates = origStates;
        symLabels(1, :) = 1:16;
        for i = 1:3
            symStates = circshift(symStates, 1, 2);
            [~, symLabels(i+1, :)] = ismember(symStates, origStates, 'rows');
        end
    end
    
    % Find symmetric permutation cycles 
    for i = 1:lenCycle
        if isCyclic
            % Find cyclic permutations
           cyclicPerms = circshift(gaitCycle, i);
        else
            cyclicPerms = gaitCycle;
        end
       if issymmetric
           symPerms = symLabels(:, cyclicPerms);
           allPerms = [allPerms; symPerms];
       else
           if i < lenCycle
               allPerms = [allPerms; cyclicPerms];
           end
       end
    
    % Remove any repeated permutations
    allPerms = unique(allPerms, 'rows');
    [nPerms, ~] = size(allPerms);
end
