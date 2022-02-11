function eulerTour = randtour(nStates)
% RANDTOUR
% randtour(nStates) creates a n(n-1) vector that specifies the order of
% n states that the robot should assume in order to perform every motion
% primitive (i.e., transition between states) exactly once. 
%
% This function assumes that the states form a complete digraph, wherein
% all states can transition to another and these transitions are directed
% (i.e., the transition from state 1 to state 2 is not the same transition
% as state 2 to state 1).
%
% The resulting vector is called an Euler tour. An Euler tour for a
% complete digraph is not unique.
% 
% This function includes a random number shuffler to ensure the output is
% not always the same Euler tour.
%
% Caitlin Freeman, Agile Robotics Lab (UA)
% 1/27/22

states = 1:nStates;
states = states(randperm(nStates));    % randomly shuffle states
eulerTour = [];

for i = nStates:-1:1
    j = 1;
    while i ~= j
        eulerTour(end+1) = states(j);
        eulerTour(end+1) = states(i);
        j = j+1;
    end

end
eulerTour(end+1) = states(1);
if ~isrow(eulerTour)
    eulerTour = eulerTour';
end
end