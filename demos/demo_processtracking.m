% This sciprt processes the raw data from visual tracking and finds the
% translation and rotation for each motion primitive. 

close all; clc; clear;

% Add dependencies to path
addpath('./');

load 'data/visualtracking/Euler 1.mat'
load 'data//visualtracking/stateOrder.mat'
trialNum = 1;
isplotted = true;
mmPerPixel = 2.426;    % mm / pixel
frameRate = 30;        % frames / sec
transitionTime = .45;  % sec / transition (motion primitive)
% Find continuous values (i.e., all frames) for SE(2) pose = [x y theta]. 
nFrames = length(all_pt);
markerPosx = all_pt(:, 1:3:22);
markerPosy = all_pt(:, 2:3:23);
markerPosz= all_pt(:, 3:3:24);
% First way to find global position - lazy approach (average of points). 
% Assumes perfect circle with equally spaced markers.
pAvg = zeros(nFrames, 3);
pAvg(:, 1) = mean(markerPosx, 2);
pAvg(:, 2) = mean(markerPosy, 2);
pAvg(:, 3) = mean(markerPosz, 2);
% Second way to find global position - global transformations. 
pGlobal = zeros(nFrames, 3);      % robot position based on global rotm
yawGlobal = zeros(nFrames, 1);    % robot heading based on global rotm
for i = 1:nFrames
    % Reconstruct global rotation matrix.
    RGlobal = reshape(all_pt(i, 37:45), [3,3]);
    if i == 1
        pGlobal(i,:) = pAvg(i, :);
    else
        % Perform extrinsic rotation relative to global frame.
        pGlobal(i,:) = RGlobal*pGlobal(1, :)' + all_pt(i, 46:48)';
    end
    % Convert to Euler yaw angle. 
    eulAngles = rotm2eul(RGlobal);
    yawGlobal(i) = eulAngles(1);
end
% Convert from pixels to cm.
pGlobal = pGlobal*10/mmPerPixel;
% Calculate from here using only SE(2) normalized to start at the origin.
p = pGlobal(: , 1:2) - pGlobal(1, 1:2);

% Find the frame where movement starts.
diffP = zeros(nFrames, 2);
for i = 2:nFrames
    diffP(i, :) = p(i, :) - p(i-1, :); 
end
i = 2;
while (norm(p(i, :) - p(i-1, :))) < 4
    i = i+1;
end
initialFrame = 192;

%% Extract keyframe data (frames 1 frame before next transition starts).
framesPerTrans = frameRate * transitionTime; 
if (framesPerTrans - floor(framesPerTrans)) < 0.05
    % If # frames per transition is close to an integer: 
    framesPerTrans = floor(framesPerTrans);    % make integer
    unorderedPoses(1:2, :) = p(initialFrame - 1:framesPerTrans:end);
else 
    keyframes = zeros(242, 1);
    keyframes(1) = initialFrame - 1;
    for k = 2:242
        if mod(k, 2) == 0 % if even index
            keyframes(k) = keyframes(k-1) + ceil(framesPerTrans);
        else
            keyframes(k) = keyframes(k-1) + floor(framesPerTrans);
%          if (i - keyframes(i-1)) == floor(framesPerTrans)
%              lowDifference = norm(p(i, :) - p(i-1, :));
%              highDifference = norm(p(i+1, :) - p(i, :));
%              if lowDifference < highDifference
%                  keyframe(i) = i;
        end
    end
end
% Keyframe poses [x y theta]'
poseUnordered = [p(keyframes, :)' ; yawGlobal(keyframes)'];
%% Find changes in pose for each transition.
motionPrimitivesUnordered = zeros(3, 240);
pCheck(1, :) = [poseUnordered(1:2, 2); 0];
for k = 1:240     % total number of transitions in Euler tour.
    % Calculate change in robot heading.
    motionPrimitivesUnordered(3, k) = poseUnordered(3, k+2) - poseUnordered(3, k+1);
    % Calculate change in robot position.
    pDiffGlobal = poseUnordered(1:2, k+2) - poseUnordered(1:2, k+1);
    % Orientation of initial pose w.r.t. global frame.
    RGlobal = eul2rotm([yawGlobal(k+1) 0 0]);
    %RGlobal = reshape(all_pt(keyframes(k+1), 37:45), [3,3]);
    pDiffLocal = RGlobal' * [pDiffGlobal; 0];
    motionPrimitivesUnordered(1:2, k) = pDiffLocal(1:2);
    transitionOrder(k) = inversemap(stateOrder(trialNum, k), stateOrder(trialNum, k+1), 16);
    pCheck(k+1, :) = pCheck(k, :)' + RGlobal*(pDiffLocal);
end
[~, orderIndex] = sort(transitionOrder);
% Sort position and heading data. 
motionPrimitives = motionPrimitivesUnordered(:, orderIndex);


if isplotted
    figure(1)
    hold on
    plot(p(1:3660,1), p(1:3660,2))
    xlabel('x (cm)')
    ylabel('y (cm)')
    title('Trial 1: Verification of Motion Primitive Calculations')
    hold on
    sz = 30;
    c1 = linspace(1,10,length(keyframes));
    c2 = [0.6350 0.0780 0.1840];
    scatter(p(keyframes, 1), p(keyframes, 2), sz, c1, 'filled')
    scatter(pCheck(:, 1), pCheck(:, 2),sz, c2)
    legend('Continuous Robot Position', 'Actual Keyframe Positions', ...
           'Keyframe Positions Reconstructed from Motion Primitives')
       
    
end

