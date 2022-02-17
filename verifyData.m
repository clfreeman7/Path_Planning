close all; clc; clear;

load 'Euler 1.mat'

vidObj = VideoReader('Euler 1.mp4');



%% Calculate the Position of the Centroid 3 Different Ways
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
pGlobal = zeros(nFrames, 3);
% Third way to find global position - intermediate transformations
%(i.e., intrinsic rotations). Prone to drift.
pLocal = zeros(nFrames, 3);

for i = 1:nFrames
    % Reconstruct global rotation matrix.
    RGlobal = reshape(all_pt(i, 37:45), [3,3]);
    if i == 1
         pGlobal(i,:) = pAvg(i,:);
         pLocal(i,:) = pAvg(i,:);
    else
        % Perform extrinsic rotation relative to global frame.
        pGlobal(i,:) = RGlobal*pGlobal(1, :)' + all_pt(i, 46:48)';
        % Reconstruct intermediate rotation matrix.
        RLocal = reshape(all_pt(i-1, 25:33), [3,3]);
        % Perform succesive intrinsic rotations.
        pLocal(i,:) = RLocal*all_pt(i, 34:36)' + pLocal(i-1, :)';
        %pLocal(i,:) = RLocal*pLocal(i-1, :)' + all_pt(i, 34:36)';
    end
end
%% Calculate Yaw of Robot 2 Different Ways
% First way to calculate yaw - use points and ABSOR (Horn's method).
yawAvg = zeros(nFrames, 1);
markersInitial = [markerPosx(1,:); markerPosy(1,:)];
markersInitial = markersInitial - pAvg(1,1:2)';
% Second  way to calculate yaw - extract from global rotation matrix. 
yawGlobal = zeros(nFrames, 1);

for i = 1:nFrames
    markersCurrent = [markerPosx(i,:); markerPosy(i,:)];
    markersShifted = markersCurrent - pAvg(i,1:2)';
    [regParams,Bfit,ErrorStats]=absor(markersInitial, markersShifted);
    yawAvg(i) = regParams.theta;
    while yawAvg(i) < 0.5
       yawAvg(i) = yawAvg(i)+360;
    end
    % Reconstruct global rotation matrix.
    RGlobal = reshape(all_pt(i, 37:45), [3,3]);
    % Convert to Euler yaw angle. 
    eulAngles = rotm2eul(RGlobal);
    yawGlobal(i) = rad2deg(eulAngles(1));
    while yawGlobal(i) < 0.5
       yawGlobal(i) = yawGlobal(i)+360;
    end
end
%% Plot Results
figure
plot(pAvg(:,1), pAvg(:, 2), 'LineWidth', 1.5)
hold on
plot(pGlobal(:,1), pGlobal(:,2))
%plot(pLocal(:, 1), pLocal(:,2))
xlabel('x (pixels)')
ylabel('y (pixels)')
legend('Average of Markers', 'Global Transformation', 'Local Transformation')
title('Verification of robot centroid calculation')

figure
plot((1:nFrames)/30, yawAvg, 'LineWidth', 1.5)
hold on
plot((1:nFrames)/30, yawGlobal)
legend('ABSOR with Markers', 'Global Transformation')
xlabel('Time (s)')
ylabel('Robot global CCW heading (degrees)')
title('Verirication of robot heading calculation')
