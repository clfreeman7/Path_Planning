%===================Script: calibrator_c1c2.m==============================
% 
% Author: Arun Mahendran Niddish anmahendran@crimson.ua.edu
%
% Dependencies: 
%   +offlineanalysis.rigid_transform_3D 
%   +offlineanalysis.createMaskCarpetBlue
%
% Description: 
%    This code generates the [se2] matrix - Rotation and translation matrix
%    placed globally in the same position but captured in 2 different
%    overhead cameras. It also verifes the projection of the tracked points
%    from camera 1 on camera 2 image.
% 
% Inputs:
% 1) Image from camera 1
% 2) Image from camera 2
%
%==========================================================================

clear all;
clc;

folder = pwd; % Current folder path
baseFileName = 'SE2\Camera1.jpg';  % Video to track
fullFileName = fullfile(folder, baseFileName);
C1 = imread(fullFileName);
newimC1 = createMaskCarpetBlue(C1);
newimC1 = bwareaopen(newimC1,35); 
newimC1 = imfill(newimC1, 'holes');
[labeledImageC1, numberOfRegionsC1] = bwlabel(newimC1);
stats = regionprops(labeledImageC1, 'BoundingBox','Centroid');
    for rb = 1:numberOfRegionsC1
        centroidsC1(rb,:) = stats(rb).Centroid;
    end
zc = zeros(size(centroidsC1,1),1);
centroidsC1 = [centroidsC1,zc];


baseFileName = 'SE2\Camera2.jpg';  % Video to track
fullFileName = fullfile(folder, baseFileName);
C2 = imread(fullFileName);
newimC2 = createMaskCarpetBlue(C2);
newimC2 = bwareaopen(newimC2,35); 
newimC2 = imfill(newimC2, 'holes');
[labeledImageC2, numberOfRegionsC2] = bwlabel(newimC2);
stats = regionprops(labeledImageC2, 'BoundingBox','Centroid');
    for rb = 1:numberOfRegionsC2
        centroidsC2(rb,:) = stats(rb).Centroid;
    end
zc = zeros(size(centroidsC2,1),1);
centroidsC2 = [centroidsC2,zc];


%Calculating se2
[Rot,T] = rigid_transform_3D(centroidsC1', centroidsC2');

    for i = 1:size(centroidsC1,1)
        projection_points(i,:) = Rot*(centroidsC1(i,:))' + T;  %Projection of points C1 on C2.
    end


figure(1)
imshow(C1)
axis on
set(gca);
hold on 
plot(centroidsC1(:,1),centroidsC1(:,2),'r*','LineWidth',2,'MarkerSize',5);
title('Camera 1 - Image');
hold off

figure(2)
imshow(C2)
axis on
set(gca);
hold on 
plot(centroidsC2(:,1),centroidsC2(:,2),'r*','LineWidth',2,'MarkerSize',5);
title('Camera 2 - Image');
hold off

figure(3)
imshow(C2)
axis on
set(gca);
hold on 
plot(projection_points(:,1),projection_points(:,2),'r*','LineWidth',2,'MarkerSize',5);
title('Projection of markers from camera 1 on 2');
hold off
