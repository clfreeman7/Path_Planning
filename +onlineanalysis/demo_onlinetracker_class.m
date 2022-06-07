cam2 = webcam(2);
preview(cam2)
cam2 = camera_properties_marker_1920_Blue(cam2);
pause(1);

% Tracker configuration
numberofmarkers = 8;
debug_code = 1; % Yes or No (1 or 0)
ifplot = 1; % Yes or No (1 or 0)


onlinetracker_class.numberofmarkers = numberofmarkers;
PrevPt = zeros(8,3);
first_pt = zeros(8,3);
Prev_mean_centroid = zeros(1,3);

for i = 1:25
    
[tr_data(i,:),PrevPt,Prev_mean_centroid,first_pt] = s.videotracker(cam2,PrevPt,Prev_mean_centroid,first_pt,...
                                                                      onlinetracker_class.numberofmarkers,1,1,i);
end