function tracking_data = onlinetracker(outputfilename_original,outputfilename_tracked,options);
if nargin>2
    disp('In nargin');
    number_of_markers = options.numberofmarkers;
    ifplot = options.outputvideo;
    debug_code = options.debug_code;
else
%   default values
    number_of_markers = 8;
    ifplot = 1;
    debug_code = 0;
end

% fullFileName = file_name(inputfilename); % To get the directory to video

[vwrite_original,vwrite_tracked] = video_params(outputfilename_original,outputfilename_tracked); % Creating objects for the video


cam1 = webcam(1);        
cam2 = webcam(2);

preview(cam1)
preview(cam2)
pause(1);

choice = menu("Press to start tracking",'Start','Exit');

    if(choice == 1)
        tracking_data = videotracker(cam1,cam2,vwrite_original,vwrite_tracked,number_of_markers,ifplot,debug_code); % Marker data
    end
    if(choice == 2)
        return
    end
    
end

function [vwrite_original,vwrite_tracked] = video_params(video_write_original_fname,video_write_tracked_fname)

% Creating object for video writing

  vwrite_original = VideoWriter(video_write_original_fname,'MPEG-4');
  open(vwrite_original);
              
  vwrite_tracked = VideoWriter(video_write_tracked_fname,'MPEG-4');
  open(vwrite_tracked);
  disp('video params');
end

% ============================ videotracker ===============================
%
% Author: Arun Niddish Mahendran anmahendran@crimson.ua.edu
%
% Description:
%    This function tracks the centroid points of the MSoRo and provides the
%    rotation and translation in global (w.r.t. first) and body frame for
%    live video.
% 
% Inputs:
% vwrite_original - videowriter object for real video.
% vwrite_tracked - videowriter object for tracked video.
% number_of_markers - number of markers. Default - 8
% ifplot - to plot or not plot the data.
% cam1 - webcam object for capturing marker
% cam2 - webcam object for capturing HD video
% =========================================================================

function tracking_data = videotracker(cam1,cam2,vwrite_original,vwrite_tracked,...
    number_of_markers,ifplot,debug_code)

start_frame = 1;
k = 1;

while(true)
%     tic;
    if k == start_frame
        centroids = zeros(8000,3*number_of_markers);
        projection_points = zeros(number_of_markers,3);
        d = zeros(1,1);
        cam1 = camera_properties_marker(cam1);
%         cam2 = camera_properties_marker(cam2);
        pause(1);
        c_res = menu("Choose the resolution of the video",'640x480','1920x1080');
       
        if c_res == 1
            distortion = load('D:\Arun Niddish\Vision\Visual Tracking\Callibration images(640x480)\cameraParams')   %Parameters for distortion correction for 640x480
            Rot_projection = load('D:\Arun Niddish\Vision\Visual Tracking\SE2\640x480\Rotation_matrix')
            T_projection = load('D:\Arun Niddish\Vision\Visual Tracking\SE2\640x480\Translation_matrix')
            marker_size = 5;
            cam1.Resolution = cam1.AvailableResolutions{1};     %Setting resolution to 640x480
            cam2.Resolution = cam2.AvailableResolutions{1};     %Setting resolution to 640x480
            pause(1);
        end
        
        if c_res == 2
            distortion = load('D:\Arun Niddish\Vision\Visual Tracking\Callibration images(1920x1080)\cameraParams')   %Parameters for distortion correction for 1920x1080
            Rot_projection = load('D:\Arun Niddish\Vision\Visual Tracking\SE2\1920x1080\Rotation_matrix')
            T_projection = load('D:\Arun Niddish\Vision\Visual Tracking\SE2\1920x1080\Translation_matrix')
            marker_size = 40;
            cam1.Resolution = cam1.AvailableResolutions{6};     %Setting resolution to 1920x1080
            cam2.Resolution = cam2.AvailableResolutions{6};     %Setting resolution to 1920x1080
            pause(1);
        end
    end

%Capturing the original video in good quality
        thisFrame_original = cam2.snapshot();
        thisFrame_original = undistortImage(thisFrame_original,distortion.cameraParams);       %Image correction
        writeVideo(vwrite_original,thisFrame_original);
        
%Capturing image for tracking        
        thisFrame = cam1.snapshot();
        thisFrame = undistortImage(thisFrame,distortion.cameraParams);       %Image correction
%         thisFrame = imcrop(thisFrame,[0,0,630,400]);
%         newim = createMaskfixedblue3(thisFrame);
        newim = createMaskCarpetBlue(thisFrame);
%         newim = createMaskpink2(thisFrame);
%         newim = createMaskpink4(thisFrame);
        newim = bwareaopen(newim,marker_size);
        newim = imfill(newim, 'holes');
%         axis on;

        [labeledImage, numberOfRegions] = bwlabel(newim);
        frame(k,:) = numberOfRegions;       
        count = 0;
        cent = [];
        stats = regionprops(labeledImage, 'BoundingBox','Centroid','Area','EquivDiameter');
                 for rb = 1:numberOfRegions
                     count = count + 1;
                     cent(count,:) = stats(rb).Centroid;
                 end
        zc = zeros(size(cent,1),1);
        cent = [cent,zc];
               
        if k == start_frame
            P0 = cent;
            PrevPt = cent;
                for i = 1:number_of_markers
                    centroids(k,(3*i)-2:(3*i)) = P0(i,:);
                end
        end


  % Cent is available here. This will be the new one.
  % PPt is also available here.
 
  if k ~= start_frame && count > number_of_markers
      resrvd = zeros(number_of_markers,3);
      for i = 1:number_of_markers
          for j = 1:count
              X = [PrevPt(i,:);cent(j,:)];
              d(j) = pdist(X,'euclidean');
          end
              [dmin,ind] = min(d);  
              if(dmin < 12)
              resrvd(i,:) = cent(ind,:);
              end
      end
  end
 
   
  if k ~= start_frame && count <= number_of_markers
      resrvd = zeros(number_of_markers,3);
      for i = 1:count
          for j = 1:number_of_markers
              X = [cent(i,:);PrevPt(j,:)];
              d(j) = pdist(X,'euclidean');
          end
                [dmin,ind] = min(d);
                if(dmin < 12)
                   resrvd(ind,:) = cent(i,:);
                end
      end
  end
 
if k ~= start_frame
% Calculation of rotation matrix and translation
TF = resrvd(:,1);  % Writing the 1st column of resrvd
index = find(TF == 0);  % Finding those rows which is empty
val = isempty(index);   % Checking whether the index is empty
newPrevPt = PrevPt;
newP0 = P0;             % 1st point
if(val == 0)            % That means checking for index whether it is empty(if yes val is 1 or val is 0)
newPrevPt(index(1:size(index,1)),:) = 0;
newP0(index(1:size(index,1)),:) = 0;      %
end

[Rot,T] = rigid_transform_3D(newPrevPt', resrvd');  % SE2 w.r.t previous frame

theta(k,:) = reshape(Rot,[1,9]);   % Changed ANM
trans(k,:) = T';

if(val == 0)
    for gg = 1:size(index,1)
        newPt = Rot*(PrevPt(index(gg),:))' + T;
        resrvd(index(gg),:) = newPt;
    end
end

[Rot_G,T_G] = rigid_transform_3D(P0', resrvd');  % SE2 w.r.t 1st frame

theta_G(k,:) = reshape(Rot_G,[1,9]);   % Rotation matrix w.r.t 1st frame
trans_G(k,:) = T_G';                   % Translation matrix w.r.t 1st frame

PrevPt = resrvd;
clear d;

% In the respective centroid variables
    for i = 1:number_of_markers
      centroids(k,(3*i)-2:(3*i)) = resrvd(i,:);
    end
end

if debug_code
    
      for i = 1:number_of_markers
          projection_points(i,:) = Rot_projection.Rot*(PrevPt(i,:))' + T_projection.T;  %Projection of points C1 on C2.
      end
    if k == start_frame
    debug_plot = figure;
    debug_plot.Name = 'Debug: Marker tracking';
    debug_plot.NumberTitle = 'off';
    end
    figure(debug_plot)
    % Plotting the points
    imshow(thisFrame_original)
%     debug_plot('Units','normalized','Position',[0 0 1 1])
%     set(gcf, 'Position',  [100, 100, 750, 400])
      set(gcf,'Units','normalized','Position',[0 0 1 1]);
    hold on
%     plot(PrevPt(:,1),PrevPt(:,2),'g*','LineWidth',0.5,'MarkerSize',5)  % Actual marker points
    plot(projection_points(:,1),projection_points(:,2),'g*','LineWidth',0.5,'MarkerSize',5)  % Projected marker points
%     mean_centroid = mean(PrevPt);
    % plot([PrevPt(1,1) PrevPt(2,1)], [PrevPt(1,2) PrevPt(2,2)],'LineWidth',1.5)
    caption = sprintf('%d blobs found in frame #%d', count, k);
    title(caption, 'FontSize', 20);
    axis on
    hold off 
end

if ifplot
    
    if(rem(k,50) == 1)
    
    mean_centroid = mean(PrevPt);
    
        if k == start_frame
            centroid_plot = figure;
            centroid_plot.Name = 'Centroid Position';
            centroid_plot.NumberTitle = 'off';
            
            figure(centroid_plot)
            p1 = plot(mean_centroid(1,1),mean_centroid(1,2),'ro');
            p1.MarkerSize = 10;
            xlim([0 1920]);
            ylim([0 1920]);
            hold on
        end

        if k ~= start_frame
            figure(centroid_plot)
            p2 = plot(mean_centroid(1,1),mean_centroid(1,2),'bo');
            p2.MarkerSize = 4;
            p2.MarkerFaceColor = '#EDB120';
            line([Prev_mean_centroid(1,1) mean_centroid(1,1)],...
                [Prev_mean_centroid(1,2) mean_centroid(1,2)]);
        end

    axis on
    Prev_mean_centroid = mean_centroid;
    end
end

% Pushbutton to exit the code
if k == start_frame 

figure(11)

    p = uipanel('Title','Press to stop tracking','FontSize',12,'FontWeight','bold',...
            'TitlePosition','centertop');
        buttonhandle = uicontrol(p,'Style', 'toggleButton', ...
                                 'String', 'Stop tracking', ...
                                 'Callback', 'delete(gcbf)',...
                                 'FontWeight','bold','Position',[230 100 100 22]);
end

    if ~ishandle(buttonhandle)
        disp('Tracking stopped by user');
        break;
    end

% toc ;
% pframe = getframe(gcf);
% writeVideo(vwrite_tracked,pframe);
k = k+1;
end
% close(vwrite_tracked);
close(vwrite_original);

sizeoftrans = size(trans,1);
centroids = centroids(1:sizeoftrans,:);
% Collectively writing all the tracked points in a single variable.
tracking_data = cat(2,centroids,theta,trans,theta_G,trans_G);
end