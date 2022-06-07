classdef onlinetracker_class < handle
    
    properties(Access = public)
        
     numberofmarkers;
     
     debug_code;
     
     ifplot;
     
    end


methods
    function [tracking_data,PrevPt,Prev_mean_centroid,P0] = videotracker(obj,cam2,previous_pt,Previous_centroid,first_pt,...
                                       number_of_markers,ifplot,debug_code,k)
                                   
                                  
        start_frame = 1;
        theta = zeros(1,9);
        trans = zeros(1,3);
        theta_G = zeros(1,9);
        trans_G = zeros(1,3);
        centroids = zeros(1,3*number_of_markers);
        PrevPt = previous_pt;
        P0 = first_pt;
        Prev_mean_centroid = Previous_centroid;
        projection_points = zeros(number_of_markers,3);
        d = zeros(1,1);
           
          
        %Capturing image for tracking        
            thisFrame = cam2.snapshot();           
            distortion = load('D:\Arun Niddish\Vision\Visual Tracking\Callibration images(1920x1080)\cameraParams')   %Parameters for distortion correction for 1920x1080
            Rot_projection = load('D:\Arun Niddish\Vision\Visual Tracking\SE2\1920x1080\Rotation_matrix')
            T_projection = load('D:\Arun Niddish\Vision\Visual Tracking\SE2\1920x1080\Translation_matrix')
            marker_size = 12;
            thisFrame = undistortImage(thisFrame,distortion.cameraParams);       %Image correction
%         thisFrame = imcrop(thisFrame,[0,0,1920,500]);
        newim = createMaskhdblue(thisFrame);
        newim = bwareaopen(newim,marker_size);
        newim = imfill(newim, 'holes');
%         axis on;

        [labeledImage, numberOfRegions] = bwlabel(newim);       
        count = 0;
        cent = [];
%         keyboard
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
                    centroids(1,(3*i)-2:(3*i)) = P0(i,:);
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
              if(dmin < 18)
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
                if(dmin < 18)
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

theta = reshape(Rot,[1,9]);   % Changed ANM
trans = T';

if(val == 0)
    for gg = 1:size(index,1)
        newPt = Rot*(PrevPt(index(gg),:))' + T;
        resrvd(index(gg),:) = newPt;
    end
end

[Rot_G,T_G] = rigid_transform_3D(P0', resrvd');  % SE2 w.r.t 1st frame

theta_G = reshape(Rot_G,[1,9]);   % Rotation matrix w.r.t 1st frame
trans_G = T_G';                   % Translation matrix w.r.t 1st frame

PrevPt = resrvd;
clear d;

% In the respective centroid variables
    for i = 1:number_of_markers
      centroids(1,(3*i)-2:(3*i)) = resrvd(i,:);
    end
end

if ifplot
    
%     if(rem(k,30) == 1)
    
    mean_centroid = mean(PrevPt);
    
        if k == start_frame
%             centroid_plot = figure;
%             centroid_plot.Name = 'Centroid Position';
%             centroid_plot.NumberTitle = 'off';
            
            figure(20)
            p1 = plot(mean_centroid(1,1),mean_centroid(1,2),'ro');
            p1.MarkerSize = 10;
            xlim([0 1920]);
            ylim([0 1080]);
            hold on
        end

        if k ~= start_frame
%             figure(centroid_plot)
              figure(20)
%             p2 = plot(mean_centroid(1,1),mean_centroid(1,2),'bo');
%             p2.MarkerSize = 4;
%             p2.MarkerFaceColor = '#EDB120';
            line([Prev_mean_centroid(1,1) mean_centroid(1,1)],...
                [Prev_mean_centroid(1,2) mean_centroid(1,2)]);
        end
    axis on
    Prev_mean_centroid = mean_centroid;
%     end
end

if debug_code
    
      for i = 1:number_of_markers
          projection_points(i,:) = Rot_projection.Rot*(PrevPt(i,:))' + T_projection.T;  %Projection of points C1 on C2.
      end
      
%     if k == start_frame
%         debug_plot = figure;
%         debug_plot.Name = 'Debug: Marker tracking'; 
%         centroid_plot.NumberTitle = 'off';
%     end
  
    figure(21)
    clf(21);
    h1 = axes;
    % Plotting the points
%     imshow(thisFrame_original);
%     set(debug_plot, 'Visible', 'off');
%     set(gcf, 'Position',  [100, 100, 750, 400])
%     hold on
%     plot(PrevPt(:,1),PrevPt(:,2),'g*','LineWidth',0.5,'MarkerSize',5)  % Actual marker points
    plot(projection_points(:,1),projection_points(:,2),'g*','LineWidth',0.5,'MarkerSize',5)  % Projected marker points
    caption = sprintf('%d blobs found in frame #%d', count, k);
    title(caption, 'FontSize', 20);
    xlim([0 1920]);
    ylim([0 1080]);
    set(h1,'Ydir','reverse')
    set(gcf,'Units','normalized','Position',[0 0 0.5 0.5]);
    
    axis on
%     keyboard
%     pframe = getframe(gcf);
%     writeVideo(vwrite_tracked,pframe);
%     hold off
    
end

% Pushbutton to exit the code
% if k == start_frame 
% figure(11)
% 
%     p = uipanel('Title','Press to stop tracking','FontSize',12,'FontWeight','bold',...
%             'TitlePosition','centertop');
%     buttonhandle = uicontrol(p,'Style', 'toggleButton', ...
%                                  'String', 'Stop tracking', ...
%                                  'Callback', 'delete(gcbf)',...
%                                  'FontWeight','bold','Position',[230 100 100 22]);
% end
% 
%     if ~ishandle(buttonhandle)
%         disp('Tracking stopped by user');
%         break;
%     end

toc ;
% pframe = getframe(gcf);
% writeVideo(vwrite_tracked,pframe);
% k = k+1;



% end
% close(vwrite_tracked);
% close(vwrite_original);



sizeoftrans = 1;
% centroids = centroids(1:sizeoftrans,:);
% Collectively writing all the tracked points in a single variable.
tracking_data = cat(2,centroids,theta,trans,theta_G,trans_G);            
    end
end
end % class