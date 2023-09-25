classdef OnlineTracking
    properties
        number_of_markers;
        centroids;
        PrevPt;
        P0;
        CurrPt;
        cent;
        tracking_data_centroid; % Added
        
        vread;
        numberOfFrames;
        vwrite;
        
        overlay_image;
        overlay;
        stop_radius;
        target_position;
        obstaclesBB;
        robo_centroid;
        trajectory_position;
        pixel_to_cm;
        robo_rotated;
    end
    methods
        function obj = OnlineTracking(params)
            obj.number_of_markers = params.number_of_markers;
            obj.PrevPt = [];
            obj.P0 = [];
            obj.CurrPt = [];
            obj.cent = [];
            obj.tracking_data_centroid = [];  % Added
%             obj.vread = params.vread;
            obj.numberOfFrames = 1;
            obj.vwrite = params.vwrite_tracked;
            obj.centroids = zeros(obj.numberOfFrames,3*obj.number_of_markers);
%             obj.overlay_image = params.overlay_img_cut;
            obj.overlay = params.overlay;
            obj.stop_radius = params.stop_radius;
            obj.target_position = params.target_position;
            obj.obstaclesBB = params.obstaclesBB;
            obj.robo_centroid = [];
            obj.trajectory_position = [];
            obj.pixel_to_cm = params.pixel_to_cm;
            obj.robo_rotated = [];
        end

        function [output] = tracking(obj,thisFrame,PrevPt,P0,robo_centroid,trajectory_position,k)
                
                obj.PrevPt = PrevPt;
                obj.P0 = P0;
                obj.robo_centroid = robo_centroid;
                obj.trajectory_position = trajectory_position;
%                 thisFrame = read(obj.vread,k);
                newim = createMaskhdblue(thisFrame);
                newim = bwareaopen(newim,25);
                newim = imfill(newim, 'holes');

                [labeledImage, numberOfRegions] = bwlabel(newim);

                count = 0;
                obj.cent = zeros(numberOfRegions,2);

                stats = regionprops(labeledImage, 'BoundingBox','Centroid','Area','EquivDiameter');
                 for rb = 1:numberOfRegions
                     count = count + 1;
                     obj.cent(count,:) = stats(rb).Centroid;
                     obj.cent(count,2) = 1080 - obj.cent(count,2) ;  % Correction for y-axis.
                 end

%                  obj.tracking_data_centroid(k,:) = [mean(obj.cent(:,1)) mean(obj.cent(:,2))];   % Added ANM
%                  tc(k,:) = obj.tracking_data_centroid(k,:);     
             
                  zc = zeros(size(obj.cent,1),1);
                  obj.cent = [obj.cent,zc];

                  if k == 1
                    obj.P0 = obj.cent;
                    output.P0 = obj.P0;
                    obj.PrevPt = obj.cent;
                    output.PrevPt = obj.PrevPt;
                    obj.centroids = data_logging(obj,k);
                    theta = zeros(1,9);
                    trans = zeros(1,3);
                    theta_G = zeros(1,9);
                    trans_G = zeros(1,3);
                    obj.robo_centroid(k,:) = [mean(obj.PrevPt(:,1)) mean(obj.PrevPt(:,2))];
                    output.robo_centroid = obj.robo_centroid;
                    output.robo_rotated = 0;
                    obj.robo_rotated = output.robo_rotated;
                    plot(obj,thisFrame,count,trajectory_position,k);
                  end

                  if k ~= 1

                     obj.CurrPt = nearest_neighbor(obj,count);                    

                     [Rot,T] = pose_estimation(obj,obj.CurrPt,obj.PrevPt,k);
                     theta(k,:) = reshape(Rot,[1,9]);
                     trans(k,:) = T';

                     [Rot,T] = pose_estimation(obj,obj.P0,obj.CurrPt,k);
                     theta_G(k,:) = reshape(Rot,[1,9]);
                     trans_G(k,:) = T';
                     output.robo_rotated = rotm2eul(Rot);

                     obj.PrevPt = obj.CurrPt;
                     obj.centroids = data_logging(obj,k);
                     
                     obj.robo_centroid(k,:) = [mean(obj.PrevPt(:,1)) mean(obj.PrevPt(:,2))];
                     output.robo_centroid = obj.robo_centroid;
                     obj.robo_rotated = output.robo_rotated;
                     plot(obj,thisFrame,count,trajectory_position,k);

                  end

            output.tracking_data = cat(2,obj.centroids,theta,trans,theta_G,trans_G);
%             close(obj.vwrite);
        end

        function centroids = nearest_neighbor(obj,count)      %Change the name of the centroid ->
            
            obj.CurrPt = zeros(obj.number_of_markers,3);
            if(count > obj.number_of_markers)
                for i = 1:obj.number_of_markers
                    for j = 1:count
                        X = [obj.PrevPt(i,:);obj.cent(j,:)];
                        d(j) = pdist(X,'euclidean');
                    end
                    [dmin,ind] = min(d);  
                    if(dmin < 15)
                        obj.CurrPt(i,:) = obj.cent(ind,:);
                    end
                end
            end
            if(count <= obj.number_of_markers)
                for i = 1:count
                    for j = 1:obj.number_of_markers
                        X = [obj.cent(i,:);obj.PrevPt(j,:)];
                        d(j) = pdist(X,'euclidean');
                    end
                    [dmin,ind] = min(d);
                    if(dmin < 15)
                        obj.CurrPt(ind,:) = obj.cent(i,:);
                    end
                end
            end
            
            clear d;
            TF = obj.CurrPt(:,1);  % Writing the 1st column of resrvd
            index = find(TF == 0);  % Finding those rows which is empty
            val = isempty(index);  % Checking whether the index is empty  
            
            if(val == 0)
                centroids = occlusion(obj,index);   %Change the name of the centroid ->
            end
            if(val~=0)
                centroids = obj.CurrPt;       %Change the name of the centroid ->
            end

        end 

        function centroids = occlusion(obj,index)   %Change the name of the centroid ->
            newPrevPt = obj.PrevPt;
            newP0 = obj.P0; 
            newPrevPt(index(1:size(index,1)),:) = 0;
            newP0(index(1:size(index,1)),:) = 0;
%             [Rot,T,~,~] = pose_estimation(newPrevPt,obj.CurrPt); % SE2 w.r.t previous frame
            [Rot,T] = pose_estimation(obj,newPrevPt,obj.CurrPt); % SE2 w.r.t previous frame
            for gg = 1:size(index,1)
                newPt = Rot*(obj.PrevPt(index(gg),:))' + T;
                obj.CurrPt(index(gg),:) = newPt;
            end
            centroids = obj.CurrPt;
        end

        function centroids = data_logging(obj,k)

            for i = 1:obj.number_of_markers
                obj.centroids(k,(3*i)-2:(3*i)) = obj.PrevPt(i,:);
                centroids = obj.centroids;
            end

        end

        function [Rot,T,theta,trans] = pose_estimation(obj,A,B,k)

            [Rot,T] = rigid_transform_3D(A',B');  % SE2 w.r.t previous frame
%             theta(k,:) = reshape(Rot,[1,9]);   % Changed ANM
%             trans(k,:) = T';

        end

        function plot(obj,thisFrame,count,trajectory_position,k)

            figure(10)
            imshow(thisFrame)
            set(gcf, 'Position',  [100, 100, 1000, 1000])
            hold on
            
            %Plot markers on the robot
            plot(obj.PrevPt(:,1),1080-obj.PrevPt(:,2),'g*','LineWidth',0.5,'MarkerSize',2)
%             plot(obj.tracking_data_centroids(k,1),1080-obj.tracking_data_centroid(k,2),'g*','LineWidth',0.5,'MarkerSize',2)
     
            %Plot centroid of the robot
            plot(obj.robo_centroid(:,1),1080-obj.robo_centroid(:,2),'c*','LineWidth',2.5,'MarkerSize',2)
            
            %Plot path planner trajectory
%             plot(obj.trajectory_position(1,:)*obj.pixel_to_cm,1080-(obj.trajectory_position(2,:)*obj.pixel_to_cm),'-r','LineWidth',2);
            plot(trajectory_position(1,:)*obj.pixel_to_cm,1080-(trajectory_position(2,:)*obj.pixel_to_cm),'-r','LineWidth',2);
            
            %Plot quiver
%             plot(obj.robo_centroid(k,1),1080-obj.robo_centroid(k,2),40*cos(obj.robo_rotated),10*sin(obj.robo_rotated),'LineWidth',2,'Color',"#D95319");
            
            
            % Bounding box
            for ii = 1:size(obj.obstaclesBB,1)
               h = rectangle('Position',obj.obstaclesBB(ii,:),'EdgeColor','y','LineWidth',3);
            end
            
            % Target/Goal position
            plot(obj.target_position(:,1),1080-obj.target_position(:,2),'r*','LineWidth',0.7,'MarkerSize',5)
            viscircles([obj.target_position(:,1) 1080-obj.target_position(:,2)],obj.stop_radius,'LineStyle','--');

            caption = sprintf('%d blobs found in frame %d', count, k);
            title(caption, 'FontSize', 20);
            
            %Legends
            legendnames = {'Robot markers','Robot path traced','Path Planner Path','Goal position'};
            legend(legendnames);
            
            axis on;
            hold off
            pframe = getframe(gcf);
            writeVideo(obj.vwrite,pframe);

        end  
%         
%         function plot(obj,thisFrame,count,k)
% % 
%             figure(10)
%             imshow(thisFrame)
%             set(gcf, 'Position',  [100, 100, 1000, 1000])
%             hold on
% %             plot(obj.PrevPt(k,1),1080-obj.PrevPt(k,2),'g*','LineWidth',0.5,'MarkerSize',2)
%             plot(obj.tracking_data_centroid(k,1),1080-obj.tracking_data_centroid(k,2),'g*','LineWidth',0.5,'MarkerSize',2)
%             caption = sprintf('%d blobs found in frame #%d 0f %d', count, k, obj.numberOfFrames);
%             title(caption, 'FontSize', 20);
%             axis on;
% %             hold off
%             ovly_img_hdl = imagesc(obj.overlay_image);              % overlay (reference) image
%             set(ovly_img_hdl, 'AlphaData', 0.4);    % set overlaid image alpha
%             hold off;
%             pframe = getframe(gcf);
%             writeVideo(obj.vwrite,pframe);
% 
%         end 

    end

end