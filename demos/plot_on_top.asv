close all; clc; clear all

fullFileName = 'data/visualtracking/Gait E_2.mp4'
% Instantiate a video reader object for this video.
  videoObject = VideoReader(fullFileName);

% Creating object for video writing
  %vwrite = VideoWriter(video_write_fname,'MPEG-4');
  %open(vwrite);
% Setup other parameters
numberOfFrames = videoObject.NumberOfFrame;

figure(1)
for k = 1 : numberOfFrames   
    thisFrame = read(videoObject,k);
     imshow(thisFrame)
%     set(gcf, 'Position',  [100, 100, 750, 400])
    set(gcf, 'Position',  [100, 100, 1000, 1000])
%     set(gca,'XLim',[0 700],'YLim',[0 700]);
    hold on
%     plot(PrevPt(:,1),PrevPt(:,2),'g*','LineWidth',0.5,'MarkerSize',2)
% %     mean_centroid = mean(PrevPt);
%     % plot([PrevPt(1,1) PrevPt(2,1)], [PrevPt(1,2) PrevPt(2,2)],'LineWidth',1.5)
%     % h = animatedline('LineStyle','-','Color','b','LineWidth',1.5);
%     caption = sprintf('%d blobs found in frame #%d 0f %d', count, k, numberOfFrames);
%     title(caption, 'FontSize', 20);
%     h = animatedline('Marker','o','Color','b','MarkerSize',1,'MarkerFaceColor','b');
%     h2 = animatedline('LineStyle','-','Color','r','LineWidth',1.5);
%     addpoints(h,mean_centroid(1,1),mean_centroid(1,2))
%     drawnow;
    axis on
    hold off
end
