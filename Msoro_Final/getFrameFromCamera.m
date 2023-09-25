function getFrameFromCamera(P,L,distortion,vid_name)
    cam1 = webcam(1);
    cam1.Resolution = '1920x1080';
    pause(1);
    vwrite = VideoWriter(vid_name,'MPEG-4');
    open(vwrite);
    ii = 0;
    cam_length = 0;
    while true
        
        [img,ts] = snapshot(cam1);
        img = undistortImage(img,distortion.cameraParams);
        ii = ii +1;
        writeVideo(vwrite,img);
        cam_data.img = img;
        cam_data.time_stamp = ts;
        if ii == 1 || mod(ii,10) == 0
        cam_length = cam_length + 1;
        send(P,cam_data);
        send(L,cam_length);
        end
%         cam_data.k = cam_data.k + 1;
%         pause(1/freq);
    end
end