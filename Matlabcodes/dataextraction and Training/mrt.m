clear;



%% bring in the first frame from the file
[FileName,PathName,FilterIndex] = uigetfile('*.*','flir','Browse Your Flir file');
t=[PathName,FileName];
v = FlirMovieReader(t);
[frame, metadata] = step(v);
frame1 = im2double(frame);
frame2 = imadjust(frame1);

%initialazing
 zz=1;
meanv=zeros(1,1000);
meanv2=zeros(1,1000);
meanv3=zeros(1,1000);
meanv4=zeros(1,1000);


%% Detection and Tracking
% Capture and process video frames from the video in a loop to detect and
% track a face. The loop will run until the video player
% window is closed.
% setup video player
videoPlayer = vision.DeployableVideoPlayer('Location', [100 100]);
videoPlayer.FrameRate = 8;
videoPlayer.Size = 'Full-screen';


pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
pointTracker2= vision.PointTracker('MaxBidirectionalError', 2);
pointTracker3= vision.PointTracker('MaxBidirectionalError', 2);
pointTracker4= vision.PointTracker('MaxBidirectionalError', 2);
runLoop = true;
numPts = 0;
frameCount = 0;

    videoFrame = im2double(step(v));
    videoFrame = imadjust(videoFrame);
    videoFrameGray = videoFrame;
    %% 
    % manualy set the ROI
    figure; imshow(videoFrameGray);title('choose the left eye')
   bbox(1, :)=round(getPosition(imrect));
   
       figure; imshow(videoFrameGray);title('choose the right eye')
   bbox2=round(getPosition(imrect));
   
       figure; imshow(videoFrameGray);title('choose the nose');
   bbox3=round(getPosition(imrect));
   
       figure; imshow(videoFrameGray);title('choose the mouth');
   bbox4=round(getPosition(imrect));

   

   %% 
 
     v.unit = 'temperatureFactory';
    [frame, metadata] = step(v);
    vtemp{zz}=frame;
     v.unit='counts';

    flag=1;
while ~isDone(v)
    %read the temprature of each frame and save it.
    zz=zz+1;
    v.unit = 'temperatureFactory';
    [frame, metadata] = step(v);
    vtemp{zz}=frame;
     v.unit='counts';
    
    % Get the next frame.
    videoFrame = im2double(step(v));
    videoFrame = imadjust(videoFrame);
    videoFrameGray = videoFrame;
    frameCount = frameCount + 1;
    
    if numPts < 10

        if ~isempty(bbox)&flag==1
            flag=0;
            % Find corner points inside the detected region.
            points = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox(1, :));
           
            % Re-initialize the point tracker.
            xyPoints = points.Location;
            numPts = size(xyPoints,1);
            release(pointTracker);
            initialize(pointTracker, xyPoints, videoFrameGray);
            
            % Save a copy of the points.
            oldPoints = xyPoints;
            
            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints = bbox2points(bbox(1, :));  
            

            
            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
            
            
            %% 
            %seccond ROI tracking...........................
                        
            % Find corner points inside the detected region.
            points2= detectMinEigenFeatures(videoFrameGray, 'ROI', bbox2);
           
            % Re-initialize the point tracker.
            xyPoints2 = points2.Location;
            numPts2 = size(xyPoints2,1);
            release(pointTracker2);
            initialize(pointTracker2, xyPoints2, videoFrameGray);
            
            % Save a copy of the points.
            oldPoints2 = xyPoints2;
            
            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints2= bbox2points(bbox2);  
            

            
            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints2, '+', 'Color', 'white');
            %% 
                        %% 
            %3rd ROI tracking...........................
                        
            % Find corner points inside the detected region.
            points3= detectMinEigenFeatures(videoFrameGray, 'ROI', bbox3);
           
            % Re-initialize the point tracker.
            xyPoints3 = points3.Location;
            numPts3 = size(xyPoints3,1);
            release(pointTracker3);
            initialize(pointTracker3, xyPoints3, videoFrameGray);
            
            % Save a copy of the points.
            oldPoints3 = xyPoints3;
            
            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints3= bbox2points(bbox3);  
            

            
            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints3, '+', 'Color', 'white');
            %%                        %% 
            %4th ROI tracking...........................
                        
            % Find corner points inside the detected region.
            points4= detectMinEigenFeatures(videoFrameGray, 'ROI', bbox4);
           
            % Re-initialize the point tracker.
            xyPoints4 = points4.Location;
            numPts4 = size(xyPoints4,1);
            release(pointTracker4);
            initialize(pointTracker4, xyPoints4, videoFrameGray);
            
            % Save a copy of the points.
            oldPoints4 = xyPoints4;
            
            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints4= bbox2points(bbox4);  
            

            
            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints4, '+', 'Color', 'white');
        else
           
            break
        end
 
    else
        % Tracking mode.
        [xyPoints, isFound] = step(pointTracker, videoFrameGray);
        visiblePoints = xyPoints(isFound, :);
        oldInliers = oldPoints(isFound, :);
        numPts = size(visiblePoints, 1);       
        
        %% 
        %seccond ROI
                [xyPoints2, isFound2] = step(pointTracker2, videoFrameGray);
        visiblePoints2 = xyPoints2(isFound2, :);
        oldInliers2 = oldPoints2(isFound2, :);
        numPts2 = size(visiblePoints2, 1); 
        
        
        %%
                %% 
        %3rd ROI
                [xyPoints3, isFound3] = step(pointTracker3, videoFrameGray);
        visiblePoints3 = xyPoints3(isFound3, :);
        oldInliers3 = oldPoints3(isFound3, :);
        numPts3 = size(visiblePoints3, 1); 
        %%        4th ROI
                [xyPoints4, isFound4] = step(pointTracker4, videoFrameGray);
        visiblePoints4 = xyPoints4(isFound4, :);
        oldInliers4 = oldPoints4(isFound4, :);
        numPts4 = size(visiblePoints4, 1); 
        %% 
        
        if numPts >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
                oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);            
            
            % Apply the transformation to the bounding box.
            bboxPoints = transformPointsForward(xform, bboxPoints);   
            


            % Display tracked points.
            %videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');
            
            % Reset the points.
            oldPoints = visiblePoints;
            setPoints(pointTracker, oldPoints);
            
            %% seccond ROI
                        % Estimate the geometric transformation between the old points
            % and the new points.
            [xform2, oldInliers2, visiblePoints2] = estimateGeometricTransform(...
                oldInliers2, visiblePoints2, 'similarity', 'MaxDistance', 4);            
            
            % Apply the transformation to the bounding box.
            bboxPoints2 = transformPointsForward(xform2, bboxPoints2);   
            


            % Display tracked points.
            %videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');
            
            % Reset the points.
            oldPoints2 = visiblePoints2;
            setPoints(pointTracker2, oldPoints2);
            %%   %% 3rd ROI
                        % Estimate the geometric transformation between the old points
            % and the new points.
            [xform3, oldInliers3, visiblePoints3] = estimateGeometricTransform(...
                oldInliers3, visiblePoints3, 'similarity', 'MaxDistance', 4);            
            
            % Apply the transformation to the bounding box.
            bboxPoints3 = transformPointsForward(xform3, bboxPoints3);   
            


            % Display tracked points.
            %videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');
            
            % Reset the points.
            oldPoints3 = visiblePoints3;
            setPoints(pointTracker3, oldPoints3);
                     %%   4th ROI
                        % Estimate the geometric transformation between the old points
            % and the new points.
            [xform4, oldInliers4, visiblePoints4] = estimateGeometricTransform(...
                oldInliers4, visiblePoints4, 'similarity', 'MaxDistance', 4);            
            
            % Apply the transformation to the bounding box.
            bboxPoints4 = transformPointsForward(xform4, bboxPoints4);   
            


            % Display tracked points.
            %videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');
            
            % Reset the points.
            oldPoints4 = visiblePoints4;
            setPoints(pointTracker4, oldPoints4);
        end

    end
        
    % Display the annotated video frame using the video player object.
    % adjast the coordination of the box
    xboss=size([min(bboxPoints(:,1)):max(bboxPoints(:,1))]);
    xboss=xboss(2);
    yboss=size([min(bboxPoints(:,2)):max(bboxPoints(:,2))]);
   yboss=yboss(2);
   bboxPoints=round( bboxPoints);
   meanv(zz)=mean(mean(vtemp{zz}(min(bboxPoints(:,2)):max(bboxPoints(:,2)),min(bboxPoints(:,1)):max(bboxPoints(:,1)))));
   %show the average temprature of the box
   STR=sprintf('left eye temp=%2.2f',meanv(zz));
   videoFrame=insertObjectAnnotation(videoFrame, 'rectangle', [min(bboxPoints(:,1))  min(bboxPoints(:,2))  xboss yboss], STR);
   %% seccond ROI
   
    % Display the annotated video frame using the video player object.
    % adjast the coordination of the box
    xboss2=size([min(bboxPoints2(:,1)):max(bboxPoints2(:,1))]);
    xboss2=xboss2(2);
    yboss2=size([min(bboxPoints2(:,2)):max(bboxPoints2(:,2))]);
   yboss2=yboss2(2);
   bboxPoints2=round(bboxPoints2);
   meanv2(zz)=mean(mean(vtemp{zz}(min(bboxPoints2(:,2)):max(bboxPoints2(:,2)),min(bboxPoints2(:,1)):max(bboxPoints2(:,1)))));
   %show the average temprature of the box
   STR2=sprintf('right eye temp=%2.2f',meanv2(zz));
   videoFrame=insertObjectAnnotation(videoFrame, 'rectangle', [min(bboxPoints2(:,1))  min(bboxPoints2(:,2))  xboss2 yboss2], STR2);
   %%  %% 3rd ROI
   
    % Display the annotated video frame using the video player object.
    % adjast the coordination of the box
    xboss3=size([min(bboxPoints3(:,1)):max(bboxPoints3(:,1))]);
    xboss3=xboss3(2);
    yboss3=size([min(bboxPoints3(:,2)):max(bboxPoints3(:,2))]);
   yboss3=yboss3(2);
   bboxPoints3=round(bboxPoints3);
   meanv3(zz)=mean(mean(vtemp{zz}(min(bboxPoints3(:,2)):max(bboxPoints3(:,2)),min(bboxPoints3(:,1)):max(bboxPoints3(:,1)))));
   %show the average temprature of the box
   STR3=sprintf('nose temp=%2.2f',meanv3(zz));
   videoFrame=insertObjectAnnotation(videoFrame, 'rectangle', [min(bboxPoints3(:,1))  min(bboxPoints3(:,2))  xboss3 yboss3], STR3);
    %%
    %%  4th ROI
   
    % Display the annotated video frame using the video player object.
    % adjast the coordination of the box
    xboss4=size([min(bboxPoints4(:,1)):max(bboxPoints4(:,1))]);
    xboss4=xboss4(2);
    yboss4=size([min(bboxPoints4(:,2)):max(bboxPoints4(:,2))]);
   yboss4=yboss4(2);
   bboxPoints4=round(bboxPoints4);
   meanv4(zz)=mean(mean(vtemp{zz}(min(bboxPoints4(:,2)):max(bboxPoints4(:,2)),min(bboxPoints4(:,1)):max(bboxPoints4(:,1)))));
   %show the average temprature of the box
   STR4=sprintf('mouth temp=%2.2f',meanv4(zz));
   videoFrame=insertObjectAnnotation(videoFrame, 'rectangle', [min(bboxPoints4(:,1))  min(bboxPoints4(:,2))  xboss4 yboss4], STR4);
    %%
    step(videoPlayer, videoFrame);
    %find the number of old images to avoind overwriting them
    directoryold=dir('C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\left eye');
    numold=size(directoryold,1);
    directoryold2=dir('C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\right eye');
    numold2=size(directoryold2,1);
    directoryold3=dir('C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\nose');
    numold3=size(directoryold3,1);
    directoryold4=dir('C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\mouth');
    numold4=size(directoryold4,1);
    
    %address=sprintf('eye%d.jpg',zz);
    address=sprintf('left eye%d.jpg',numold);
    address2=sprintf('right eye%d.jpg',numold2);
    address3=sprintf('nose%d.jpg',numold3);
    address4=sprintf('mouth%d.jpg',numold4);
    
     add='C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\left eye\';
     add2='C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\right eye\';
     add3='C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\nose\';
     add4='C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\mouth\';
    
    %imwrite(videoFrame(min(bboxPoints(:,2)):max(bboxPoints(:,2)),min(bboxPoints(:,1)):max(bboxPoints(:,1))),[add address]);
     imwrite(vtemp{zz}(min(bboxPoints(:,2)):max(bboxPoints(:,2)),min(bboxPoints(:,1)):max(bboxPoints(:,1)))*1000,[add address],'Mode','lossless','BitDepth',16);
     imwrite(vtemp{zz}(min(bboxPoints2(:,2)):max(bboxPoints2(:,2)),min(bboxPoints2(:,1)):max(bboxPoints2(:,1)))*1000,[add2 address2],'Mode','lossless','BitDepth',16);
     imwrite(vtemp{zz}(min(bboxPoints3(:,2)):max(bboxPoints3(:,2)),min(bboxPoints3(:,1)):max(bboxPoints3(:,1)))*1000,[add3 address3],'Mode','lossless','BitDepth',16);
     imwrite(vtemp{zz}(min(bboxPoints4(:,2)):max(bboxPoints4(:,2)),min(bboxPoints4(:,1)):max(bboxPoints4(:,1)))*1000,[add4 address4],'Mode','lossless','BitDepth',16);
    
    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
  
       
      
end
   close all

mrt