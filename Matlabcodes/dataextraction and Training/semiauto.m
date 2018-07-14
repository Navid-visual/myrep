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


%% Detection and Tracking
% Capture and process video frames from the video in a loop to detect and
% track a face. The loop will run until the video player
% window is closed.
% setup video player
videoPlayer = vision.DeployableVideoPlayer('Location', [100 100]);
videoPlayer.FrameRate = 30;
videoPlayer.Size = 'Full-screen';


pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

runLoop = true;
numPts = 0;
frameCount = 0;

    videoFrame = im2double(step(v));
    videoFrame = imadjust(videoFrame);
    videoFrameGray = videoFrame;
    %% 
    % manualy set the ROI
    figure; imshow(videoFrameGray);
   bbox(1, :)=round(getPosition(imrect));
  
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
        else
            break
        end
 
    else
        % Tracking mode.
        [xyPoints, isFound] = step(pointTracker, videoFrameGray);
        visiblePoints = xyPoints(isFound, :);
        oldInliers = oldPoints(isFound, :);
                
        numPts = size(visiblePoints, 1);       
        
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
   STR=sprintf('temp=%2.2f',meanv(zz));
   videoFrame=insertObjectAnnotation(videoFrame, 'rectangle', [min(bboxPoints(:,1))  min(bboxPoints(:,2))  xboss yboss], STR);
    
    step(videoPlayer, videoFrame);
    %find the number of old images to avoind overwriting them
    directoryold=dir('C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset');
    numold=size(directoryold,1);
    %address=sprintf('eye%d.jpg',zz);
    address2=sprintf('eyetemp%d.jpg',zz+numold);
    add='C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\';
    imwrite(videoFrame(min(bboxPoints(:,2)):max(bboxPoints(:,2)),min(bboxPoints(:,1)):max(bboxPoints(:,1))),[add address2]);
  %  imwrite(vtemp{zz}(min(bboxPoints(:,2)):max(bboxPoints(:,2)),min(bboxPoints(:,1)):max(bboxPoints(:,1)))*1000,[add address2],'Mode','lossless','BitDepth',16);
    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
  
        

      
end
%semiauto