function varargout = mrtgui(varargin)
% MRTGUI MATLAB code for mrtgui.fig
%      MRTGUI, by itself, creates a new MRTGUI or raises the existing
%      singleton*.
%
%      H = MRTGUI returns the handle to a new MRTGUI or the handle to
%      the existing singleton*.
%
%      MRTGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MRTGUI.M with the given input arguments.
%
%      MRTGUI('Property','Value',...) creates a new MRTGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before mrtgui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to mrtgui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mrtgui

% Last Modified by GUIDE v2.5 13-Jul-2018 16:24:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mrtgui_OpeningFcn, ...
                   'gui_OutputFcn',  @mrtgui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before mrtgui is made visible.
function mrtgui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mrtgui (see VARARGIN)

% Choose default command line output for mrtgui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes mrtgui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = mrtgui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
global videoFrameGray  bbox bbox2 bbox3 bbox4

% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
       if handles.rotate==1
        videoFrameGray=videoFrameGray';
       end
    imshow(videoFrameGray);title('choose the face')
   bbox(1, :)=round(getPosition(imrect));
   AnnotatedImage=insertObjectAnnotation(videoFrameGray,'rectangle',bbox(1, :),'face');
    
   imshow(AnnotatedImage);title('choose an eye')
    bbox2=round(getPosition(imrect));
   AnnotatedImage=insertObjectAnnotation(AnnotatedImage,'rectangle',bbox2,'eye');
    
   imshow(AnnotatedImage);title('choose the nose');
    bbox3=round(getPosition(imrect));
   AnnotatedImage=insertObjectAnnotation(AnnotatedImage,'rectangle',bbox3,'nose');
   
   imshow(AnnotatedImage);title('choose the mouth');
   bbox4=round(getPosition(imrect));
   AnnotatedImage=insertObjectAnnotation(AnnotatedImage,'rectangle',bbox4,'mouth');



% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global videoFrameGray  v frame metadata pointTracker  pointTracker2 pointTracker3 pointTracker4
global videoFrame frameCount runLoop  numPts videoPlayer frame2 zz FileName
LoadingMessage='wait for the file to be loaded....';
set(handles.text3, 'string',LoadingMessage);
[FileName,PathName,FilterIndex] = uigetfile('*.*','flir','Browse Your Flir file');
t=[PathName,FileName];
v = FlirMovieReader(t);
[frame, metadata] = step(v);
frame1 = im2double(frame);
frame2 = imadjust(frame1);

%initialazing
 zz=1;

%rotation
handles.rotate=0;
guidata(hObject,handles);
handles = guidata(hObject);

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
    set(handles.text2, 'string',t);
    LoadingMessage='the file is loaded!';
    set(handles.text3, 'string',LoadingMessage);




    
    
    
% --- Executes on button press in pushbutton4.

function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global videoFrameGray  v frame metadata pointTracker  pointTracker2 pointTracker3 pointTracker4
global videoFrame frameCount  numPts videoPlayer  bbox bbox2 bbox3 bbox4 zz FileName
global numPts2 numPts3 numPts4
numPts2=0; numPts3=0; numPts4=0; numPts=0;
v.unit = 'temperatureFactory';
    [frame, metadata] = step(v);
    vtemp{zz}=frame;
     v.unit='counts';

    flag=1;
    handles.stop=1;
guidata(hObject,handles)
handles = guidata(hObject);
     
while ~isDone(v)&& handles.stop
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
    if handles.rotate==1
        videoFrameGray=videoFrameGray';
        videoFrame=videoFrame';
        vtemp{zz}=vtemp{zz}';
    end
    
    if numPts < 10 || numPts2 < 10 || numPts3 < 10 || numPts4 < 10

        if ~isempty(bbox)&flag==1
            %to avoid error when there is not enogh points to track
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
            %videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
            
            
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
            %videoFrame = insertMarker(videoFrame, xyPoints2, '+', 'Color', 'white');
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
            %videoFrame = insertMarker(videoFrame, xyPoints3, '+', 'Color', 'white');
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
            %videoFrame = insertMarker(videoFrame, xyPoints4, '+', 'Color', 'white');
        else
           hintflag=1;
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
        %stop button controll

        if numPts >= 10 && numPts2 >= 10 && numPts3 >= 10 && numPts4 >= 10
            
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
            
            % Reset the points.4
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
   %meanv(zz)=mean(mean(vtemp{zz}(min(bboxPoints(:,2)):max(bboxPoints(:,2)),min(bboxPoints(:,1)):max(bboxPoints(:,1)))));
   %show the average temprature of the box
   %STR=sprintf('left eye temp=%2.2f',meanv(zz));
   videoFrame=insertObjectAnnotation(videoFrame, 'rectangle', [min(bboxPoints(:,1))  min(bboxPoints(:,2))  xboss yboss],'1');
   %% seccond ROI
   
    % Display the annotated video frame using the video player object.
    % adjast the coordination of the box
    xboss2=size([min(bboxPoints2(:,1)):max(bboxPoints2(:,1))]);
    xboss2=xboss2(2);
    yboss2=size([min(bboxPoints2(:,2)):max(bboxPoints2(:,2))]);
   yboss2=yboss2(2);
   bboxPoints2=round(bboxPoints2);
   %meanv2(zz)=mean(mean(vtemp{zz}(min(bboxPoints2(:,2)):max(bboxPoints2(:,2)),min(bboxPoints2(:,1)):max(bboxPoints2(:,1)))));
   %show the average temprature of the box
   %STR2=sprintf('right eye temp=%2.2f',meanv2(zz));
   videoFrame=insertObjectAnnotation(videoFrame, 'rectangle', [min(bboxPoints2(:,1))  min(bboxPoints2(:,2))  xboss2 yboss2],'2');
   %%  %% 3rd ROI
   
    % Display the annotated video frame using the video player object.
    % adjast the coordination of the box
    xboss3=size([min(bboxPoints3(:,1)):max(bboxPoints3(:,1))]);
    xboss3=xboss3(2);
    yboss3=size([min(bboxPoints3(:,2)):max(bboxPoints3(:,2))]);
   yboss3=yboss3(2);
   bboxPoints3=round(bboxPoints3);
   %meanv3(zz)=mean(mean(vtemp{zz}(min(bboxPoints3(:,2)):max(bboxPoints3(:,2)),min(bboxPoints3(:,1)):max(bboxPoints3(:,1)))));
   %show the average temprature of the box
   %STR3=sprintf('nose temp=%2.2f',meanv3(zz));
   videoFrame=insertObjectAnnotation(videoFrame, 'rectangle', [min(bboxPoints3(:,1))  min(bboxPoints3(:,2))  xboss3 yboss3],'3');
    %%
    %%  4th ROI
   
    % Display the annotated video frame using the video player object.
    % adjast the coordination of the box
    xboss4=size([min(bboxPoints4(:,1)):max(bboxPoints4(:,1))]);
    xboss4=xboss4(2);
    yboss4=size([min(bboxPoints4(:,2)):max(bboxPoints4(:,2))]);
   yboss4=yboss4(2);
   bboxPoints4=round(bboxPoints4);
  % meanv4(zz)=mean(mean(vtemp{zz}(min(bboxPoints4(:,2)):max(bboxPoints4(:,2)),min(bboxPoints4(:,1)):max(bboxPoints4(:,1)))));
   %show the average temprature of the box
   %STR4=sprintf('mouth temp=%2.2f',meanv4(zz));
   videoFrame=insertObjectAnnotation(videoFrame, 'rectangle', [min(bboxPoints4(:,1))  min(bboxPoints4(:,2))  xboss4 yboss4],'4');
    %%
    %step(videoPlayer, videoFrame);
    imshow(videoFrame)
    drawnow
    directoryoldtxt=dir('C:\Users\vc-lab\Desktop\dataset\txt1');
    numoldtxt=size(directoryoldtxt,1);
    directoryoldjpg=dir('C:\Users\vc-lab\Desktop\dataset\jpg');
    numoldjpg=size(directoryoldjpg,1);
%     directoryold2=dir('C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\right eye');
%     numold2=size(directoryold2,1);
%     directoryold3=dir('C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\nose');
%     numold3=size(directoryold3,1);
%     directoryold4=dir('C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\mouth');
%     numold4=size(directoryold4,1);
%     
    %address=sprintf('eye%d.jpg',zz);
%     address=sprintf('left eye%d.jpg',numold);
%     address2=sprintf('right eye%d.jpg',numold2);
%     address3=sprintf('nose%d.jpg',numold3);
%     address4=sprintf('mouth%d.jpg',numold4);
    
    addresstxt=sprintf('bbox%d.txt',numoldtxt);
      addtxt='C:\Users\vc-lab\Desktop\dataset\txt1\';
      addjpg='C:\Users\vc-lab\Desktop\dataset\jpg\';
%      add='C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\left eye\';
%      add2='C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\right eye\';
%      add3='C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\nose\';
%      add4='C:\Users\Navid\Desktop\NAVID\work\Deutschland\dataset\mouth\';
     
     fileID = fopen([addtxt addresstxt],'w');
     address=sprintf('%d.jpg',numoldjpg);
   
    fprintf(fileID,' %d %d %d %d\r\n',min(bboxPoints(:,1)),  min(bboxPoints(:,2)),  xboss, yboss);
    fprintf(fileID,' %d %d %d %d\r\n',min(bboxPoints2(:,1)),  min(bboxPoints2(:,2)),  xboss2, yboss2);
    fprintf(fileID,' %d %d %d %d\r\n',min(bboxPoints3(:,1)),min(bboxPoints3(:,2)),  xboss3, yboss3);
    fprintf(fileID,' %d %d %d %d\r\n',min(bboxPoints4(:,1)),min(bboxPoints4(:,2)) ,xboss4, yboss4);
    fprintf(fileID,' filename: %s Framecount: %d',FileName,metadata.FrameNumber);
    fclose(fileID);
    
    imwrite(vtemp{zz}*1000,[addjpg address],'Mode','lossless','BitDepth',16);

    
    %imwrite(videoFrame(min(bboxPoints(:,2)):max(bboxPoints(:,2)),min(bboxPoints(:,1)):max(bboxPoints(:,1))),[add address]);
%    imwrite(vtemp{zz}(min(bboxPoints(:,2)):max(bboxPoints(:,2)),min(bboxPoints(:,1)):max(bboxPoints(:,1)))*1000,[add address],'Mode','lossless','BitDepth',16);
%      imwrite(vtemp{zz}(min(bboxPoints2(:,2)):max(bboxPoints2(:,2)),min(bboxPoints2(:,1)):max(bboxPoints2(:,1)))*1000,[add2 address2],'Mode','lossless','BitDepth',16);
%      imwrite(vtemp{zz}(min(bboxPoints3(:,2)):max(bboxPoints3(:,2)),min(bboxPoints3(:,1)):max(bboxPoints3(:,1)))*1000,[add3 address3],'Mode','lossless','BitDepth',16);
%      imwrite(vtemp{zz}(min(bboxPoints4(:,2)):max(bboxPoints4(:,2)),min(bboxPoints4(:,1)):max(bboxPoints4(:,1)))*1000,[add4 address4],'Mode','lossless','BitDepth',16);
    
    % Check whether the video player window has been closed.
    %runLoop = isOpen(videoPlayer);

handles = guidata(hObject);
drawnow
end
    if hintflag==1
       lostROImessage='the points are lost choose them again';
       set(handles.text3, 'string',lostROImessage);
       hintflag=0; 
    else
    endingMessage='this file has reached its end please load the next file';
    set(handles.text3, 'string',endingMessage);
    end
    
 

    
    
    
    

% --- Executes on button press in pushbuttoncontinue.
function pushbuttoncontinue_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttoncontinue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global videoFrameGray v videoFrame frameCount
handles.stop=1;
guidata(hObject,handles)
handles = guidata(hObject);
while ~isDone(v)&& handles.stop

    
        
    videoFrame = im2double(step(v));
    videoFrame = imadjust(videoFrame);
    videoFrameGray = videoFrame;
   if handles.rotate==1
       videoFrameGray=videoFrameGray';
   end
    imshow(videoFrameGray);
    handles = guidata(hObject);
    drawnow
    
end
endingMessage='this file has reached its end please load the next file';
set(handles.text2, 'string',endingMessage);


% --- Executes on button press in pushbuttonstop.
function pushbuttonstop_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonstop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.stop = 0;
guidata(hObject,handles);


% --- Executes on button press in pushbuttonRotate.
function pushbuttonRotate_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonRotate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global videoFrameGray
handles.rotate=1;
guidata(hObject,handles);
videoFrameGray=videoFrameGray';
imshow(videoFrameGray);
videoFrameGray=videoFrameGray';


% --- Executes on button press in pushbuttonpreview.
function pushbuttonpreview_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonpreview (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global videoFrameGray
imshow(videoFrameGray);
