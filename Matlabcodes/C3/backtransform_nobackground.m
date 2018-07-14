function varargout = backtransform(varargin)
% BACKTRANSFORM MATLAB code for backtransform.fig
%      BACKTRANSFORM, by itself, creates a new BACKTRANSFORM or raises the existing
%      singleton*.
%
%      H = BACKTRANSFORM returns the handle to a new BACKTRANSFORM or the handle to
%      the existing singleton*.
%
%      BACKTRANSFORM('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BACKTRANSFORM.M with the given input arguments.
%
%      BACKTRANSFORM('Property','Value',...) creates a new BACKTRANSFORM or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before backtransform_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to backtransform_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help backtransform

% Last Modified by GUIDE v2.5 25-Jun-2018 13:55:15

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @backtransform_OpeningFcn, ...
                   'gui_OutputFcn',  @backtransform_OutputFcn, ...
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


% --- Executes just before backtransform is made visible.
function backtransform_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to backtransform (see VARARGIN)
    axes(handles.axes1);
    disconnect=imread('disconnected1.jpg');
    imshow(disconnect)
% Choose default command line output for backtransform
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes backtransform wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = backtransform_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbuttonstart.
function pushbuttonstart_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonstart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();

% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam('FLIR');

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object. 
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

%% Detection and Tracking
% Capture and process video frames from the webcam in a loop to detect and
% track a face. The loop will run for 400 frames or until the video player
% window is closed.

runLoop = true;
numPts = 0;
frameCount = 0;
a=1;
 handles.stop=1;
 guidata(hObject,handles)
handles = guidata(hObject);
MIN=handles.min;
MAX=handles.max;
 while handles.stop
    
    % Get the next frame.
    videoFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
 
    frameCount = frameCount + 1;
    %temprature transformation
     temp=videoFrameGray;
     temp=im2double(temp);
     T=(MAX-MIN)/(max(max(temp))-min(min(temp)));
     temp=temp*T+26.4;
     videoFrame= videoFrame.*temp(30<temp);

    if numPts < 10
        % Detection mode.
        bbox = faceDetector.step(videoFrameGray);
       
        if ~isempty(bbox)
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
            
            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4] 
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);
            
            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);
            
            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
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
            
            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4] 
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);            
            
            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);
            
            % Display tracked points.
            videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');
            
            % Reset the points.
            oldPoints = visiblePoints;
            setPoints(pointTracker, oldPoints);
            bboxPoints=round(bboxPoints);
            %calculate the average temprature of the face
             tempbbox=[min(bboxPoints(:,1)) min(bboxPoints(:,2)) max(bboxPoints(:,1))-min(bboxPoints(:,1)) max(bboxPoints(:,2))-min(bboxPoints(:,2))];
             cx=tempbbox(1)+tempbbox(3)/2;
             cy=tempbbox(2)+tempbbox(4)/2;
             cr=tempbbox(3)/2;
            faceaverage=mean(mean(temp(max(min(bboxPoints(:,2)),1):min(max(bboxPoints(:,2)),240),max(min(bboxPoints(:,1)),1):min(max(bboxPoints(:,1)),320))));
            temperature=sprintf('face temprature=%2.2f',faceaverage);
            videoFrame=insertObjectAnnotation( videoFrame, 'circle', [cx cy cr], temperature);
        end

    end
        
    % Display the annotated video frame using the video player object.
    %step(videoPlayer, videoFrame);
   handles = guidata(hObject);
    % Check whether the video player window has been closed.
    %runLoop = isOpen(videoPlayer);
    axes(handles.axes1);
    imshow(videoFrame)
    drawnow
end
    axes(handles.axes1);
    disconnect=imread('disconnected1.jpg');
    imshow(disconnect)
% Clean up.
clear cam;
release(videoPlayer);
release(pointTracker);
release(faceDetector);

% --- Executes on button press in pushbuttonstop.
function pushbuttonstop_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonstop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.stop = 0;
guidata(hObject,handles);



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonset.
function pushbuttonset_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 handles.min= str2num(get(handles.edit1,'string'));
 handles.max=str2num(get(handles.edit2,'string'));
 guidata(hObject,handles);
 