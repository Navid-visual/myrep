function varargout = GUIC3(varargin)
% GUIC3 MATLAB code for GUIC3.fig
%      GUIC3, by itself, creates a new GUIC3 or raises the existing
%      singleton*.
%
%      H = GUIC3 returns the handle to a new GUIC3 or the handle to
%      the existing singleton*.
%
%      GUIC3('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUIC3.M with the given input arguments.
%
%      GUIC3('Property','Value',...) creates a new GUIC3 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUIC3_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUIC3_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUIC3

% Last Modified by GUIDE v2.5 14-Jun-2018 11:59:44

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUIC3_OpeningFcn, ...
                   'gui_OutputFcn',  @GUIC3_OutputFcn, ...
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


% --- Executes just before GUIC3 is made visible.
function GUIC3_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUIC3 (see VARARGIN)

% Choose default command line output for GUIC3
handles.output = hObject;
guidata(hObject, handles);
fig = imread('Header.png');
axes(handles.axes2);
imshow(fig);
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUIC3 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUIC3_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbuttonsearch.
function pushbuttonsearch_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonsearch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%AddAssembly Flir.Atlas.Live.dll
atPath = getenv('FLIR_Atlas_MATLAB');
atLive = strcat(atPath,'Flir.Atlas.Live.dll');
asmInfo = NET.addAssembly(atLive);
%init camera discovery
test = Flir.Atlas.Live.Discovery.Discovery;
% search for cameras for 10 seconds
set(handles.text2, 'String','wait for 10 sec.....');
drawnow
disc = test.Start(10);
set(handles.text2, 'String','Done!');
% put the result in a list box
for i =0:disc.Count-1 
 s1 = strcat(char(disc.Item(i).Name),'::');
 s2 = strcat(s1,char(disc.Item(i).SelectedStreamingFormat));
 str{i+1} =  s2;   
end   
set(handles.listbox1,'string',str);
handles.disc = disc;
guidata(hObject,handles)

% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1
index_selected = get(hObject,'Value');

% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonconnect.
function pushbuttonconnect_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonconnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
index_selected = get(handles.listbox1,'Value');
disc = handles.disc;
% check if it is FlirFileFormat or mpeg streaming
if(strcmp(char(disc.Item(index_selected-1).SelectedStreamingFormat),'FlirFileFormat'))
    %It is FlirFileFormat init a ThermalCamera
    ImStream = Flir.Atlas.Live.Device.ThermalCamera(true);    
    ImStream.Connect(disc.Item(index_selected-1));
    %save the stream
    handles.ImStream = ImStream;
    handles.stop = 1;
    guidata(hObject,handles)
    %set the Iron palette
    pal = ImStream.ThermalImage.PaletteManager;
    ImStream.ThermalImage.Palette = pal.Iron; 

    pause(1);
    while handles.stop
      %get the colorized image   
      img = ImStream.ThermalImage.ImageArray;
      %convert to Matlab type
      X = uint8(img);
      axes(handles.axes1);
      %show image with Matlab imshow
      imshow(X);
      drawnow
      handles = guidata(hObject);
     

    end
else
    %mpeg stream
   ImStream = Flir.Atlas.Live.Device.VideoOverlayCamera(true);
   %connect
    ImStream.Connect(disc.Item(index_selected-1));
    handles.ImStream = ImStream;
    handles.stop = 1;
    guidata(hObject,handles)
    pause(1);
    while handles.stop
        % get the Image
        img = ImStream.VisualImage.ImageArray;
        X = uint8(img);  
        axes(handles.axes1);
        imshow(X,[]);
        drawnow
        handles = guidata(hObject);
    end    
end
ImStream = handles.ImStream;
ImStream.Disconnect();
ImStream.Dispose();

% --- Executes on button press in pushbuttonstop.
function pushbuttonstop_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonstop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.stop = 0;
guidata(hObject,handles);
