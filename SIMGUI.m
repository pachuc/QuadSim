function varargout = SIMGUI(varargin)
% SIMGUI MATLAB code for SIMGUI.fig
%      SIMGUI, by itself, creates a new SIMGUI or raises the existing
%      singleton*.
%
%      H = SIMGUI returns the handle to a new SIMGUI or the handle to
%      the existing singleton*.
%
%      SIMGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIMGUI.M with the given input arguments.
%
%      SIMGUI('Property','Value',...) creates a new SIMGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SIMGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SIMGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SIMGUI

% Last Modified by GUIDE v2.5 30-Nov-2014 17:48:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SIMGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @SIMGUI_OutputFcn, ...
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
end

% --- Executes just before SIMGUI is made visible.
function SIMGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SIMGUI (see VARARGIN)

% Choose default command line output for SIMGUI
handles.output = hObject;
%set up array to track variable toggles
vars = [0 0 0 0 0 0];
handles.vars = vars;
handles.runtime = 0;
handles.steptime = 0;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SIMGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);




end


% --- Outputs from this function are returned to the command line.
function varargout = SIMGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
end

% --- Executes on button press in Run.
function Run_Callback(hObject, eventdata, handles)
% hObject    handle to Run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end

% --- Executes on button press in graph.
function graph_Callback(hObject, eventdata, handles)
% hObject    handle to graph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

end

function timeStepEntry_Callback(hObject, eventdata, handles)
% hObject    handle to timeStepEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of timeStepEntry as text
%        str2double(get(hObject,'String')) returns contents of timeStepEntry as a double

end

% --- Executes during object creation, after setting all properties.
function timeStepEntry_CreateFcn(hObject, eventdata, handles)
% hObject    handle to timeStepEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function runTimeEntry_Callback(hObject, eventdata, handles)
% hObject    handle to runTimeEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of runTimeEntry as text
%        str2double(get(hObject,'String')) returns contents of runTimeEntry as a double

end
% --- Executes during object creation, after setting all properties.
function runTimeEntry_CreateFcn(hObject, eventdata, handles)
% hObject    handle to runTimeEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes on button press in setTime.
function setTime_Callback(hObject, eventdata, handles)
% hObject    handle to setTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.runtime = str2double(get(handles.runTimeEntry,'String'));
handles.steptime = str2double(get(handles.timeStepEntry,'String'));
string = sprintf('Set Run Time to %d. Set Time Step to %d.\n', handles.runtime, handles.steptime);
set(handles.out, 'String', string);
end

% --- Executes on button press in xCheck.
function xCheck_Callback(hObject, eventdata, handles)
% hObject    handle to xCheck (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of xCheck
end

% --- Executes on button press in yCheck.
function yCheck_Callback(hObject, eventdata, handles)
% hObject    handle to yCheck (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of yCheck
end

% --- Executes on button press in zCheck.
function zCheck_Callback(hObject, eventdata, handles)
% hObject    handle to zCheck (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of zCheck
end

% --- Executes on button press in phiCheck.
function phiCheck_Callback(hObject, eventdata, handles)
% hObject    handle to phiCheck (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of phiCheck
end

% --- Executes on button press in thetaCheck.
function thetaCheck_Callback(hObject, eventdata, handles)
% hObject    handle to thetaCheck (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of thetaCheck
end

% --- Executes on button press in psiCheck.
function psiCheck_Callback(hObject, eventdata, handles)
% hObject    handle to psiCheck (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of psiCheck

end

% --- Executes on button press in saveButton.
function saveButton_Callback(hObject, eventdata, handles)
% hObject    handle to saveButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
vars = handles.vars;
vars(1) = get(handles.xCheck, 'Value');
vars(2) = get(handles.yCheck, 'Value');
vars(3) = get(handles.zCheck, 'Value');
vars(4) = get(handles.psiCheck, 'Value');
vars(5) = get(handles.thetaCheck, 'Value');
vars(6) = get(handles.phiCheck, 'Value');
handles.vars = vars;

string = sprintf('Set the variables to record.\n');
set(handles.out, 'String', string);

end