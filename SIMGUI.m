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

% Last Modified by GUIDE v2.5 16-Dec-2014 16:56:20

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
handles.runtime = 0;
handles.steptime = 0;
handles.vehicle = QuadRotor();
handles.QR = false;
handles.EDF = false;
init = [0 0 0 0 0 0 0 0 0 0 0 0];
handles.init_state = init;
handles.d_state = init;
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
obj = handles.vehicle;

handles.results = obj.startSim();
set(handles.out, 'String', 'Vehicle Simulation Data Generated.');
%format shortG;
%disp(handles.results);
% Update handles structure
guidata(hObject, handles);
end

% --- Executes on button press in graph.
function graph_Callback(hObject, eventdata, handles)
% hObject    handle to graph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

numvars = length(handles.results(:,1));
numtime = length(handles.results(1, :));
time = 1:1:numtime;
time = time * handles.steptime;
for i=1:numvars
    figure;
    data = handles.results(i, :);
    plot(time, data);
    
    if(handles.QR)
        
        switch(i)
            case 1
                title('Graph of Quad Rotor X Position');
            case 2
                title('Graph of Quad Rotor Y Position');
            case 3
                title('Graph of Quad Rotor Z Position');
            case 4
                title('Graph of Quad Rotor Phi Angle');
            case 5
                title('Graph of Quad Rotor Theta Angle');
            case 6
                title('Graph of Quad Rotor Psi Angle');
            case 7
                title('Graph of Quad Rotor X Velocity');
            case 8
                title('Graph of Quad Rotor Y Velocity');
            case 9
                title('Graph of Quad Rotor Z Velocity');
            case 10
                title('Graph of Quad Rotor Phi Angular Velocity');
            case 11
                title('Graph of Quad Rotor Theta Angular Velocity');
            case 12
                title('Graph of Quad Rotor Psi Angular Velocity');
                
        end
        
    elseif(handles.EDF)
        
        switch(i)
            case 1
                title('Graph of EDF8 Phi Angle');
            case 2
                title('Graph of EDF8 Theta Angle');
            case 3
                title('Graph of EDF8 Psi Angle');
            case 4
                title('Graph of EDF8 P Angular Body Frame Velocity');
            case 5
                title('Graph of EDF8 Q Angular Body Frame Velocity');
            case 6
                title('Graph of EDF8 R Angular Body Frame Velocity');
                
        end
    end
end

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
obj = handles.vehicle;
obj.setTime(handles.runtime, handles.steptime);
string = sprintf('Set Run Time to %d. Set Time Step to %d.\n', handles.runtime, handles.steptime);
set(handles.out, 'String', string);
% Update handles structure
guidata(hObject, handles);
end




% --- Executes on button press in control_button.
function control_button_Callback(hObject, eventdata, handles)
% hObject    handle to control_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Import control algorithm
[filename pathname filter]= uigetfile('*.m','Import Map File');
    

if(filename)
    temp = filename;
    temp = temp(1:end-2);
    funchandle = str2func(temp);
    obj = handles.vehicle;
    obj.setControl(funchandle);
    string = sprintf('Set control algorithm.\n');
    set(handles.out, 'String', string);
end

end


% --- Executes when selected object is changed in vPanel.
function vPanel_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in vPanel 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)

val = get(handles.edf8_sel, 'Value');
val2 = get(handles.QR_sel, 'Value');

if(val)
    handles.vehicle = EDF8();
    handles.init_state = [0 0 0 0 0 0];
    handles.d_state = [0 0 0 0 0 0];
    set(handles.init_state_qr, 'visible', 'off');
    set(handles.dstate_qr, 'visible', 'off');
    set(handles.init_state_edf, 'visible', 'on');
    set(handles.dstate_edf, 'visible', 'on');
    handles.QR = false;
    handles.EDF = true;
    string = sprintf('Set the vehicle to EDF8\n');
    set(handles.out, 'String', string);

elseif(val2)
    handles.vehicle = QuadRotor();
    handles.init_state = [0 0 0 0 0 0 0 0 0 0 0 0];
    handles.d_state = [0 0 0 0 0 0 0 0 0 0 0 0];
    set(handles.init_state_edf, 'visible', 'off');
    set(handles.dstate_edf, 'visible', 'off');
    set(handles.init_state_qr, 'visible', 'on');
    set(handles.dstate_qr, 'visible', 'on');
    handles.QR = true;
    handles.EDF = false;
    string = sprintf('Set the vehicle to QuadRotor\n');
    set(handles.out, 'String', string);
end

% Update handles structure
guidata(hObject, handles);
end


% --- Executes on button press in set_state.
function set_state_Callback(hObject, eventdata, handles)
% hObject    handle to set_state (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.QR)
    handles.init_state(1) = str2double(get(handles.xin1, 'String'));
    handles.init_state(2) = str2double(get(handles.yin1, 'String'));
    handles.init_state(3) = str2double(get(handles.zin1, 'String'));
    handles.init_state(4) = str2double(get(handles.phiin1, 'String'));
    handles.init_state(5) = str2double(get(handles.thetain1, 'String'));
    handles.init_state(6) = str2double(get(handles.psiin1, 'String'));
    handles.init_state(7) = str2double(get(handles.xdotin1, 'String'));
    handles.init_state(8) = str2double(get(handles.ydotin1, 'String'));
    handles.init_state(9) = str2double(get(handles.zdotin1, 'String'));
    handles.init_state(10) = str2double(get(handles.phidotin1, 'String'));
    handles.init_state(11) = str2double(get(handles.thetadotin1, 'String'));
    handles.init_state(12) = str2double(get(handles.psidotin1, 'String'));

    handles.d_state(1) = str2double(get(handles.xin2, 'String'));
    handles.d_state(2) = str2double(get(handles.yin2, 'String'));
    handles.d_state(3) = str2double(get(handles.zin2, 'String'));
    handles.d_state(4) = str2double(get(handles.phiin2, 'String'));
    handles.d_state(5) = str2double(get(handles.thetain2, 'String'));
    handles.d_state(6) = str2double(get(handles.psiin2, 'String'));
    handles.d_state(7) = str2double(get(handles.xdotin2, 'String'));
    handles.d_state(8) = str2double(get(handles.ydotin2, 'String'));
    handles.d_state(9) = str2double(get(handles.zdotin2, 'String'));
    handles.d_state(10) = str2double(get(handles.phidotin2, 'String'));
    handles.d_state(11) = str2double(get(handles.thetadotin2, 'String'));
    handles.d_state(12) = str2double(get(handles.psidotin2, 'String'));
elseif(handles.EDF)
    
    handles.init_state(1) = str2double(get(handles.phiin3, 'String'));
    handles.init_state(2) = str2double(get(handles.thetain3, 'String'));
    handles.init_state(3) = str2double(get(handles.psiin3, 'String'));
    handles.init_state(4) = str2double(get(handles.pin3, 'String'));
    handles.init_state(5) = str2double(get(handles.qin3, 'String'));
    handles.init_state(6) = str2double(get(handles.rin3, 'String'));

    handles.d_state(1) = str2double(get(handles.phiin4, 'String'));
    handles.d_state(2) = str2double(get(handles.thetain4, 'String'));
    handles.d_state(3) = str2double(get(handles.psiin4, 'String'));
    handles.d_state(4) = str2double(get(handles.pin4, 'String'));
    handles.d_state(5) = str2double(get(handles.qin4, 'String'));
    handles.d_state(6) = str2double(get(handles.rin4, 'String'));

end




obj = handles.vehicle;

string1 = obj.setStartState(handles.init_state);
string2 = obj.setDesiredState(handles.d_state);
string = strcat(string1, ' and ', string2);
set(handles.out, 'String', string);

% Update handles structure
guidata(hObject, handles);

end


% --- Executes during object creation, after setting all properties.
function xin2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function xin1_Callback(hObject, eventdata, handles)
% hObject    handle to xin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xin1 as text
%        str2double(get(hObject,'String')) returns contents of xin1 as a double
end

% --- Executes during object creation, after setting all properties.
function xin1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function yin1_Callback(hObject, eventdata, handles)
% hObject    handle to yin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yin1 as text
%        str2double(get(hObject,'String')) returns contents of yin1 as a double
end

% --- Executes during object creation, after setting all properties.
function yin1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function zin1_Callback(hObject, eventdata, handles)
% hObject    handle to zin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zin1 as text
%        str2double(get(hObject,'String')) returns contents of zin1 as a double

end

% --- Executes during object creation, after setting all properties.
function zin1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function phiin1_Callback(hObject, eventdata, handles)
% hObject    handle to phiin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of phiin1 as text
%        str2double(get(hObject,'String')) returns contents of phiin1 as a double

end


% --- Executes during object creation, after setting all properties.
function phiin1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phiin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function thetain1_Callback(hObject, eventdata, handles)
% hObject    handle to thetain1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thetain1 as text
%        str2double(get(hObject,'String')) returns contents of thetain1 as a double
end

% --- Executes during object creation, after setting all properties.
function thetain1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thetain1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function psiin1_Callback(hObject, eventdata, handles)
% hObject    handle to psiin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of psiin1 as text
%        str2double(get(hObject,'String')) returns contents of psiin1 as a double
end

% --- Executes during object creation, after setting all properties.
function psiin1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to psiin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function xin2_Callback(hObject, eventdata, handles)
% hObject    handle to xin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xin2 as text
%        str2double(get(hObject,'String')) returns contents of xin2 as a double
end


function yin2_Callback(hObject, eventdata, handles)
% hObject    handle to yin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yin2 as text
%        str2double(get(hObject,'String')) returns contents of yin2 as a double
end


% --- Executes during object creation, after setting all properties.
function yin2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function zin2_Callback(hObject, eventdata, handles)
% hObject    handle to zin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zin2 as text
%        str2double(get(hObject,'String')) returns contents of zin2 as a double

end

% --- Executes during object creation, after setting all properties.
function zin2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function phiin2_Callback(hObject, eventdata, handles)
% hObject    handle to phiin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of phiin2 as text
%        str2double(get(hObject,'String')) returns contents of phiin2 as a double

end

% --- Executes during object creation, after setting all properties.
function phiin2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phiin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function thetain2_Callback(hObject, eventdata, handles)
% hObject    handle to thetain2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thetain2 as text
%        str2double(get(hObject,'String')) returns contents of thetain2 as a double

end

% --- Executes during object creation, after setting all properties.
function thetain2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thetain2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function psiin2_Callback(hObject, eventdata, handles)
% hObject    handle to psiin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of psiin2 as text
%        str2double(get(hObject,'String')) returns contents of psiin2 as a double

end

% --- Executes during object creation, after setting all properties.
function psiin2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to psiin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function psidotin2_Callback(hObject, eventdata, handles)
% hObject    handle to psidotin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of psidotin2 as text
%        str2double(get(hObject,'String')) returns contents of psidotin2 as a double

end

% --- Executes during object creation, after setting all properties.
function psidotin2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to psidotin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function thetadotin2_Callback(hObject, eventdata, handles)
% hObject    handle to thetadotin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thetadotin2 as text
%        str2double(get(hObject,'String')) returns contents of thetadotin2 as a double
end

% --- Executes during object creation, after setting all properties.
function thetadotin2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thetadotin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function phidotin2_Callback(hObject, eventdata, handles)
% hObject    handle to phidotin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of phidotin2 as text
%        str2double(get(hObject,'String')) returns contents of phidotin2 as a double

end
% --- Executes during object creation, after setting all properties.
function phidotin2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phidotin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function zdotin2_Callback(hObject, eventdata, handles)
% hObject    handle to zdotin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zdotin2 as text
%        str2double(get(hObject,'String')) returns contents of zdotin2 as a double
end

% --- Executes during object creation, after setting all properties.
function zdotin2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zdotin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function ydotin2_Callback(hObject, eventdata, handles)
% hObject    handle to ydotin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ydotin2 as text
%        str2double(get(hObject,'String')) returns contents of ydotin2 as a double

end
% --- Executes during object creation, after setting all properties.
function ydotin2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ydotin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function xdotin2_Callback(hObject, eventdata, handles)
% hObject    handle to xdotin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xdotin2 as text
%        str2double(get(hObject,'String')) returns contents of xdotin2 as a double
end

% --- Executes during object creation, after setting all properties.
function xdotin2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xdotin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function xdotin1_Callback(hObject, eventdata, handles)
% hObject    handle to xdotin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xdotin1 as text
%        str2double(get(hObject,'String')) returns contents of xdotin1 as a double

end

% --- Executes during object creation, after setting all properties.
function xdotin1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xdotin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function ydotin1_Callback(hObject, eventdata, handles)
% hObject    handle to ydotin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ydotin1 as text
%        str2double(get(hObject,'String')) returns contents of ydotin1 as a double
end

% --- Executes during object creation, after setting all properties.
function ydotin1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ydotin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function zdotin1_Callback(hObject, eventdata, handles)
% hObject    handle to zdotin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zdotin1 as text
%        str2double(get(hObject,'String')) returns contents of zdotin1 as a double

end
% --- Executes during object creation, after setting all properties.
function zdotin1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zdotin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function phidotin1_Callback(hObject, eventdata, handles)
% hObject    handle to phidotin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of phidotin1 as text
%        str2double(get(hObject,'String')) returns contents of phidotin1 as a double

end
% --- Executes during object creation, after setting all properties.
function phidotin1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phidotin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function thetadotin1_Callback(hObject, eventdata, handles)
% hObject    handle to thetadotin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thetadotin1 as text
%        str2double(get(hObject,'String')) returns contents of thetadotin1 as a double

end

% --- Executes during object creation, after setting all properties.
function thetadotin1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thetadotin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function psidotin1_Callback(hObject, eventdata, handles)
% hObject    handle to psidotin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of psidotin1 as text
%        str2double(get(hObject,'String')) returns contents of psidotin1 as a double

end
% --- Executes during object creation, after setting all properties.
function psidotin1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to psidotin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


end
function rin3_Callback(hObject, eventdata, handles)
% hObject    handle to rin3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rin3 as text
%        str2double(get(hObject,'String')) returns contents of rin3 as a double

end
% --- Executes during object creation, after setting all properties.
function rin3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rin3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function qin3_Callback(hObject, eventdata, handles)
% hObject    handle to qin3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of qin3 as text
%        str2double(get(hObject,'String')) returns contents of qin3 as a double
end

% --- Executes during object creation, after setting all properties.
function qin3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qin3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function pin3_Callback(hObject, eventdata, handles)
% hObject    handle to pin3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pin3 as text
%        str2double(get(hObject,'String')) returns contents of pin3 as a double

end

% --- Executes during object creation, after setting all properties.
function pin3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pin3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function psiin3_Callback(hObject, eventdata, handles)
% hObject    handle to psiin3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of psiin3 as text
%        str2double(get(hObject,'String')) returns contents of psiin3 as a double

end

% --- Executes during object creation, after setting all properties.
function psiin3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to psiin3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function thetain3_Callback(hObject, eventdata, handles)
% hObject    handle to thetain3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thetain3 as text
%        str2double(get(hObject,'String')) returns contents of thetain3 as a double
end


% --- Executes during object creation, after setting all properties.
function thetain3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thetain3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


end

function phiin3_Callback(hObject, eventdata, handles)
% hObject    handle to phiin3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of phiin3 as text
%        str2double(get(hObject,'String')) returns contents of phiin3 as a double

end

% --- Executes during object creation, after setting all properties.
function phiin3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phiin3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function phiin4_Callback(hObject, eventdata, handles)
% hObject    handle to phiin4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of phiin4 as text
%        str2double(get(hObject,'String')) returns contents of phiin4 as a double

end

% --- Executes during object creation, after setting all properties.
function phiin4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phiin4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function thetain4_Callback(hObject, eventdata, handles)
% hObject    handle to thetain4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thetain4 as text
%        str2double(get(hObject,'String')) returns contents of thetain4 as a double
end

% --- Executes during object creation, after setting all properties.
function thetain4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thetain4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function psiin4_Callback(hObject, eventdata, handles)
% hObject    handle to psiin4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of psiin4 as text
%        str2double(get(hObject,'String')) returns contents of psiin4 as a double
end

% --- Executes during object creation, after setting all properties.
function psiin4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to psiin4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function pin4_Callback(hObject, eventdata, handles)
% hObject    handle to pin4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pin4 as text
%        str2double(get(hObject,'String')) returns contents of pin4 as a double

end

% --- Executes during object creation, after setting all properties.
function pin4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pin4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function qin4_Callback(hObject, eventdata, handles)
% hObject    handle to qin4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of qin4 as text
%        str2double(get(hObject,'String')) returns contents of qin4 as a double

end

% --- Executes during object creation, after setting all properties.
function qin4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qin4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


end

function rin4_Callback(hObject, eventdata, handles)
% hObject    handle to rin4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rin4 as text
%        str2double(get(hObject,'String')) returns contents of rin4 as a double

end

% --- Executes during object creation, after setting all properties.
function rin4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rin4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


% --- Executes on button press in save_button.
function save_button_Callback(hObject, eventdata, handles)
% hObject    handle to save_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
myvar = handles.results;
uisave('myvar', 'untitled');
end
% --- Executes on button press in load_button.
function load_button_Callback(hObject, eventdata, handles)
% hObject    handle to load_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Import map file
[filename pathname filter]= uigetfile('*.mat','Import Data Set');

if(filename)
    load = importdata(filename);
    handles.results = load;
end

% Update handles structure
guidata(hObject, handles);

end