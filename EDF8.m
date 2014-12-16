classdef EDF8 < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess= 'private', SetAccess= 'private')
        %state vars
        cur_state;
        start_state;
        desired_state;
        
        %recorded vars
        phi;
        theta;
        psi;
        p;
        q;
        r;
        
        %time variables
        run_time;
        time_step;
        step_count;
        
        %control algorithm
        control;
        
    end
    
    methods(Access= 'public')
        
        % Constructor Function
        function obj= EDF8(varargin)
        % obj = RedRover
        % Creates instance of the user-defined class Create Robot and
        % initializes all properties. Note that if RedRover is not
        % called by SimulatorGUI (with handlesGUI argument), full
        % functionality impossible.
        %
        % obj = RedRover(handlesGUI)
        % Format of function call from SimulatorGUI. Passes along structure
        % of handles for the GUI to allow full functionality
        %
        % Input:
        % handlesGUI - Structure of handles to GUI objects
        %   e.g. handlesGUI.push_adv - handle for advance button
        %
        % Output:
        % obj - Instance of class RedRover with all fields initialized
        
        obj.run_time = 0;
        obj.time_step = 0;
        obj.step_count = 1;
        obj.phi = [];
        obj.theta = [];
        obj.psi = [];
        obj.p = [];
        obj.q = [];
        obj.r = [];
        obj.cur_state = [0 0 0 0 0 0];
        obj.start_state = [0 0 0 0 0 0];
        obj.desired_state = [0 0 0 0 0 0];
        end
        
        
        function set = setTime(obj, runTime, timeStep)
            
            if( timeStep < runTime)
                steps = runTime/timeStep;
                
                if(~mod(steps,1))
                    obj.run_time = runTime;
                    obj.time_step = timeStep;
                    obj.step_count = 1;
                    set = 'Time set correctly.';
                else
                    set = 'Failed to set time due to unacceptable values';
                end
            else
                set = 'Failed to set time due to unacceptable values';
            end
            
        end
        
        function set = setStartState(obj, inState)
            if length(inState) == 6
                set = 'Set start state.';
                obj.start_state = inState;
            else
                set = 'Did not set start state due to incorrect length.';
            end
        end
        
        function set = setDesiredState(obj, inState)
            if length(inState) == 6
                set = 'Set desired state.';
                obj.desired_state = inState;
            else
                set = 'Did not set desired state due to incorrect length.';
            end
        end
        
        function recordState(obj)
            obj.phi(obj.step_count) = obj.cur_state(1);
            obj.theta(obj.step_count) = obj.cur_state(2);
            obj.psi(obj.step_count) = obj.cur_state(3);
            obj.p(obj.step_count) = obj.cur_state(4);
            obj.q(obj.step_count) = obj.cur_state(5);
            obj.r(obj.step_count) = obj.cur_state(6);
        end
        
        function setControl(obj, control)
            obj.control = control;
        end
        
        function controlvec = getControl(obj)
            controlvec = obj.control(obj.cur_state, obj.desired_state, obj.time_step);
        end
        
        function nextState(obj)
            controlvec = obj.getControl();
            xd = transpose(TruncatedLinearDynamics(obj.cur_state, controlvec));
            xd = xd * obj.time_step;
            obj.cur_state = obj.cur_state + xd;
            obj.step_count = obj.step_count+1;
        end
        
        function results = startSim(obj)
            obj.cur_state = obj.start_state;
            time = obj.time_step * obj.step_count;
            
            while time <= obj.run_time
                obj.recordState();
                obj.nextState();
                time = obj.time_step * obj.step_count;
            end
            
            results = obj.stopSim();
        end
        
        function results = stopSim(obj)
            results = [];
            results(1, :) = obj.phi;
            results(2, :) = obj.theta;
            results(3, :) = obj.psi;
            results(4, :) = obj.p;
            results(5, :) = obj.q;
            results(6, :) = obj.r;
            obj.step_count = 1;
            obj.cur_state = obj.start_state;
            obj.phi = [];
            obj.theta = [];
            obj.psi = [];
            obj.p = [];
            obj.q = [];
            obj.r = [];
            
            
        end
        
        
    end
    
end


function output = TruncatedLinearDynamics(state, control)

%Declare the A matrix.
A(1:6, 1:6) = 0;
A(1, 4) = 1;
A(5, 4) = -6.74166;
A(1, 5) = -1.31031*10^-16;
A(2, 5) = 1;
A(3, 5) = -5.34932*10^-09;
A(4, 5) = 7.17817;
A(6, 5) = 1.79506*10^-16;
A(1, 6) = 2.4495*10^-08;
A(2, 6) = 5.34932*10^-09;
A(3, 6) = 1;

%B vanes matrix.
B(1:6, 1:5) = 0;
B(4, 1) = -0.432786;
B(6, 1) = 0.187629;
B(4, 2) = -0.432786;
B(6, 2) = -0.187629;
B(4, 3) = -6.27632*10^-18;
B(5, 3) = -0.406468;
B(6, 3) = -0.187629;
B(4, 4) = 6.27632*10^-18;
B(5, 4) = -0.406468;
B(6, 4) = 0.187629;
B(4, 5) = -7.35525*10^-12;
B(5, 5) = -1.71181*10^-10;
B(6, 5) = -5.70122*10^-05;

format shortG;

init_state = [1.23*10^-7 1.95*10^-7 -0.108 0 0 0];%phi theta psi p q r

init_vanes = [-0.32 0.32 0.32 -0.32 8301.46]; %d1 d2 d3 d4 dt



xd = transpose(state-init_state);
ud = transpose(control - init_vanes);
output = A*xd + B*ud;
end


% function xdot = EDF8(x, u)
% %x is the state vector
% %u is vanes
% %delta?
% 
% A(1:6, 1:6) = 0;
% B(1:4, 1:4) = 0;
% 
% A(1, 4) = 1;
% A(5, 4) = -6.74166;
% A(1, 5) = -1.31031 * 10^-16;
% A(2, 5) = 1;
% A(3, 5) = -5.34932*10^-09;
% A(4, 5) = 7.17817;
% A(6, 5) = 1.79506*10^-16;
% A(1, 6) = 2.4495*10^-08;
% A(2, 6) = 5.34932*10^-09;
% A(3, 6) = 1;
% 
% B(4, 1) = -0.865572;
% B(6, 1) = -2.16457*10^-17;
% B(5, 2) = -0.812936;
% B(4, 3) = -2.51056*10^-17;
% B(5, 3) = 1.70038*10^-15;
% B(6, 3) = -0.750516;
% B(4, 4) = -7.35525*10^-12;
% B(5, 4) = -1.71181*10^-10;
% B(6, 4) = -5.70122*10^-05;
% 
% format shortG;
% disp(A);
% disp(B);
% 

%end

