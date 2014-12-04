classdef QuadRotor < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess= 'private', SetAccess= 'private')
        %state vars
        cur_state;
        start_state;
        end_state;
        
        %recorded vars
        x;
        y;
        z;
        phi;
        theta;
        psi;
        xdot;
        ydot;
        zdot;
        phidot;
        thetadot;
        psidot;
        
        %time variables
        run_time;
        time_step;
        step_count;
        
    end
    
    methods(Access= 'public')
        
        % Constructor Function
        function obj= Rover8(varargin)
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
        obj.step_count = 0;
        obj.x = [];
        obj.y = [];
        obj.z = [];
        obj.phi = [];
        obj.theta = [];
        obj.psi = [];
        obj.xdot = [];
        obj.ydot = [];
        obj.zdot = [];
        obj.phidot = [];
        obj.thetadot = [];
        obj.psidot = [];
        obj.cur_state = [0 0 0 0 0 0 0 0 0 0 0 0];
        obj.start_state = [0 0 0 0 0 0 0 0 0 0 0 0];
        obj.end_state = [0 0 0 0 0 0 0 0 0 0 0 0];
        end
        
        
        function set = setTime(obj, runTime, timeStep)
            
            if( timeStep < runTime)
                steps = runTime/timeStep;
                
                if(isinteger(steps))
                    obj.run_time = runTime;
                    obj.time_step = timeStep;
                    obj.step_count = steps;
                    set = 'Time set correctly.';
                else
                    set = 'Failed to set time due to unacceptable values';
                end
            else
                set = 'Failed to set time due to unacceptable values';
            end
            
        end
        
        function set = setStartState(obj, inState)
            if length(inState) == 12
                set = 'Set start state.';
                obj.start_state = inState;
            else
                set = 'Did not set start state due to incorrect length.';
            end
        end
        
        function recordState(obj)
            obj.x(obj.step_count) = obj.cur_state(1);
            obj.y(obj.step_count) = obj.cur_state(2);
            obj.z(obj.step_count) = obj.cur_state(3);
            obj.phi(obj.step_count) = obj.cur_state(4);
            obj.theta(obj.step_count) = obj.cur_state(5);
            obj.psi(obj.step_count) = obj.cur_state(6);
            obj.xdot(obj.step_count) = obj.cur_state(7);
            obj.ydot(obj.step_count) = obj.cur_state(8);
            obj.zdot(obj.step_count) = obj.cur_state(9);
            obj.phidot(obj.step_count) = obj.cur_state(10);
            obj.thetadot(obj.step_count) = obj.cur_state(11);
            obj.psidot(obj.step_count) = obj.cur_state(12);
        end
        
        function nextState(obj)
            
            obj.step_count = obj.step_count+1;
            
        end
        
        function startSim(obj)
            time = obj.time_step * obj.step_count;
            
            while time < obj.run_time
                obj.recordState();
                obj.nextState();
                time = obj.time_step * obj.step_count;
            end
        end
        
        
        
    end
    
end

