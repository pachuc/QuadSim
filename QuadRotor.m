classdef QuadRotor < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess= 'private', SetAccess= 'private')
        %state vars
        cur_state;
        start_state;
        desired_state;
        
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
        
        %control algorithm
        control;
        
    end
    
    methods(Access= 'public')
        
        % Constructor Function
        function obj= QuadRotor(varargin)
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
        obj.desired_state = [0 0 0 0 0 0 0 0 0 0 0 0];
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
            if length(inState) == 12
                set = 'Set start state.';
                obj.start_state = inState;
            else
                set = 'Did not set start state due to incorrect length.';
            end
        end
        
        function set = setDesiredState(obj, inState)
            if length(inState) == 12
                set = 'Set desired state.';
                obj.desired_state = inState;
            else
                set = 'Did not set desired state due to incorrect length.';
            end
        end
        
        function recordState(obj)
            obj.x(obj.step_count*obj.time_step) = obj.cur_state(1);
            obj.y(obj.step_count*obj.time_step) = obj.cur_state(2);
            obj.z(obj.step_count*obj.time_step) = obj.cur_state(3);
            obj.phi(obj.step_count*obj.time_step) = obj.cur_state(4);
            obj.theta(obj.step_count*obj.time_step) = obj.cur_state(5);
            obj.psi(obj.step_count*obj.time_step) = obj.cur_state(6);
            obj.xdot(obj.step_count*obj.time_step) = obj.cur_state(7);
            obj.ydot(obj.step_count*obj.time_step) = obj.cur_state(8);
            obj.zdot(obj.step_count*obj.time_step) = obj.cur_state(9);
            obj.phidot(obj.step_count*obj.time_step) = obj.cur_state(10);
            obj.thetadot(obj.step_count*obj.time_step) = obj.cur_state(11);
            obj.psidot(obj.step_count*obj.time_step) = obj.cur_state(12);
        end
        
        function setControl(obj, control)
            obj.control = control;
        end
        
        function controlvec = getControl(obj)
            controlvec = obj.control(obj.cur_state, obj.desired_state, obj.time_step);
        end
        
        function nextState(obj)
            controlvec = obj.getControl();
            obj.cur_state = quadr(controlvec, obj.cur_state, obj.time_step);
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
            results(1, :) = obj.x;
            results(2, :) = obj.y;
            results(3, :) = obj.z;
            results(4, :) = obj.phi;
            results(5, :) = obj.theta;
            results(6, :) = obj.psi;
            results(7, :) = obj.xdot;
            results(8, :) = obj.ydot;
            results(9, :) = obj.zdot;
            results(10, :) = obj.phidot;
            results(11, :) = obj.thetadot;
            results(12, :) = obj.psidot;
            obj.step_count = 1;
            obj.cur_state = obj.start_state;
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
            
        end
        
        
    end
    
end

function nextState = quadr(controlvec, statevec, timestep)

x = statevec(1);
y = statevec(2);
z = statevec(3);
phi = statevec(4);
theta = statevec(5);
psi = statevec(6);
xdot = statevec(7);
ydot = statevec(8);
zdot = statevec(9);
phidot = statevec(10);
thetadot = statevec(11);
psidot = statevec(12);
U1 = controlvec(1);
U2 = controlvec(2);
U3 = controlvec(3);
U4 = controlvec(4);

[U1_2, U2_2, U3_2, U4_2, omega] = Omegasquared(U1, U2, U3, U4);

[phidotdot, thetadotdot, psidotdot] = Angles(phidot, thetadot, psidot, U2_2, omega, U4_2, U3_2);

phidot = phidot + (phidotdot * timestep);
thetadot = thetadot + (thetadotdot * timestep);
psidot = psidot + (psidotdot * timestep);

phi = phi + (phidot * timestep);
theta = theta + (thetadot * timestep);
psi = psi + (psidot * timestep);

[xdotdot, ydotdot, zdotdot] = Displacement(phi, theta, psi, U1_2);
xdot = xdot + (xdotdot * timestep);
ydot = ydot + (ydotdot * timestep);
zdot = zdot + (zdotdot * timestep);

x = x + (xdot * timestep);
y = y + (ydot * timestep);
z = z + (zdot * timestep);

nextState = [x y z phi theta psi xdot ydot zdot phidot thetadot psidot];

end

function [U1_2, U2_2, U3_2, U4_2, omega] = Omegasquared(U1, U2, U3, U4)
b = 1; %thrust factor  (kg.m) 
d = 0.3048;  % Drag factor
l = 0.23;  % Distance to the center of the Quadrotor

T1 = U1/(4*b);
T2 = U2/(2*b*l);
T3 = U3/(2*b*l);
T4 = U4/(4*d);

OS1 = T1 + T3 - T4;
OS2 = T1 - T2 + T4;
OS3 = T1 - T3 - T4;
OS4 = T1 + T2 + T4;
OS5 = OS2 - OS1 - OS3 + OS4;

omega = dot(OS5, d);

OS1cal = OS1 + OS2 + OS3 + OS4;
OS2cal = OS4 - OS2;
OS3cal = OS1 - OS3;
OS4cal = OS2 - OS1 - OS3 + OS4;

U1_2 = b * OS1cal;
U2_2 = b * OS2cal;
U3_2 = b * OS3cal;
U4_2 = b * OS4cal;

end

function [xdotdot ydotdot zdotdot] = Displacement(phi, theta, psi, U1)
m = 1.89;  % Mass of the Quadrotor in Kg
g = 9.81;   % Gravitational acceleration


POE = cos(phi) * sin(theta) * cos(psi);
POE1 = sin(phi) * sin(psi);
POE2 = cos(phi) * sin(theta) *sin(psi);
POE3 = sin(phi) * cos(psi);
POE4 = cos(phi) * cos(theta);

D1 = U1/m;

xdotdot = (POE + POE1) * D1;
ydotdot = (POE2 + POE3) * D1;
zdotdot = (POE4 *D1) - g;

end

function [phidotdot thetadotdot psidotdot] = Angles(phidot, thetadot, psidot, U2, omega, U4, U3)

l = 0.23;  % Distance to the center of the Quadrotor

%moment of intertia units: kg.m^2
Ix = 2.297e-2;  % Quadrotor moment of inertia around X axis
Iy = 2.297e-2;  % Quadrotor moment of inertia around Y axis
Iz = 4.935e-2;  % Quadrotor moment of inertia around Z axis
%%%%%%%%
%need to check this param
Jr = 6.5*10^(-5);  % Total rotational moment of inertia around the propeller axis
%k  = 6e-5;					% rotor inertia (kg.m^2) same param?
%k = Jr; %set to 2?

a1 = (Iy - Iz)/Ix;
a2 = Jr/Ix;
b1 = l/Ix;
a3 = (Iz-Ix)/Iy;
a4 = Jr/Iy;
b2 = l/Iy;
a5 = (Ix-Iy)/Iz;
b3 = l/Iz;

POE = a1 * thetadot * psidot;
POE1 = a2 * thetadot * omega;
POE2 = b1 * U2;
phidotdot = POE + POE1 + POE2;

POE3 = a3 * phidot * psidot;
POE4 = a4 * phidot * omega;
POE5 = b2 * U3;
thetadotdot = POE3 + POE4 + POE5;

POE6 = a5 * phidot * thetadot;
POE7 = b3 * U4;
psidotdot = POE6 + POE7;

end