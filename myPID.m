function myPID()
%QuadParams.m
%A file for loading all the parameter data into matlab.
%This file will initialize all the parameters to be used in the simulation.
%It will then start the simulation.

clc %clear the command window
clear all %clear all variables
close all %close all scripts.

global Jr Ix Iy Iz b d l m g kpz kdz kpp kdp kpt kdt kpps kdps 

% Quadrotor constants
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%moment of intertia units: kg.m^2
Ix = 2.297e-2;  % Quadrotor moment of inertia around X axis
Iy = 2.297e-2;  % Quadrotor moment of inertia around Y axis
Iz = 4.935e-2;  % Quadrotor moment of inertia around Z axis
%%%%%%%%
%need to check this param
Jr = 6.5*10^(-5);  % Total rotational moment of inertia around the propeller axis
%k  = 6e-5;					% rotor inertia (kg.m^2) same param?
k = Jr; %set to 2?
%%%%%

b = 1; %thrust factor  (kg.m)
%b  = 1.2176e-5;
d = 0.3048;  % Drag factor
l = 0.23;  % Distance to the center of the Quadrotor
m = 1.89;  % Mass of the Quadrotor in Kg
g = 9.81;   % Gravitational acceleration

%quadrotor state vars:
global x y z phi theta psi xdot ydot zdot phidot thetadot psidot;
global timestep;

%inint the state vars:
x = 0;
y = 0;
z = 0;
phi = 0;
theta = 0;
psi = 0;
xdot = 0;
ydot = 0;
zdot = 0;
phidot = 0;
thetadot = 0;
psidot = 0;    
timestep = 0.02;%seconds    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xd = [0 0 1 0 0 0 0 0 0 0 0 0];	% desired state



% of the PD controller

kpp = 0.8;
kdp = 0.4;

kpt = 1.2;
kdt = 0.4;

kpps = 1;
kdps = 0.4;

kpz = 100;
kdz = 20;
Gains = [kpp kdp kpt kdt kpps kdps kpz kdz];
disp(Gains);


%the sim running my functions:
for time = 1:5000
    
    fprintf('Time: %d\n', (time*timestep));
    fprintf('x: %d\n', x);
    fprintf('y: %d\n', y);
    fprintf('z: %d\n', z);
    fprintf('phi: %d\n', phi);
    fprintf('theta: %d\n', theta);
    fprintf('psi: %d\n', psi);
    fprintf('xdot: %d\n', xdot);
    fprintf('ydot: %d\n', ydot);
    fprintf('zdot: %d\n', zdot);
    fprintf('phidot: %d\n', phidot);
    fprintf('thetadot: %d\n', thetadot);
    fprintf('psidot: %d\n', psidot);
    
    %difference vector.
    xdiff = [0 0 0 0 0 0 0 0 0 0 0 0];
    
%     xdiff(1) = x - xd(1);
%     xdiff(2) = y - xd(2);
%     xdiff(3) = z - xd(3);
%     xdiff(4) = phi - xd(4);
%     xdiff(5) = theta - xd(5);
%     xdiff(6) = psi - xd(6);
%     xdiff(7) = xdot - xd(7);
%     xdiff(8) = ydot - xd(8);
%     xdiff(9) = zdot - xd(9);
%     xdiff(10) = phidot - xd(10);
%     xdiff(11) = thetadot - xd(11);
%     xdiff(12) = psidot - xd(12);
    
    xdiff(1) = xd(1) - x;
    xdiff(2) = xd(2) - y;
    xdiff(3) = xd(3) - z;
    xdiff(4) = xd(4) - phi;
    xdiff(5) = xd(5) - theta;
    xdiff(6) = xd(6) - psi;
    xdiff(7) = xd(7) - xdot;
    xdiff(8) = xd(8) - ydot;
    xdiff(9) = xd(9) - zdot;
    xdiff(10) = xd(10) - phidot;
    xdiff(11) = xd(11) - thetadot;
    xdiff(12) = xd(12) - psidot;
    
    comp1 = xdiff(3) * kpz;
    comp2 = (z/timestep) * kdz;
    comp3 = (comp1 - comp2) * m;
    comp4 = cos(phi)*cos(theta);
    comp5 = comp3/comp4;
    U1 = 1 - comp5;
    
    comp1 = xdiff(4) * kpp;
    comp2 = (phi/timestep) * kdp;
    U2 = comp1 - comp2;
    
    comp1 = xdiff(5) * kpt;
    comp2 = (theta/timestep) * kdt;
    U3 = comp1 - comp2;
    
    comp1 = xdiff(6) * kpps;
    comp2 = (psi/timestep) * kdps;
    U4 = comp1 - comp2;
    
    quadr(U1, U2, U3, U4);
    
    fprintf('U1: %d\n', U1);
    fprintf('U2: %d\n', U2);
    fprintf('U3: %d\n', U3);
    fprintf('U4: %d\n', U4);
    fprintf('----------------------------------------\n');   
    
end
% % Controlling the Quadrotor
%sim('QuadSimulink');

%disp(xcur)
%disp(xd);
%disp(K);
%xt = transpose(xcur);
%disp(xt);
%u = K * transpose(xcur);
%disp(u);

end

