function myLQR()
%QuadParams.m
%A file for loading all the parameter data into matlab.
%This file will initialize all the parameters to be used in the simulation.
%It will then start the simulation.

clc %clear the command window
clear all %clear all variables
close all %close all scripts.

global Jr Ix Iy Iz b d l m g; %Kpz Kdz Kpp Kdp Kpt Kdt Kpps Kdps ZdF PhidF ThetadF PsidF ztime phitime thetatime psitime Zinit Phiinit Thetainit Psiinit Uone Utwo Uthree Ufour Ez Ep Et Eps

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

% Control parameters
wx = [1 1 1]*1e01;      % weights for state vector - position
wa = [1 1 1]*1e10;			% weights for state vector - orientation
wr = [1 1 1]*1e01;			% weights for state vector - rates
wp = [1 1 1]*1e01;			% weights for state vector - angular rates 
wu = [1 1 1 1]*1e05;		% weights for control vector

% from paper:
%wx = [1 1 1]*1e10;      % weights for state vector - position
%wa = [1 1 1]*1e10;      % weights for state vector - orientation
%wr = [1 1 1]*1e10;      % weights for state vector - rates
%wp = [1 1 1]*1e10;      % weights for state vector - angular rates 
%wu = [1 1 1 1]*1e0;    % weights for control vector

minControl =  700; %1000;
maxControl = 1000; %2000;

initX = [0 0 0.03];
initA = [0 0 0];
initR = [0 0 -2];
initP = [0 0 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%xd = zeros(12, 1);
xcur = [0 0 5 0 0 0 0 0 0 0 0 0];
xd = [0 0 0 0 0 0 0 0 0 0 0 0];	% desired state
delta = 0; %pi/4;					% yaw at linearization point

% LQR Controller
% state defined as: [x y z tx ty tz x' y' z' tx' ty' tz']
Q = diag([wx wa wr wp]);
%disp(Q);
R = diag(wu);
%disp(R)
% Linearized System Dynamics
M = [(-b/m)*ones(1,4); (d*b/Ix)*[0 -1 0 1]; (d*b/Iy)*[1 0 -1 0]; (k/Iz)*[1 -1 1 -1]];
N = [-g*sin(delta) -g*cos(delta); g*cos(delta) -g*sin(delta)];

A = [zeros(6) eye(6); zeros(2,3) N zeros(2,1) eye(2) zeros(2, 4); zeros(4, 8) eye(4)];
B = [zeros(8,4); M];

A_hat = [A (1500*pi^2*0.00896657/(4*1.225*0.1524^4))*B*ones(4,1); zeros(1, 12) 1];
B_hat = [B; zeros(1, 4)];

% Control gain
[K, P, L] = lqr(A, B, Q, R);
%[K, P, L] = lqr(A_hat, B_hat, Q, R);
save K.mat K


%the sim running my functions:


for time = 1:100
    
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
    
    xt = transpose(xdiff);
    u = K * xt;
    
    U1 = u(1);
    U2 = u(2);
    U3 = u(3);
    U4 = u(4);
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

function [xdotdot ydotdot zdotdot] = Displacement(phi, theta, psi, U1)
global m g;

POE = cos(phi) * sin(theta) * cos(psi);
POE1 = sin(phi) * sin(psi);
POE2 = cos(phi) * sin(theta) *sin(psi);
POE3 = sin(phi) * cos(psi);
POE4 = cos(phi) * cos(theta);

D1 = U1/m;
xdotdot = (POE + POE1) * D1;
ydotdot = (POE2 + POE3) * D1;
zdotdot = g - (POE4 *D1);

end

function [phidotdot thetadotdot psidotdot] = Angles(phidot, thetadot, psidot, U2, omega, U4, U3)
global Jr Ix Iy Iz b d l m g;

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

function [U1_2, U2_2, U3_2, U4_2, omega] = omegasquared(U1, U2, U3, U4)
global b l d;

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

function quadr(U1, U2, U3, U4)

global x y z phi theta psi xdot ydot zdot phidot thetadot psidot;
global timestep;

[U1_2, U2_2, U3_2, U4_2, omega] = omegasquared(U1, U2, U3, U4);

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

end