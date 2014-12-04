function myLQR()
%QuadParams.m
%A file for loading all the parameter data into matlab.
%This file will initialize all the parameters to be used in the simulation.
%It will then start the simulation.

clc %clear the command window
clear all %clear all variables
close all %close all scripts.

global Jr Ix Iy Iz b d l m g;

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
global timestep;    
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

state = [0 0 0 0 0 0 0 0 0 0 0 0];%current state
xd = [0 0 1 0 0 0 0 0 0 0 0 0];	% desired state

%simulation total run time.
simlength = 20;

%vars for graphing.
x = zeros(1, simlength);
y = zeros(1, simlength);
z = zeros(1, simlength);
phi = zeros(1, simlength);
theta = zeros(1, simlength);
psi = zeros(1, simlength);
xdot = zeros(1, simlength);
ydot = zeros(1, simlength);
zdot = zeros(1, simlength);
phidot = zeros(1, simlength);
thetadot = zeros(1, simlength);
psidot = zeros(1, simlength);

for time = 1:simlength
    
    %store the state vars for graphing.
    x(time) = state(1);
    y(time) = state(2);
    z(time) = state(3);
    phi(time) = state(4);
    theta(time) = state(5);
    psi(time) = state(6);
    xdot(time) = state(7);
    ydot(time) = state(8);
    zdot(time) = state(9);
    phidot(time) = state(10);
    thetadot(time) = state(11);
    psidot(time) = state(12);
    
    fprintf('Time: %d\n', (time*timestep));
    fprintf('x: %d\n', state(1));
    fprintf('y: %d\n', state(2));
    fprintf('z: %d\n', state(3));
    fprintf('phi: %d\n', state(4));
    fprintf('theta: %d\n', state(5));
    fprintf('psi: %d\n', state(6));
    fprintf('xdot: %d\n', state(7));
    fprintf('ydot: %d\n', state(8));
    fprintf('zdot: %d\n', state(9));
    fprintf('phidot: %d\n', state(10));
    fprintf('thetadot: %d\n', state(11));
    fprintf('psidot: %d\n', state(12));
    
    %difference vector.
    xdiff = [0 0 0 0 0 0 0 0 0 0 0 0];
    
    xdiff(1) = state(1) - xd(1);
    xdiff(2) = state(2) - xd(2);
    xdiff(3) = state(3) - xd(3);
    xdiff(4) = state(4) - xd(4);
    xdiff(5) = state(5) - xd(5);
    xdiff(6) = state(6) - xd(6);
    xdiff(7) = state(7) - xd(7);
    xdiff(8) = state(8) - xd(8);
    xdiff(9) = state(9) - xd(9);
    xdiff(10) = state(10) - xd(10);
    xdiff(11) = state(11) - xd(11);
    xdiff(12) = state(12) - xd(12);
    
    xt = transpose(xdiff);
    u = K * xt;
    
    U1 = u(1);
    U2 = u(2);
    U3 = u(3);
    U4 = u(4);
    state = quadr(U1, U2, U3, U4, state);
    
    fprintf('U1: %d\n', U1);
    fprintf('U2: %d\n', U2);
    fprintf('U3: %d\n', U3);
    fprintf('U4: %d\n', U4);
    fprintf('----------------------------------------\n');   
    
end
format shortG;
disp(x);
end
