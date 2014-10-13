%QuadParams.m
%A file for loading all the parameter data into matlab.
%This file will initialize all the parameters to be used in the simulation.
%It will then start the simulation.

clc %clear the command window
clear all %clear all variables
close all %close all scripts.

global Jr Ix Iy Iz b d l m g Kpz Kdz Kpp Kdp Kpt Kdt Kpps Kdps ZdF PhidF ThetadF PsidF ztime phitime thetatime psitime Zinit Phiinit Thetainit Psiinit Uone Utwo Uthree Ufour Ez Ep Et Eps

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
xd = [0 0 0.03 0 0 0 0 0 0 0 0 0];	% desired state
delta = 0; %pi/4;					% yaw at linearization point


%start the simulink simulation
%sim('QuadSimulink')

%%%%%%%%%%Have to do this part in simulink.
%%%%% Dynamics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Nonlinear System Dynamics

Rx = @(tx)([1 0 0; 0 cos(tx) -sin(tx); 0 sin(tx) cos(tx)]);
Ry = @(ty)([cos(ty) 0 -sin(ty); 0 1 0; sin(ty) 0 cos(ty)]);
Rz = @(tz)([cos(tz) -sin(tz) 0; sin(tz) cos(tz) 0; 0 0 1]);
Rxyz = @(theta)(Rx(theta(1))*Ry(theta(2))*Rz(theta(3)));

x_dot = @(x, u)([	x(7);
					x(8);
					x(9);
					(x(10) + sin(x(4))*tan(x(5))*x(11) + cos(x(4))*tan(x(5))*x(12));
					(cos(x(4))*x(11) + sin(x(4))*x(12));
					((sin(x(4))*x(11) + cos(x(4))*x(12))/cos(x(5)));
					(-(b*sum(u)/m)*(cos(x(4))*sin(x(5))*cos(x(6)) + sin(x(4))*sin(x(6))));
					(-(b*sum(u)/m)*(cos(x(4))*sin(x(5))*sin(x(6)) + sin(x(4))*cos(x(6))));
					(g - (b*sum(u)/m)*cos(x(4))*cos(x(5)));
					((d*b/Ix)*(u(4) - u(2)) - ((Iz - Iy)/Ix)*x(11)*x(12));
					((d*b/Iy)*(u(1) - u(3)) - ((Ix - Iz)/Iy)*x(10)*x(12));
					((k/Iz)*(u(1) - u(2) + u(3) - u(4)) - ((Iy - Ix)/Iz)*x(10)*x(11))
				]);

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

% % Controlling the Quadrotor
%sim('QuadSimulink');

%disp(xcur)
%disp(xd);
%disp(K);
%xt = transpose(xcur);
%disp(xt);
%u = K * transpose(xcur);
%disp(u);