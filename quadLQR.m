function controlvec = quadLQR(state_vec, desired_state, time_step)
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
m = 1.89;  % Mass of the Quadrotor in Kg
g = 9.81;   % Gravitational acceleration
delta = 0; %pi/4;					% yaw at linearization point


% Control parameters
wx = [1 1 1]*1e01;      % weights for state vector - position
wa = [1 1 1]*1e10;			% weights for state vector - orientation
wr = [1 1 1]*1e01;			% weights for state vector - rates
wp = [1 1 1]*1e01;			% weights for state vector - angular rates 
wu = [1 1 1 1]*1e05;		% weights for control vector



% LQR Controller
% state defined as: [x y z tx ty tz x' y' z' tx' ty' tz']
Q = diag([wx wa wr wp]);
%disp(Q);
R = diag(wu);
M = [(-b/m)*ones(1,4); (d*b/Ix)*[0 -1 0 1]; (d*b/Iy)*[1 0 -1 0]; (k/Iz)*[1 -1 1 -1]];
N = [-g*sin(delta) -g*cos(delta); g*cos(delta) -g*sin(delta)];
A = [zeros(6) eye(6); zeros(2,3) N zeros(2,1) eye(2) zeros(2, 4); zeros(4, 8) eye(4)];
B = [zeros(8,4); M];
[K, P, L] = lqr(A, B, Q, R);


state(1) = state_vec(1) - desired_state(1);
state(2) = state_vec(1) - desired_state(1);
state(3) = state_vec(1) - desired_state(1);
state(4) = state_vec(1) - desired_state(1);
state(5) = state_vec(1) - desired_state(1);
state(6) = state_vec(1) - desired_state(1);

state(7) = state_vec(1) - desired_state(1);
state(8) = state_vec(1) - desired_state(1);
state(9) = state_vec(1) - desired_state(1);
state(10) = state_vec(1) - desired_state(1);
state(11) = state_vec(1) - desired_state(1);
state(12) = state_vec(1) - desired_state(1);

xt = transpose(state_vec);
disp(xt);
disp(K)
controlvec = K * xt;
end

