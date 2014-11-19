function quadr(U1, U2, U3, U4)

global x y z phi theta psi xdot ydot zdot phidot thetadot psidot;
global timestep;

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

end