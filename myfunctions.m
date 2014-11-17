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


[U1_2, U2_2, U3_2, U4_2, omega] = omegasquared(U1, U2, U3, U4);

[phidotdot, thetadotdot, psidotdot] = Angles(phidot, thetadot, psidot, U2_2, omega, U4_2, U3_2);

phidot = diff(phidotdot);
thetadot = diff(thetadotdot);
psidot = diff(psidotdot);

phi = phi + phidot;
theta = theta + thetadot;
psi = psi + psidot;

[xdotdot, ydotdot, zdotdot] = Displacement(phi, theta, psi, U1_2);

xdot = diff(xdotdot);
ydot = diff(ydotdot);
zdot = diff(zdotdot);

x = x + xdot;
y = y + ydot;
z = z + zdot;

end