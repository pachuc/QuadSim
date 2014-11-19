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