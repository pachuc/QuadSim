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