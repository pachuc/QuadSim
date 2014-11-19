function [U1_2, U2_2, U3_2, U4_2, omega] = Omegasquared(U1, U2, U3, U4)
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