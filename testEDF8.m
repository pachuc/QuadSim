
start = [0 0 0 1.23*10^-7 1.95*10^-7 -0.108];
desired = [0 0 0 0 0 0];
runtime = 100;
timestep = 1;

obj = EDF8();
string = obj.setStartState(start);
disp(string);
string = obj.setDesiredState(desired);
disp(string);
string = obj.setTime(runtime, timestep);
disp(string);
obj.setControl(@edfLQR);
results = obj.startSim();
format shortG;
disp(results);