
start = [0 0 0 0 0 0 0 0 0 0 0 0];
desired = [0 0 0 0 0 0 0 0 0 0 0 0];
runtime = 100;
timestep = 1;

obj = QuadRotor();
string = obj.setStartState(start);
disp(string);
string = obj.setDesiredState(desired);
disp(string);
string = obj.setTime(runtime, timestep);
disp(string);
obj.setControl(@quadLQR);
results = obj.startSim();
format shortG;
disp(results);