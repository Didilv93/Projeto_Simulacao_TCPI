clear all;
close all;
clc;

sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    sim.simxAddStatusbarMessage(clientID,'Simulacao com o Matlab iniciada!',sim.simx_opmode_oneshot);
    
    [rolling_rl, slipping_rl, wheel_rl, ...
    rolling_rr, slipping_rr, wheel_rr, ...
    rolling_fl, slipping_fl, wheel_fl, ...
    rolling_fr, slipping_fr, wheel_fr] = sysCall_init(sim, clientID);

    while true
        setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,0,0.5,0);
        pause(10);
        setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,0,0,0.5);
    end
    
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended')