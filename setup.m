clear all;
close all;
clc;

%% Configurações dos motores DC
Bm = 4.17*10^-6;
J = 1.3*10^-6;
Kt = 0.02661;
Kg = 0.02661;
La = 10.3632*10^-3;
Ra = 15.24;


%A = [(-Bm/J) Kt/J; (-Kg/La) (-Ra/La)];
A = [-0.1003 2.0469; -0.0001 -0.0659];
%B = [0; 1/La];
B = [0; 46.2963];
C = [0 1];
D = 0;

sys = ss(A,B,C,D);
step(sys);

L = 0.75;
T = 42;

%% Simulação, CoppeliaSim
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
        %pause(10);
        %setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,0,0,0.5);
    end
    
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended')