clear;
close all;
clc;

%% Configurações dos motores DC
%Ts = 0.4;

Bm = 4.17*10^-6;
J = 1.3*10^-6;
Kt = 0.02661;
Kg = 0.02661;
La = 10.3632*10^-3;
Ra = 15.24;

L = 1;
l = 1;
r = 1;


%A = [(-Bm/J) Kt/J; (-Kg/La) (-Ra/La)];
A = [-0.1003 2.0469; -0.0001 -0.0659];
%B = [0; 1/La];
B = [0; 46.2963];
C = [0 1];
D = 0;

sys = ss(A,B,C,D);
%step(sys);

sim('motor_DC.slx');
w1_signal = ans.yout.get('w1');
w2_signal = ans.yout.get('w2');
w3_signal = ans.yout.get('w3');
w4_signal = ans.yout.get('w4');
w1 = w1_signal.Values.Data;
w2 = w2_signal.Values.Data;
w3 = w3_signal.Values.Data;
w4 = w4_signal.Values.Data;

teste = ;

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

    for i = length(w1_signal.Values.Time)
        output_matrix = r/4*[1 1 1 1; -1 1 1 -1; -1/(L+l) 1/(L+l) -1/(L+l) 1/(L+l)]...
            * [w1(i,1); w2(i,1); w2(i,1); w2(i,1)];
        
        nu_1 = output_matrix(1,1);
        nu_2 = output_matrix(2,1);
        w = output_matrix(3,1);
        setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,nu_1,nu_2,w);
    end

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