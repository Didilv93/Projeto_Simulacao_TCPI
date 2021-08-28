clear;
close all;
clc;

%% Guia para vetor de velocidades angulares
%  __                         % __
% |   roda frontal esquerda   %   |
% |                           %   |
% |   roda frontal direita    %   |
% |                           %   |
% |   roda traseira esquerda  %   |
% |                           %   |
% |__ roda traseira direita   %  _|

% !IMPORTANTE!  ROTAÇÕES NA ROA SENTIDO HORÁRIO DEVEM TER SINAIS PWM
% NEGATIVOS E NO ANT-HORÁRIO, POSIIVOS!

%% Dimensões do robô
L = (3.5708e-01)/2 - (9.0000e-02)/2; % Distância do centro de massa ao eixo traseiro ou dianteiro
l = (1.6204e-01)/2 - (9.0000e-02)/2; % Distância do centro de massa ao entre rodas lateirais
r = (9.0000e-02)/2; % Acredito que seja o raio da roda

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
        %% Sinal PWM Entrada (VAMOS SUBISTITUIR ESSE TREXO PELA ATUAÇÃO DA MALHA EXTERNA, O SINAL DEVE SER PWM DE ZERO
        % À +- 3.3 V, NEGATIVO PARA GIRAR A RODA NO SENTIDO HORÁRIO, POSITIVO PARA ANTI-HORÁRIO)
        freq_wav=10/0.2;
        offset={-3.3/2; 0; -3.3/2; 0};
        amp={3.3/2; 0; 3.3/2; 0};
        t=0:0.0001:1;
        sq_wav={offset{1}+amp{1}*square(2*pi*freq_wav.*t), offset{2}+amp{2}*square(2*pi*freq_wav.*t), ...
            offset{3}+amp{3}*square(2*pi*freq_wav.*t), offset{4}+amp{4}*square(2*pi*freq_wav.*t)};

        %% Malha Interna

        [sys_motor] = getMotorSys();
        [num,den] = tfdata(sys_motor);
        arrayNum = cell2mat(num);
        arrayDen = cell2mat(den);
        sys_motor_t = poly2sym(arrayNum,s)/poly2sym(arrayDen,s);

        freq_rodas = zeros(4,1);

        for i = 1: 4
            simu_sys = lsim(sys_motor, sq_wav{i}, t);
            [freq] = getSysFrequency(simu_sys);
            freq_rodas(i,1) = freq;
        end

        Vetor_deslocamento = r/4*[1 1 1 1; -1 1 1 -1; -1/(L+l) 1/(L+l) -1/(L+l) 1/(L+l)] * freq_rodas;

        %plot(t,sq_wav); ylabel('w pwm(t)'); xlabel('t'); grid; figure; hold on;
        %plot(t,simu_sys); ylabel('w motor(t)'); xlabel('t'); grid;

        setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
        Vetor_deslocamento(1),Vetor_deslocamento(2),Vetor_deslocamento(3));
    
    end
    
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended')