clear;
close all;
clc;

syms s

%% CONFIGURAÇÕES SETUP

TYPES_SETUP_STATES = ['TIPOS DE DESLOCAMENTOS DO ROBÔ', ...
    'PERCURSO, PRIMERIO DESTIVO',                       ...
    'PERCURSO, SEGUNDO DESTIVO',                        ...
    'PERCURSO, TERCEIRO DESTIVO',                       ...
    'PERCURSO, QUARTO DESTIVO'];
setupState = TYPES_SETUP_STATES(1);

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
        [sys_motor] = getMotorSys();
        switch setupState
        case TYPES_SETUP_STATES(1)
            [sq_wav, t] = generatePWMSignal(10, [-1; -1; -1; -1]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
        
            [sq_wav, t] = generatePWMSignal(10, [1; 1; 1; 1]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
        
            [sq_wav, t] = generatePWMSignal(10, [1; -1; -1; 1]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
        
            [sq_wav, t] = generatePWMSignal(10, [-1; 1; 1; -1]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
            
            [sq_wav, t] = generatePWMSignal(5, [-1; 1; -1; 1]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
            
            [sq_wav, t] = generatePWMSignal(5, [1; -1; 1; -1]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
            
            [sq_wav, t] = generatePWMSignal(5, [-1; 0; -1; 0]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
            
            [sq_wav, t] = generatePWMSignal(5, [1; 0; 1; 0]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
            
            [sq_wav, t] = generatePWMSignal(10, [-1; 0; 0; -1]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
                 
            [sq_wav, t] = generatePWMSignal(10, [1; 0; 0; 1]);
            
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
            
            setupState = TYPES_SETUP_STATES(2);
        case TYPES_SETUP_STATES(2)
            setupState = TYPES_SETUP_STATES(3);
        case TYPES_SETUP_STATES(3)
            setupState = TYPES_SETUP_STATES(4);
        case TYPES_SETUP_STATES(4)
            setupState = TYPES_SETUP_STATES(5);
        case TYPES_SETUP_STATES(5)
            setupState = '';
        otherwise
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,0,0,0);
            disp('FIM DA SIMULAÇÃO');
            break
        end
        
    
    end
    
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended')