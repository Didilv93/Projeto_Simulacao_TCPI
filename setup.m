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

ERROR_MIN_DISTANCE = 0.001;

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
        % Leitura da posição do robô em relação ao centro do cenário
        % Leitura da posição das tags de destino em relação ao centro de massa do robô 
        [sys_motor] = getMotorSys();
        [positionRobo, eulerAnglesRobo] = getPositionRobo(sim,clientID);
        [positionTag] = getPositionTags(sim,clientID);
        
        switch setupState
        case TYPES_SETUP_STATES(1)
            gain = 3.3/2;
            [sq_wav, t] = generatePWMSignal([10 10 10 10], [-gain; -gain; -gain; -gain]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
        
            [sq_wav, t] = generatePWMSignal([10 10 10 10], [gain; gain; gain; gain]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
        
            [sq_wav, t] = generatePWMSignal([10 10 10 10], [gain; -gain; -gain; gain]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
        
            [sq_wav, t] = generatePWMSignal([10 10 10 10], [-1; 1; 1; -1]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
            
            [sq_wav, t] = generatePWMSignal([10 10 10 10], [-1; 1; -1; 1]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
            
            [sq_wav, t] = generatePWMSignal([10 10 10 10], [1; -1; 1; -1]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
            
            [sq_wav, t] = generatePWMSignal([10 10 10 10], [-1; 0; -1; 0]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
            
            [sq_wav, t] = generatePWMSignal([10 10 10 10], [1; 0; 1; 0]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
            
            [sq_wav, t] = generatePWMSignal([10 10 10 10], [-1; 0; 0; -1]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
                 
            [sq_wav, t] = generatePWMSignal([10 10 10 10], [1; 0; 0; 1]);
            
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            pause(5);
            
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,0,0,0);
            setupState = TYPES_SETUP_STATES(2);
        case TYPES_SETUP_STATES(2)
            error_x = positionRobo(1) - positionTag{1}(1);
            error_y = positionRobo(1) - positionTag{1}(1);
            taxas = [(100*error_x)/5 (100*error_y)/5];
            [sq_wav, t] = generatePWMSignal([taxas(1) taxas(2) taxas(1) taxas(2)], [-1; -1; -1; -1]);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
            if abs(error_x) < ERROR_MIN_DISTANCE && error_y < ERROR_MIN_DISTANCE
                setupState = TYPES_SETUP_STATES(3);
            end
        case TYPES_SETUP_STATES(3)
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,0,0,0);
            setupState = TYPES_SETUP_STATES(4);
        case TYPES_SETUP_STATES(4)
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,0,0,0);
            setupState = TYPES_SETUP_STATES(5);
        case TYPES_SETUP_STATES(5)
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,0,0,0);
            setupState = '';
        otherwise
            disp('FIM DA  ');
            break
        end
        
    
    end
    
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended')