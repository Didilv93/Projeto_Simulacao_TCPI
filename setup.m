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
setupState = TYPES_SETUP_STATES(2);

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

AMP_ERROR_ANGLE = 0;

if (clientID>-1)
    disp('Connected to remote API server');
    sim.simxAddStatusbarMessage(clientID,'Simulacao com o Matlab iniciada!',sim.simx_opmode_oneshot);
    
    [rolling_rl, slipping_rl, wheel_rl, ...
    rolling_rr, slipping_rr, wheel_rr, ...
    rolling_fl, slipping_fl, wheel_fl, ...
    rolling_fr, slipping_fr, wheel_fr] = sysCall_init(sim, clientID);

    while true
        % Leitura da posição do robô em relação ao centro do cenário
        % Leitura da posição das tags de destino em relação ao centro do cenário
        [sys_motor] = getMotorSys();
        [positionRobo, eulerAnglesRobo] = getPositionRobo(sim,clientID);
        [positionTag, eulerAnglesTag] = getPositionTags(sim,clientID);
        
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
            setupState = 'FIM DA SIMULACAO';
            
        case TYPES_SETUP_STATES(2)
            errors_xyz = positionTag{1} - positionRobo;
            error_w = (eulerAnglesTag{1}(3) - eulerAnglesRobo(3))*AMP_ERROR_ANGLE;

            [freq, array_direction] = getArrayVelocityByDisplacement([errors_xyz(1); errors_xyz(2); error_w(1)]);

            [sq_wav, t] = generatePWMSignal(freq, array_direction);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
        
            if freq(1) == 0 && freq(2) == 0 && freq(3) == 0 && freq(4) == 0
                setupState = TYPES_SETUP_STATES(3);
            end
            
        case TYPES_SETUP_STATES(3)
            errors_xyz = positionTag{2} - positionRobo;
            error_w = (eulerAnglesTag{2}(3) - eulerAnglesRobo(3))*AMP_ERROR_ANGLE;

            [freq, array_direction] = getArrayVelocityByDisplacement([errors_xyz(1); errors_xyz(2); error_w(1)]);

            [sq_wav, t] = generatePWMSignal(freq, array_direction);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
        
            if freq(1) == 0 && freq(2) == 0 && freq(3) == 0 && freq(4) == 0
                setupState = TYPES_SETUP_STATES(4);
            end
        case TYPES_SETUP_STATES(4)
            errors_xyz = positionTag{3} - positionRobo;
            error_w = (eulerAnglesTag{3}(3) - eulerAnglesRobo(3))*AMP_ERROR_ANGLE;

            [freq, array_direction] = getArrayVelocityByDisplacement([errors_xyz(1); errors_xyz(2); error_w(1)]);

            [sq_wav, t] = generatePWMSignal(freq, array_direction);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
        
            if freq(1) == 0 && freq(2) == 0 && freq(3) == 0 && freq(4) == 0
                setupState = TYPES_SETUP_STATES(5);
                pause(5);
            end
        case TYPES_SETUP_STATES(5)
            errors_xyz = positionTag{4} - positionRobo;
            error_w = (eulerAnglesTag{4}(3) - eulerAnglesRobo(3))*AMP_ERROR_ANGLE;

            [freq, array_direction] = getArrayVelocityByDisplacement([errors_xyz(1); errors_xyz(2); error_w(1)]);

            [sq_wav, t] = generatePWMSignal(freq, array_direction);
            [speedVector] = getInnerMesh(sys_motor, sq_wav, t);
            setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,...
            speedVector(1),speedVector(2),speedVector(3));
                    
            if freq(1) == 0 && freq(2) == 0 && freq(3) == 0 && freq(4) == 0
                setupState = 'FIM DA SIMULACAO';
            end
        otherwise
            disp(setupState);
            break
        end
    end
    
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended')