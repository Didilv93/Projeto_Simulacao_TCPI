function [freq, array_direction] = getArrayVelocityByDisplacement(errors)
    % Dimensões do robô
    L = (3.5708e-01)/2 - (9.0000e-02)/2; % Distância do centro de massa ao eixo traseiro ou dianteiro
    l = (1.6204e-01)/2 - (9.0000e-02)/2; % Distância do centro de massa ao entre rodas lateirais
    r = (9.0000e-02)/2; % Acredito que seja o raio da roda
    
    %Velocidades
    VX = errors(1);
    VY = errors(2);
    VW = errors(3);


    %Matriz de conversão
    %Matriz de Entradas
    Entrada=[VY; VX; VW];
    %Matriz intermediaria
    M=[1 -1 -(L+l); 1 1 (L+l); 1 1 -(L+l); 1 -1 (L+l)];

    %Matriz das Saidas
    %Calculo das Saidas
    v_ang = fix((1/r)*M*Entrada); %Saidas=[W1;W2;W3,W4];
    array_rotations = [0; 0; 0; 0];
    for i = 1:4
        if(v_ang(i) == 0)
            array_rotations(i) = 0;
        else
            array_rotations(i) = v_ang(i)/abs(v_ang(i));
        end
    end
    
    freq = abs(v_ang);
    array_direction = array_rotations;
end