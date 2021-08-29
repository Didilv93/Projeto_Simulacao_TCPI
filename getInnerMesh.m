function [speedVector] = getInnerMesh(sys_motor, sq_wav, t)
    %% Dimensões do robô
    L = (3.5708e-01)/2 - (9.0000e-02)/2; % Distância do centro de massa ao eixo traseiro ou dianteiro
    l = (1.6204e-01)/2 - (9.0000e-02)/2; % Distância do centro de massa ao entre rodas lateirais
    r = (9.0000e-02)/2; % Acredito que seja o raio da roda
    %% Malha Interna
    freq_rodas = zeros(4,1);

    for i = 1: 4
        simu_sys = lsim(sys_motor, sq_wav{i}, t);
        [freq] = getSysFrequency(simu_sys);
        freq_rodas(i,1) = freq;
    end

    speedVector = r/4*[1 1 1 1; -1 1 1 -1; -1/(L+l) 1/(L+l) -1/(L+l) 1/(L+l)] * freq_rodas;
end