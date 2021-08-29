function [sq_wav, t] = generatePWMSignal(freq, array_rotations)
        %% Sinal PWM Entrada (O SINAL DEVE SER PWM DE ZERO À +- 3.3 V, NEGATIVO PARA GIRAR A RODA NO SENTIDO HORÁRIO, POSITIVO PARA ANTI-HORÁRIO)
        
        gain = 3.3/2;
        offset = gain*array_rotations;
        amp=gain*abs(array_rotations);
        t=0:0.0001:10;
        sq_wav={offset(1)+amp(1)*square(2*pi*freq(1).*t), offset(2)+amp(2)*square(2*pi*freq(2).*t), ...
            offset(3)+amp(3)*square(2*pi*freq(3).*t), offset(4)+amp(4)*square(2*pi*freq(4).*t)};
        %for i = 1: 4
            %plot(t,sq_wav{i}); ylabel('w pwm(t)'); xlabel('t');
        %end
end