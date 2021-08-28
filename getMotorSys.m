function [sys] = getMotorSys(Ts)
    A = [-0.1003 2.0469; -0.0001 -0.0659];
    B = [0; 46.2963];
    C = [0 1];
    D = 0;

    Gs = ss(A,B,C,D);

    Kp = 1.2*42/0.75;
    Ki = 1/(2*0.75);
    Kd = 0.5*0.75;

    if nargin == 0
        C = pid(Kp,Ki,Kd, 1);
        sys = feedback(Gs*C,1);
    else
        Gz = c2d(Gs,Ts);
        C = pid(Kp,Ki,Kd, 1, Ts);
        sys = feedback(Gz*C,1);
    end
end