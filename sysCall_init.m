function [rolling_rl, slipping_rl, wheel_rl, ...
    rolling_rr, slipping_rr, wheel_rr, ...
    rolling_fl, slipping_fl, wheel_fl, ...
    rolling_fr, slipping_fr, wheel_fr] = sysCall_init(sim,clientID) 
    [r_rl,rolling_rl]=sim.simxGetObjectHandle(clientID,'rollingJoint_rl',sim.simx_opmode_blocking);
    [r_rr,rolling_rr]=sim.simxGetObjectHandle(clientID,'rollingJoint_rr',sim.simx_opmode_blocking);
    [r_fl,rolling_fl]=sim.simxGetObjectHandle(clientID,'rollingJoint_fl',sim.simx_opmode_blocking);
    [r_fr,rolling_fr]=sim.simxGetObjectHandle(clientID,'rollingJoint_fr',sim.simx_opmode_blocking);
    [s_rl,slipping_rl]=sim.simxGetObjectHandle(clientID,'slippingJoint_rl',sim.simx_opmode_blocking);
    [s_rr,slipping_rr]=sim.simxGetObjectHandle(clientID,'slippingJoint_rr',sim.simx_opmode_blocking);
    [s_fl,slipping_fl]=sim.simxGetObjectHandle(clientID,'slippingJoint_fl',sim.simx_opmode_blocking);
    [s_fr,slipping_fr]=sim.simxGetObjectHandle(clientID,'slippingJoint_fr',sim.simx_opmode_blocking);
    [w_rl,wheel_rl]=sim.simxGetObjectHandle(clientID,'wheel_respondable_rl',sim.simx_opmode_blocking);
    [w_rr,wheel_rr]=sim.simxGetObjectHandle(clientID,'wheel_respondable_rr',sim.simx_opmode_blocking);
    [w_fl,wheel_fl]=sim.simxGetObjectHandle(clientID,'wheel_respondable_fl',sim.simx_opmode_blocking);
    [w_fr,wheel_fr]=sim.simxGetObjectHandle(clientID,'wheel_respondable_fr',sim.simx_opmode_blocking);
end