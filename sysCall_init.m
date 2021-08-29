function [rolling_rl, slipping_rl, wheel_rl, ...
    rolling_rr, slipping_rr, wheel_rr, ...
    rolling_fl, slipping_fl, wheel_fl, ...
    rolling_fr, slipping_fr, wheel_fr] = sysCall_init(sim,clientID) 
    [~,rolling_rl]=sim.simxGetObjectHandle(clientID,'rollingJoint_rl',sim.simx_opmode_blocking);
    [~,rolling_rr]=sim.simxGetObjectHandle(clientID,'rollingJoint_rr',sim.simx_opmode_blocking);
    [~,rolling_fl]=sim.simxGetObjectHandle(clientID,'rollingJoint_fl',sim.simx_opmode_blocking);
    [~,rolling_fr]=sim.simxGetObjectHandle(clientID,'rollingJoint_fr',sim.simx_opmode_blocking);
    [~,slipping_rl]=sim.simxGetObjectHandle(clientID,'slippingJoint_rl',sim.simx_opmode_blocking);
    [~,slipping_rr]=sim.simxGetObjectHandle(clientID,'slippingJoint_rr',sim.simx_opmode_blocking);
    [~,slipping_fl]=sim.simxGetObjectHandle(clientID,'slippingJoint_fl',sim.simx_opmode_blocking);
    [~,slipping_fr]=sim.simxGetObjectHandle(clientID,'slippingJoint_fr',sim.simx_opmode_blocking);
    [~,wheel_rl]=sim.simxGetObjectHandle(clientID,'wheel_respondable_rl',sim.simx_opmode_blocking);
    [~,wheel_rr]=sim.simxGetObjectHandle(clientID,'wheel_respondable_rr',sim.simx_opmode_blocking);
    [~,wheel_fl]=sim.simxGetObjectHandle(clientID,'wheel_respondable_fl',sim.simx_opmode_blocking);
    [~,wheel_fr]=sim.simxGetObjectHandle(clientID,'wheel_respondable_fr',sim.simx_opmode_blocking);
end