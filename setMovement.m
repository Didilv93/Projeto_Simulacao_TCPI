function [] = setMovement(sim,clientID,wheel_rl,wheel_rr,wheel_fl,wheel_fr,forwBackVel,leftRightVel,rotVel)
    %% Apply the desired wheel velocities:
    sim.simxSetJointTargetVelocity(clientID,wheel_rl,-forwBackVel-leftRightVel-rotVel,sim.simx_opmode_blocking);
    sim.simxSetJointTargetVelocity(clientID,wheel_rr,-forwBackVel+leftRightVel-rotVel,sim.simx_opmode_blocking);
    sim.simxSetJointTargetVelocity(clientID,wheel_fl,-forwBackVel-leftRightVel+rotVel,sim.simx_opmode_blocking);
    sim.simxSetJointTargetVelocity(clientID,wheel_fr,-forwBackVel+leftRightVel+rotVel,sim.simx_opmode_blocking);
end