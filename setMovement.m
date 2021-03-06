function [] = setMovement(sim,clientID,rolling_rl,rolling_rr,rolling_fl,rolling_fr,forwBackVel,leftRightVel,rotVel)
    %% Apply the desired rolling velocities:
    sim.simxSetJointTargetVelocity(clientID,rolling_fl,-forwBackVel-leftRightVel-rotVel,sim.simx_opmode_blocking);
    sim.simxSetJointTargetVelocity(clientID,rolling_rl,-forwBackVel+leftRightVel-rotVel,sim.simx_opmode_blocking);
    sim.simxSetJointTargetVelocity(clientID,rolling_rr,-forwBackVel-leftRightVel+rotVel,sim.simx_opmode_blocking);
    sim.simxSetJointTargetVelocity(clientID,rolling_fr,-forwBackVel+leftRightVel+rotVel,sim.simx_opmode_blocking);
end