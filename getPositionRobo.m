function [positionRobo, eulerAnglesRobo] = getPositionRobo(sim,clientID)
    [~,objectHandleNumber]=sim.simxGetObjectHandle(clientID,'Robo',sim.simx_opmode_blocking);
    [~,objectReferenceHandleNumber]=sim.simxGetObjectHandle(clientID,'Floor',sim.simx_opmode_blocking);
    [~, position] = sim.simxGetObjectPosition(clientID, objectHandleNumber,objectReferenceHandleNumber, sim.simx_opmode_blocking);
    [~, eulerAngles] = sim.simxGetObjectOrientation(clientID, objectHandleNumber,objectReferenceHandleNumber, sim.simx_opmode_blocking);
    positionRobo = position;
    eulerAnglesRobo = eulerAngles;
end