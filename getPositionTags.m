function [positionTag] = getPositionTags(sim,clientID)
    positionTag = {0, 0, 0, 0};
    [~,objectHandleNumber_1]=sim.simxGetObjectHandle(clientID,'tag_1',sim.simx_opmode_blocking);
    [~,objectHandleNumber_2]=sim.simxGetObjectHandle(clientID,'tag_2',sim.simx_opmode_blocking);
    [~,objectHandleNumber_3]=sim.simxGetObjectHandle(clientID,'tag_3',sim.simx_opmode_blocking);
    [~,objectHandleNumber_4]=sim.simxGetObjectHandle(clientID,'tag_4',sim.simx_opmode_blocking);
    [~,objectReferenceHandleNumber]=sim.simxGetObjectHandle(clientID,'Robo',sim.simx_opmode_blocking);
    [~, position_1] = sim.simxGetObjectPosition(clientID, objectHandleNumber_1,objectReferenceHandleNumber, sim.simx_opmode_blocking);
    [~, position_2] = sim.simxGetObjectPosition(clientID, objectHandleNumber_2,objectReferenceHandleNumber, sim.simx_opmode_blocking);
    [~, position_3] = sim.simxGetObjectPosition(clientID, objectHandleNumber_3,objectReferenceHandleNumber, sim.simx_opmode_blocking);
    [~, position_4] = sim.simxGetObjectPosition(clientID, objectHandleNumber_4,objectReferenceHandleNumber, sim.simx_opmode_blocking);
    positionTag{1} = position_1;
    positionTag{2} = position_2;
    positionTag{3} = position_3;
    positionTag{4} = position_4;
end