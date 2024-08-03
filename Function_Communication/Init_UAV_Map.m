%*********************************************************************
% Discription:  Initialize UAVs' search information by ground station
% input:        UAV                 UAVs structure array
% input:        GS                  Ground station information
% putput:       UAV                 UAVs structure array
%*********************************************************************

function UAV=Init_UAV_Map(UAV,GS)
[~,uav_num]=size(UAV);
for k=1:uav_num
    UAV(k).proba=GS.probability;        % Update target existance probability matrix
    UAV(k).uncer=GS.uncertainty;        % Update environment uncertainty matrix
    UAV(k).obser=GS.observation;        % Update observation information matrix
end
end
