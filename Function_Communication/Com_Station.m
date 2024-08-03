%*****************************************************************
% Discription:  UAVs transmit search information to ground station
% input:        time                time structure
% input:        uav                 UAV structure
% input:        GS                  Ground station information
% putput:       GS                  Ground station information
%*****************************************************************

function GS=Com_Station(time,UAV,GS)
GS.search_count=GS.search_count+1;                     
[~,UAV_num]=size(UAV);

% Ground station starts to receive search information
if GS.search_count*time.delta>GS.search_interval
    GS.search_num=GS.search_num+1;
    for i=1:UAV_num
        if UAV(i).task==1
            GS.observation=GS.observation+UAV(i).obser;         % Update observation information matrix
            GS.uncertainty=...
                Merge_Uncer(GS.uncertainty,UAV(i).uncer);       % Update environment uncertainty matrix
            GS.probability=...
                Merge_Proba_Station(GS.probability,UAV(i));     % Update target existance probability matrix
            GS.search_grid(i,1,GS.search_num)=UAV(i).xs;
            GS.search_grid(i,2,GS.search_num)=UAV(i).ys;
            GS.search_yaw(i,GS.search_num)=UAV(i).yaw;
        end
    end
    GS.proba(:,:,GS.search_num)=GS.probability;                 % Save current existance probability matrix
    GS.uncer(:,:,GS.search_num)=GS.uncertainty;                 % Save current environment uncertainty matrix
    GS.obser(:,:,GS.search_num)=GS.observation;                 % Save current target existance probability matrix
    GS.search_stamp(GS.search_num,1)=time.step;                 % Save current timestamp
    GS.search_count=1;
end

% Save UAVs' path in ground station 
for i=1:UAV_num
    GS.UAV_path(i,1,time.step)=UAV(i).x;
    GS.UAV_path(i,2,time.step)=UAV(i).y;
    GS.obj_data(:,:,GS.search_num,i)=UAV(i).obj_data;
end
end

