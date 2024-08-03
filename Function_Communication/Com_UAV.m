%*****************************************************************
% Discription:  Under the constraint of communication range, 
%               achieve mutual communication function between UAVs
% input:        i                   Current UAV index
% input:        uav                 UAV structure
% input:        UAV                 UAVs structure array
% putput:       uav                 UAV structure
%*****************************************************************

function uav=Com_UAV(i,uav,UAV)

[~,uav_num]=size(UAV);                                      % Obtain UAVs number
member=0;                                                   % UAVs number within communication range
for n=1:uav_num
    if n==i
        continue;                                           % n=i repersents self
    end
    d=sqrt((UAV(i).x-UAV(n).x)^2+(UAV(i).y-UAV(n).y)^2);    % Calculate the distance between UAVs

    %% Information transmission begins when the distance between UAVs is less than the communication range
    if d<UAV(n).com_distance
        member=member+1;
        uav.teammates(member,1)=n;                          % Recored the teammate index
        uav.teammates(member,2)=UAV(n).x;                   % Recored the teammate's x-coordinate
        uav.teammates(member,3)=UAV(n).y;                   % Recored the teammate's y-coordinate
        [uav,update_info]=Update_Search_Coop(uav,UAV(n));
        if uav.task==1
            uav=Merge_Search_Info(uav,update_info);
        end
    end
end
end

