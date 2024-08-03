%*******************************************************************************************
% Discription:  Update the search information matrix based on the current search results
% input:        uav                 UAV structure
% input:        time                time structure
% input:        tar_find            Discovered target Information
% output:       uav                 UAVs structure
%*******************************************************************************************

function uav = Update_Search_Self(uav,time,tar_find)
%% Update the current search information to the search_delf matrix
uav.search_self(uav.search_num,1)=time.step;
uav.search_self(uav.search_num,2)=uav.xsg;
uav.search_self(uav.search_num,3)=uav.ysg;
uav.search_self(uav.search_num,4)=uav.yaw;
[row_num,~]=size(uav.search_coop);

%% Update self search information in the search_comop matrix
if uav.search_num<row_num
    uav.search_coop(uav.search_num,4*(uav.index-1)+1)=time.step;
    uav.search_coop(uav.search_num,4*(uav.index-1)+2)=uav.xsg;
    uav.search_coop(uav.search_num,4*(uav.index-1)+3)=uav.ysg;
    uav.search_coop(uav.search_num,4*(uav.index-1)+4)=uav.yaw;
else
    for i=1:row_num
        uav.search_coop(i,4*(uav.index-1)+1)=...
            uav.search_self(uav.search_num-row_num+i,1);
        uav.search_coop(i,4*(uav.index-1)+2)=...
            uav.search_self(uav.search_num-row_num+i,2);
        uav.search_coop(i,4*(uav.index-1)+3)=...
            uav.search_self(uav.search_num-row_num+i,3);
        uav.search_coop(i,4*(uav.index-1)+4)=...
            uav.search_self(uav.search_num-row_num+i,4);
    end
end

%% Update discovered target information
[row_num,~]=size(tar_find);
for k=1:row_num
    if tar_find(k,1)==0                                     % If the current target number is 0, terminate the loop
        break;
    else
        uav.target_info(tar_find(k,1),1)=time.step;         % Record target discovered time
        uav.target_info(tar_find(k,1),2)=tar_find(k,2);     % Record target x-position in grid Coordinate
        uav.target_info(tar_find(k,1),3)=tar_find(k,3);     % Record target y-position in grid Coordinate
    end
end
end

