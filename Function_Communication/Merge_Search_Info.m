%************************************************************************************************
% Discription:  Update the search information of the UAV itself based on the communication content
% input:        uav                 UAV structure
% input:        update_info         Data update time of UAVs
% output:       uav                 UAV structure
%************************************************************************************************

function uav = Merge_Search_Info(uav,update_info)

[uav_num,~]=size(update_info);
[tar_num,~]=size(uav.target_info);
[row_num,~]=size(uav.search_coop);

xg_save=uav.xg;
yg_save=uav.yg;
x_save=uav.x;
y_save=uav.y;
yaw_save=uav.yaw;

% Traverse the search results of each drone in its own search_comop matrix
for i=1:uav_num
    if i==uav.index
        continue;
    end

    % If the information of the UAV is not updated, no further calculations will be performed
    if max(uav.search_coop(:,4*(i-1)+1))<update_info(i,1)||...
            update_info(i,2)==0
        continue;
    end

    % Traverse every search information in the historical record 
    for j=1:row_num
        % If the record time is before the latest update time, do not repeat the update
        if uav.search_coop(j,4*(i-1)+1)<=update_info(i,1)
            continue;
        end
        % Assign the information from search_comop to the UAV
        uav.xg=uav.search_coop(j,4*(i-1)+2);
        uav.yg=uav.search_coop(j,4*(i-1)+3);
        uav.x=uav.map_x_min+(uav.xg-0.5)*uav.map_grid_size;
        uav.y=uav.map_y_min+(uav.yg-0.5)*uav.map_grid_size;

        % Pretend that a drone is searching and updating search information in the area
        uav.yaw=uav.search_coop(j,4*(i-1)+4);
        [cover_grid,cover_num]=Seeker(uav);
        for k=1:cover_num
            xg=cover_grid(k,1);
            yg=cover_grid(k,2);
            uav.uncer(xg,yg)=0.5*uav.uncer(xg,yg);
            uav.obser(xg,yg)=uav.obser(xg,yg)+1;

            % Check if there is a target in the grid
            for m=1:tar_num
                tar_flag=0;                                     % 0 no target，1 has target
                tar_grid_x=uav.target_info(m,2);                % Calculate target x position in grid coordinate
                tar_grid_y=uav.target_info(m,3);                % Calculate target y position in grid coordinate
                if tar_grid_x==xg&&tar_grid_y==yg
                    tar_flag=1;
                    break
                end
            end

            pd=uav.seeker_pd;                                   % Target detection probability
            pf=uav.seeker_pf;                                   % Target false alarm probability

            if tar_flag
                uav.proba(xg,yg)=Update_Probability(...
                    uav.proba(xg,yg),pd,pf,1);
            else
                uav.proba(xg,yg)=Update_Probability(...
                    uav.proba(xg,yg),pd,pf,0);
            end
        end
    end
end

uav.xg=xg_save;
uav.yg=yg_save;
uav.x=x_save;
uav.y=y_save;
uav.yaw=yaw_save;
end

