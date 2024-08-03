%*******************************************************************************************
% Discription:  Detect the grid within detection area and update search information
% input:        uav                 UAV structure
% input:        TAR                 Targets structure array
% output:       uav                 UAVs structure
% output:       tar_find            Discovered target Information
%*******************************************************************************************
function [uav,tar_find] = Search_Grid(uav,TAR)
[~,tar_num]=size(TAR);
tar_find=zeros(10,3);                                       % Initialize the discovered target information
tar_count=0;                                                % Initialize the number of discovered targets
uav.xs=uav.x;                                               % Record the x-coordinate of the UAV
uav.ys=uav.y;                                               % Record the y-coordinate of the UAV
uav.xsg=uav.xg;                                             % Record the UAV x-position in grid coordinate
uav.ysg=uav.yg;                                             % Record the UAV y-position in grid coordinate
[cover_grid,cover_num]=Seeker(uav);                         % Obtain grids covered by FOV

for i=1:cover_num
    xg=cover_grid(i,1);                                     % get current covered grid x-coordinate
    yg=cover_grid(i,2);                                     % get current covered grid y-coordinate
    uav.uncer(xg,yg)=0.5*uav.uncer(xg,yg);                  % update environment uncertainty of grid
    uav.obser(xg,yg)=uav.obser(xg,yg)+1;                    % update observation status of grid
    
    % Check if there is a target in the grid
    for j=1:tar_num
        tar_flag=0;                                         % 0 has no target, 1 has a target
        tar_grid_x=TAR(j).xg;                               % Calculate target x-position in grid Coordinate
        tar_grid_y=TAR(j).yg;                               % Calculate target y-position in grid Coordinate
        if tar_grid_x==xg&&tar_grid_y==yg
            tar_flag=1;
            tar_count=tar_count+1;                          % Updata discovered target number
            uav.monitor_time=uav.monitor_time+1;            % Update the total monitoring time of the UAV on the target
            break
        end
    end
    
    pd=uav.seeker_pd;                                       % Detection probability
    pf=uav.seeker_pf;                                       % False alarm probability
    
    if tar_flag
        if rand<pd                                          % There is a target and it has been detected
            uav.proba(xg,yg)=Update_Probability(...         % Update target existance probability 
                uav.proba(xg,yg),pd,pf,1);
                tar_find(tar_count,1)=j;                    % Record target index  
                tar_find(tar_count,2)=xg;                   % Record target x-position in grid Coordinate
                tar_find(tar_count,3)=yg;                   % Record target y-position in grid Coordinate
        else                                                % There is a target that has not been detected
            uav.proba(xg,yg)=Update_Probability(...         % Update target existance probability 
                uav.proba(xg,yg),pd,pf,0);
        end
    else
        if rand<pf                                          % No target, but false alarmed
            uav.proba(xg,yg)=Update_Probability(...         % Update target existance probability 
                uav.proba(xg,yg),pd,pf,1);
        else                                                % No target, no target detected
            uav.proba(xg,yg)=Update_Probability(...         % Update target existance probability 
                uav.proba(xg,yg),pd,pf,0);
        end
    end
    
end
end

