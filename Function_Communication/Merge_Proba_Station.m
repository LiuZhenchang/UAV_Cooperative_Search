%***************************************************************************
% Discription:  Merging the target existance probability of UAV
% input:        uav1                UAV structure
% input:        proba               Target existance probability matrix
% output:       proba               Merged target existance probability matrix
%***************************************************************************

function proba = Merge_Proba_Station(proba,uav)
uav_temp=uav;                                   
uav_temp.x=uav.xs;
uav_temp.y=uav.ys;
uav_temp.xg=uav.xsg;
uav_temp.yg=uav.ysg;
[cover_grid,cover_num]=Seeker(uav_temp);        % Obtain the grids covered by the FOV of uav
for i=1:cover_num
    xg=cover_grid(i,1);                         % Obtain the x-direction position of the grid
    yg=cover_grid(i,2);                         % Obtain the y-direction position of the grid
    proba(xg,yg)=uav_temp.proba(xg,yg);         % Integrate the target existance probability of the UAV
end
end

