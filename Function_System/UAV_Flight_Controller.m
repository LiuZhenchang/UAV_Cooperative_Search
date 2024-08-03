%*****************************************************************
% Discription:  UAV controller (Simplified Version)
% input:        uav                 UAVs structure array
% output:       uav                 UAVs structure array
%*****************************************************************
function uav = UAV_Flight_Controller(uav)
uav.x=uav.xr;
uav.y=uav.yr;
uav.yaw=uav.yaw_r;
uav.xg=ceil((uav.x-uav.map_x_min)/uav.map_grid_size);        % Calculate UAV grid x-coordinate
uav.yg=ceil((uav.y-uav.map_y_min)/uav.map_grid_size);        % Calculate UAV grid y-coordinate
end

