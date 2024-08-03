%***********************************************************************************************************
% Discription:  Realize data transmission between companion computers and controller
% input:        uav_coor            UAVs structure array in companion computer
% input:        uav_cont            UAVs structure array in controller
% input:        flag                Transmission direction, 1 companion computer to controller, 0 vice versa
% output:        uav                UAV structure array
%***********************************************************************************************************

function uav = Trans_Mesg(uav_coor,uav_cont,flag)
% Transmission data from companion computer to controller
if flag==1
    uav_cont.xr=uav_coor.xr;                    
    uav_cont.yr=uav_coor.yr;
    uav_cont.yaw_r=uav_coor.yaw_r;
    uav=uav_cont;

% Transmission data from controller yo companion computer
else
    uav_coor.x=uav_cont.x;
    uav_coor.y=uav_cont.y;
    uav_coor.xg=uav_cont.xg;
    uav_coor.yg=uav_cont.yg;
    uav_coor.vel=uav_cont.vel;
    uav_coor.yaw=uav_cont.yaw;
    uav=uav_coor;
end
end

