%*****************************************************************
% Discription:  Create targets structure array
% input:        property_tar        targets property structure
% output:       tar                 targets structure array
%*****************************************************************

function TAR = Creat_TAR(property_tar,map)

x_min=property_tar.x_lim(1);
x_max=property_tar.x_lim(2);
y_min=property_tar.x_lim(1);
y_max=property_tar.x_lim(2);
vel_min=property_tar.vel_lim(1);
vel_max=property_tar.vel_lim(2);
tar_stru=struct(...
    'x',0,'y',0,'xg',0,'yg',0,...                   % Define tar structure
    'vel',0,'acc',0,...
    'yaw',0,'yaw_rate',0,'flag',0);
TAR(1:property_tar.num)=tar_stru;                   % Initialize targets structure array

if property_tar.initflag==1                         % Initial automatically
    for i=1:property_tar.num
        TAR(i).x=x_min+(x_max-x_min)*rand;          % Generate the target x-coordinate randomly
        TAR(i).y=y_min+(y_max-y_min)*rand;          % Generate the target y-coordinate randomly
        TAR(i).xg=ceil(TAR(i).x/map.grid_size);     % Calculate target x-position in grid coordinate
        TAR(i).yg=ceil(TAR(i).y/map.grid_size);     % Calculate target y-position in grid coordinate
        TAR(i).vel=vel_min+(vel_max-vel_min)*rand;  % Generate the target velocity randomly
        TAR(i).yaw=-pi+2*pi*rand;                   % Generate the target yaw angle randomly
    end
else                                                % Initial manually
    TAR=Init_TAR(TAR,map);
end
end