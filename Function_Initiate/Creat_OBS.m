%*****************************************************************
% Discription:  Create obstables structure array
% input:        property_obs        Obstacles property structure
% output:       OBS                 Obstacles structure array
%*****************************************************************
function OBS = Creat_OBS(property_obs)
x_min=property_obs.x_lim(1);
x_max=property_obs.x_lim(2);
y_min=property_obs.x_lim(1);
y_max=property_obs.x_lim(2);
r_min=property_obs.r_lim(1);
r_max=property_obs.r_lim(2);
obs_stru=struct(...                                 % Define obs structure  
    'x',0,'y',0,...                                 % Obstacle position coordinate
    'psi',0,'speed','0',...                         % Obstacle heading angle and speed
    'r',0,...                                       % Obstacle radius
    'history_position',zeros(600,2));

OBS(1:property_obs.num)=obs_stru;                   % Initialize obstacles structure array

if property_obs.initflag==1                         % Initial automatically
    for i=1:property_obs.num
        OBS(i).x=x_min+(x_max-x_min)*rand;          % generate the obstacle x-coordinate randomly
        OBS(i).y=y_min+(y_max-y_min)*rand;          % generate the obstacle y-coordinate randomly
        OBS(i).r=r_min+(r_max-r_min)*rand;          % generate the obstacle radius randomly
    end
else                                                % Initial manually
    OBS=Init_OBS(OBS);
end
end
