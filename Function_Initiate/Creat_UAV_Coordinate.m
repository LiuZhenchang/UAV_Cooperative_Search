%*****************************************************************************
% Discription:  Create UAVs structure array of companion computer
% input:        property            property structure
% input:        map                 Map structure
% output:       UAV                 UAVs structure array of companion computer
%*****************************************************************************

function UAV = Creat_UAV_Coordinate(property,map)
search_coop=zeros(property.uav.footprint_length,...
    4*property.uav.num);                                    % Matrix to recored cluster search history
target_info=zeros(property.tar.num,6);                      % Matrix to record discovered targets infomation
perdict_pos_self=zeros(property.uav.foresee_num,2);         % Predictive state sequence of self UAV
perdict_pos_other=...                                       % Predictive state sequence of other UAVs
    zeros(property.uav.num*property.uav.foresee_num,2)-1e5;       

vel_min=property.uav.vel_lim(1);
vel_max=property.uav.vel_lim(2);
x_num=(map.x_max-map.x_min)/map.grid_size;                  % Grids number in x direction
y_num=(map.y_max-map.y_min)/map.grid_size;                  % Grids number in y direction

uav_stru=struct(...
    'index',0,...                                           % Index
    'x',0,'y',0,...                                         % Initial position
    'xg',0,'yg',0,...                                       % Position in grid coordinates
    'xs',0,'ys',0,...                                       % Search position
    'xsg',0,'ysg',0,...                                     % Search position in grid coordinates 
    'xr',0,'yr',0,...                                       % Reference (next time) position
    'vel',0,'acc',0,...
    'yaw',0,'yaw_r',0,...                                   % current yaw angle and reference yaw angle
    'com_distance',property.uav.com_distance,...            % Communication range
    'com_type',property.uav.com_type,...                    % Communication mode, 0 local, 1 global
    'life_span',1000,...                                    % Life span
    'map_x_min',map.x_min,...
    'map_x_max',map.x_max,...
    'map_y_min',map.y_min,...
    'map_y_max',map.y_max,...
    'map_grid_size',map.grid_size,...
    'monitor_time',0,...
    'expert_system',property.uav.expert_system,...          % Whether to use expert system, 0 no, 1 yes
    'foresee_traj',property.uav.foresee_traj,...            % Whether foresee the path of other UAVs, 0 no, 1 yes
    'foresee_num',property.uav.foresee_num,...              % Consider the pre planned steps of other UAVs when making decisions
    'perdict_pos_self',perdict_pos_self,...                 % Predictive state sequence of self UAV
    'perdict_pos_other',perdict_pos_other,...               % Predictive state sequence of other UAVs
    'APF_distance',property.uav.APF_distance,...            % Effective range of artificial potential field
    'APF_param1',property.uav.APF_param1,...                % Artificial potential field parameters miu
    'APF_param2',property.uav.APF_param2,...                % Artificial potential field parameters k
    'safe_xg',property.uav.safe_xg,...                      % The number of safe grids from the UAV to the x boundary of map
    'safe_yg',property.uav.safe_yg,...                      % The number of safe grids from the UAV to the y boundary of map
    'safe_obs',property.uav.safe_obs,...                    % Safe distance from the UAV to obstacles
    'safe_uav',property.uav.safe_uav,...                    % Safe distance between UAVs
    'seeker_type',property.uav.seeker_type,...              % Types of FOV
    'seeker_pd',property.uav.seeker_pd,...                  % Target detection probability
    'seeker_pf',property.uav.seeker_pf,...                  % Target false alarm probability
    'search_interval',property.uav.search_interval,...      % Search algorithm call interval
    'search_jump',property.uav.search_jump,...              % Jumping grid number
    'search_index',1,...                                    % The index selected in the action sequence
    'search_count',property.uav.search_count,...            % The grid number selected in the search sequence
    'search_num',0,...                                      % UAV search number
    'search_coop',search_coop,...                           % Matrix to recored cluster search history
    'search_self',zeros(600,4),...
    'history_gene_best',zeros(600,20),...
    'history_gene_length',zeros(600,1),...
    'history_grid',zeros(600,2),...
    'history_position',zeros(600,2),...
    'history_yaw',zeros(600,1),...
    'history_jump',zeros(600,1),...
    'history_target',zeros(600,1),...
    'history_calc_time',zeros(600,3),...
    'target_info',target_info,...                           % Matrix to record discovered targets infomation
    'revisit',0,...                                         % Whether to revisit discovered targets, 0 No, 1 Yes
    'w_JA1',0.8,...                                         % Weight of JA1, target existance probability
    'w_JA2',0.04,...                                        % Weight of JA2, environment uncertainty
    'w_JB',0,...                                            % Weight of JB
    'w_JC',3,...                                            % Weight of JC
    'k_wJA1',1,...                                          % Correction coefficient of JA1
    'k_wJA2',1,...                                          % Correction coefficient of JA2
    'k_wJB',0,...                                           % Correction coefficient of JB
    'k_wJC',1,...                                           % Corriecton coefficient of JC
    'E1',0,...                                              % Expert system input 1
    'E2',0,...                                              % Expert system input 2
    'E3',0,...                                              % Expert system input 3
    'd_tar',property.uav.d_tar,...                          % Target warning distance
    'd_obs',property.uav.d_obs,...                          % Obstacle warning distance
    'obj_data',zeros(4,4),...                               % Calculation results of objective function
    'proba',zeros(x_num,y_num),...                          % Matrix of target existance probability
    'uncer',zeros(x_num,y_num),...                          % Matrix of environment uncertainty
    'obser',zeros(x_num,y_num),...                          % Matrix of observation information
    'teammates',zeros(property.uav.num,3),...               % Teammates information
    'param_GA',property.GA,...                              % genetic algorithm parameters
    'decision_trend',0,...                                  % The decision trend of UAVs in the previous step
    'task',1);                                              % UAV task types

UAV(1:property.uav.num)=uav_stru;                           % Initialize UAVs structure array

if property.uav.initflag==1                                 % Initial automatically
    for i=1:property.uav.num
        UAV(i).index=i;                                     % UAV index
        UAV(i).x=map.x_min+(map.x_max-map.x_min)*rand;      % Generate the UAV x-coordinate randomly
        UAV(i).y=map.y_min+(map.y_max-map.y_min)*rand;      % Generate the UAV y-coordinate randomly
        UAV(i).xg=ceil(UAV(i).x/map.grid_size);             % Calculate UAV x-position in grid coordinate
        UAV(i).yg=ceil(UAV(i).y/map.grid_size);             % Calculate UAV y-position in grid coordinate
        UAV(i).vel=vel_min+(vel_max-vel_min)*rand;          % Generate the UAV velocity randomly
        UAV(i).yaw=-pi+2*pi*rand;                           % Generate the UAV yaw angle randomly
    end
else                                                        % Initial manually
    UAV=Init_UAV(UAV,map); 
end
end