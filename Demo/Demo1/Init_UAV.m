%*****************************************************************
% Discription:  Initialize UAVs
% input:        UAV                 UAVs structure array
% input:        map                 Map structure
%*****************************************************************

function UAV=Init_UAV(UAV,map)
UAV(1).index=1;
UAV(1).x=100*2;                                             % Set initial x-coordinate
UAV(1).y=50*2;                                              % Set initial y-coordinate
UAV(1).xg=ceil(UAV(1).x/map.grid_size);                     % Calculate grid x-coordinate
UAV(1).yg=ceil(UAV(1).y/map.grid_size);                     % Calculate grid y-coordinate
UAV(1).vel=25;                                              % Set initial velocity
UAV(1).yaw=0;                                               % Set initial yaw angle
UAV(1).com_type=0;                                          % Communication mode, 0 local, 1 global
UAV(1).task=1;                                              % Set task, 0 Relay Node£¬1 search


UAV(2).index=2;
UAV(2).x=100*2;
UAV(2).y=150*2;
UAV(2).xg=ceil(UAV(2).x/map.grid_size);        
UAV(2).yg=ceil(UAV(2).y/map.grid_size);   
UAV(2).vel=25;
UAV(2).yaw=0;
UAV(2).com_type=0;                                     
UAV(2).task=1;                                 


UAV(3).index=3;
UAV(3).x=600*2;
UAV(3).y=350*2;
UAV(3).xg=ceil(UAV(3).x/map.grid_size);        
UAV(3).yg=ceil(UAV(3).y/map.grid_size);   
UAV(3).vel=25;
UAV(3).yaw=pi;
UAV(3).com_type=0;                                     
UAV(3).task=1; 

UAV(4).index=4;
UAV(4).x=750*2;
UAV(4).y=100*2;
UAV(4).xg=ceil(UAV(4).x/map.grid_size);        
UAV(4).yg=ceil(UAV(4).y/map.grid_size);   
UAV(4).vel=25;
UAV(4).yaw=pi;
UAV(4).life_span=100;
UAV(4).com_type=0;                                     
UAV(4).task=1; 

end