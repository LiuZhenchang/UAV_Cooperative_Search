%*****************************************************************
% Discription:  Initialize targets
% input:        UAV                 UAVs structure array
% input:        map                 Map structure
%*****************************************************************

function TAR=Init_TAR(TAR,map)
TAR(1).x=515*2;                                 % Set initial x-coordinate
TAR(1).y=330*2;                                 % Set initial y-coordinate
TAR(1).xg=ceil(TAR(1).x/map.grid_size);         % Calculate grid x-coordinate
TAR(1).yg=ceil(TAR(1).y/map.grid_size);         % Calculate grid y-coordinate
TAR(1).vel=10;                                  % Set initial velocity
TAR(1).yaw=0;                                   % Set initial yaw angle

TAR(2).x=485*2;
TAR(2).y=50*2;
TAR(2).xg=ceil(TAR(2).x/map.grid_size);   
TAR(2).yg=ceil(TAR(2).y/map.grid_size);   
TAR(2).vel=10;
TAR(2).yaw=0;  

TAR(3).x=565*2;
TAR(3).y=150*2;
TAR(3).xg=ceil(TAR(3).x/map.grid_size);   
TAR(3).yg=ceil(TAR(3).y/map.grid_size);   
TAR(3).vel=10;
TAR(3).yaw=0;    

TAR(4).x=435*2;
TAR(4).y=230*2;
TAR(4).vel=10;
TAR(4).xg=ceil(TAR(4).x/map.grid_size);   
TAR(4).yg=ceil(TAR(4).y/map.grid_size);   
TAR(4).yaw=0;   

TAR(5).x=215*2;
TAR(5).y=330*2;
TAR(5).xg=ceil(TAR(5).x/map.grid_size);   
TAR(5).yg=ceil(TAR(5).y/map.grid_size);   
TAR(5).vel=10;
TAR(5).yaw=0;    
end