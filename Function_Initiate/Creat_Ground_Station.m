%*****************************************************************
% Discription:  Generate ground station structure
% input:        map                 Map structure
% input:        tar                 Targets structure array
% output:       GS                  Ground station information
%*****************************************************************

function GS=Creat_Ground_Station(time,property,map,TAR)
% Initialize ground station structure
x_num=(map.x_max-map.x_min)/map.grid_size;                      % Grids number in x direction
y_num=(map.y_max-map.y_min)/map.grid_size;                      % Grids number in y direction

GS.probability=zeros(x_num,y_num);                              % Matrix of target existance probability
GS.uncertainty=zeros(x_num,y_num);                              % Matrix of environment uncertainty
GS.observation=zeros(x_num,y_num);                              % Matrix of observation information

GS.search_interval=1;
GS.search_count=GS.search_interval/time.delta;
GS.search_num=0;

max_search_num=...
    ceil(time.step_max*time.delta/GS.search_interval);          % Maximum communication times of ground station
GS.search_stamp=zeros(max_search_num,1);                        % Search infomation communication timestamp
GS.search_grid=zeros(property.uav.num,2,max_search_num)-1;      % Record of UAVs search position in grid coordinates
GS.proba=zeros(x_num,y_num,max_search_num);                     % Matrix to save target existance probability in different timestamps
GS.uncer=zeros(x_num,y_num,max_search_num);                     % Matrix to save environment uncertainty in different timestamps
GS.obser=zeros(x_num,y_num,max_search_num);                     % Matrix to save observation information in different timestamps
GS.obj_data=zeros(4,4,max_search_num,property.uav.num);         % Matrix to save objective function value in different timestamps
GS.UAV_path=zeros(property.uav.num,2,time.step_max);            % Matrix to save UAVs path in different timestamps
GS.TAR_path=zeros(property.tar.num,2,time.step_max);            % Matrix to save targets path in different timestamps

% Calculate initial map information
c_n=0.3;                                                        % Peak height of target existance probability
v_n=50;                                                         % Peak width of target existance probability
[~,target_num]=size(TAR);                                       % Obtain targets number
for i=1:x_num
    for j=1:y_num
        for k=1:target_num
            x_t=TAR(k).x;
            y_t=TAR(k).y;
            x=map.x_min+(i-0.5)*map.grid_size;
            y=map.y_min+(j-0.5)*map.grid_size;
            % Initialize the target existence probability using Gaussian distribution function
            GS.probability(i,j) = GS.probability(i,j)+...
                c_n*exp(-((x-x_t)^2+(y-y_t)^2)/v_n^2);
        end
        GS.uncertainty(i,j)=1;                                  % The initial environmental uncertainty is 1
    end
end

end
