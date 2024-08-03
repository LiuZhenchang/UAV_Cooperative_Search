%**********************************************************************************************************************************
% Discription:simulation of 4 homogenous UAVs, using jumping grid method, with communication constraints and stationary obstacles
% Auhor: Zhengchang Liu
%**********************************************************************************************************************************

%% Initialize parameters
clear;
close all;
clc ;
% Define map information
map.x_min = 0;                                              % Minimum value in the x-direction of the map (meters)
map.x_max = 1600;                                           % Maximum value in the x-direction of the map (meters)
map.y_min = 0;                                              % Minimum value in the y-direction of the map (meters)
map.y_max = 800;                                            % Maximum value in the y-direction of the map (meters)
map.grid_size = 4;                                          % Square grid edge length (meters)

% Define simulation time information
time.delta = 1;                                             % Simulation step size
time.step = 0;                                              % Simulation step number
time.step_max = 300;                                        % Simulation maximum step number
time.t = 0;                                                 % Simulation time t = delta*step

% Define simulation properties
property.sim.flag_plot = 1;                                 % Whether to draw search results during the simulation process
property.sim.plot_interval = 10;                            % Drawing interval

% Define UAV cluster properties
property.uav.num = 5;                                       % Number of drones
property.uav.initflag = 0;                                  % Initialization method, 0 manual, 1 automatic
property.uav.vel_lim = [20,40];                             % speed limit
property.uav.acc_lim = [-2,2];                              % acceleration rate limit
property.uav.yaw_lim = [-pi,pi];                            % Heading angle limiting
property.uav.yaw_rate_lim = [-pi/7,pi/7];                   % Heading angular velocity limit
property.uav.PID_roll = [15,0,0];                           % PID parameters of Roll channel
property.uav.PID_pitch = [10,0,0];                          % PID parameters of pitch channel
property.uav.PID_yaw = [5,0,10];                            % PID parameters of yaw channel
property.uav.footprint_length = 100;                        % Path record length
property.uav.com_distance = 160;                            % Communication range
property.uav.com_type=0;                                    % Communication mode, 0 local, 1 global
property.uav.expert_system = 1;                             % Whether to use expert system, 0 no, 1 yes
property.uav.foresee_traj=0;                                % Whether foresee the path of other drones, 0 no, 1 yes
property.uav.foresee_num=3;                                 % Consider the pre planned steps of other UAVs when making decisions
property.uav.APF_distance = 200;                            % Effective range of artificial potential field
property.uav.APF_param1 = 0.6*10e-2;                        % Artificial potential field parameters miu
property.uav.APF_param2 = 10;                               % Artificial potential field parameters k
property.uav.d_tar = 160;                                   % Target warning distance
property.uav.d_obs = 30;                                    % Obstacle warning distance
property.uav.safe_xg = 3;                                   % The number of safe grids from the UAV to the x boundary of map
property.uav.safe_yg = 3;                                   % The number of safe grids from the UAV to the y boundary of map
property.uav.safe_obs = 40;                                 % Safe distance from the UAV to obstacles
property.uav.safe_uav = 60;                                 % Safe distance between UAVs
property.uav.seeker_type = 2;                               % Types of FOV
property.uav.seeker_pd = 0.80;                              % Target detection probability
property.uav.seeker_pf = 0.00;                              % Target false alarm probability
property.uav.search_interval = 1;                           % Search algorithm call interval
property.uav.search_jump = 7;                               % Jumping grid number, ultimately determined by the expert system
property.uav.search_count = ...
    property.uav.search_interval/time.delta;

% Define targets properties
property.tar.num = 5;                                       % target number
property.tar.initflag = 0;                                  % Initialization method, 0 manual, 1 automatic
property.tar.x_lim = [map.x_min,map.x_max];                 % Target position limitation in x dirextion
property.tar.y_lim = [map.y_min,map.y_max];                 % Target position limitation in y dirextion
property.tar.vel_lim = [20,40];                             % Target velocity limitation
property.tar.acc_lim = [-2,2];                              % Target acceleration limitation
property.tar.yaw_lim = [-pi,pi];                            % Target yaw limitation
property.tar.yaw_rate_lim = [-pi/7,pi/7];                   % Target rate limitation
property.tar.footprint_length = 400;                        % Target path record length

% Define obstacles properties
property.obs.num = 6;                                       % Obstacle number
property.obs.initflag = 0;                                  % Initialization method, 0 manual, 1 automatic
property.obs.x_lim = [map.x_min,map.x_max];                 % Obstacle position limitation in x dirextion
property.obs.y_lim = [map.y_min,map.y_max];                 % Obstacle position limitation in y dirextion
property.obs.r_lim = [40,160];                               % Obstacle radius limitation

% Define genetic algorithm parameters
property.GA.population_num = 100;                           % population number
property.GA.gene_length = 5;                                % Gene length，ultimately determined by the expert system
property.GA.mp = 0.5;                                       % mutation probability
property.GA.cp = 0.5;                                       % cross probability
property.GA.iteration = 50;                                 % maximum generation number

% Initialize object information
UAV_Coordinate = ...                                        % Initialize the UAVs structure array in companion computer
    Creat_UAV_Coordinate(property,map);
UAV_Control = ...                                           % Initialize the UAVs structure array in controller
    Creat_UAV_Control(property.uav,UAV_Coordinate,map);
TAR = Creat_TAR(property.tar,map);                          % Initialize the targets structure array
OBS = Creat_OBS(property.obs);                              % Initialize the obstacles structure array
GS = Creat_Ground_Station(time,property,map,TAR);           % Initialize the ground station
UAV_Coordinate = ...                                        % Initialize the search information
    Init_UAV_Map(UAV_Coordinate,GS);

%% Main loop
print_count=0;
plot_count=0;
for step=1:time.step_max
    time.step=step;
    print_count=print_count+1;
    if print_count==10
        fprintf('Step number = %d, finised %d percent\n',...
            step,step/time.step_max*100);
        print_count=0;
    end
    % Determine whether to draw simulation results
    if property.sim.flag_plot
        plot_count=plot_count+1;
        if plot_count==property.sim.plot_interval
            if(step>property.sim.plot_interval)
                close(F);
            end
            F = Plot_UAV_Trajectory(map,GS,UAV_Coordinate,TAR,OBS);
            drawnow;
            plot_count=0;
        end
    end

    % Perform distributed computing with each UAV as a separate entity
    for i=1:property.uav.num
        UAV_Coordinate(i)=...                               % Companion computer get data from the controller
            Trans_Mesg(UAV_Coordinate(i),UAV_Control(i),0);
        UAV_Coordinate(i)=...                               % Companion computer starts computing
            UAV_Companion_Computer...
            (time,i,UAV_Coordinate,TAR,OBS);
        UAV_Control(i)=...                                  % Controller get data from the companion computer
            Trans_Mesg(UAV_Coordinate(i),UAV_Control(i),1);
        UAV_Control(i)=...                                  % Controller starts computing
            UAV_Flight_Controller(UAV_Control(i));
    end
    OBS=OBS_Movement(OBS,time,map);                         % Update obstacles position
    GS=Com_Station(time,UAV_Coordinate,GS);                 % Update ground station information
end

%% Plot results
Plot_UAV_Trajectory(map,GS,UAV_Coordinate,TAR,OBS);
Global_Proba=Plot_Data_Proba(time,map,property,UAV_Coordinate,TAR);
Global_Uncer=Plot_Data_Uncer(time,map,property,UAV_Coordinate,TAR);

