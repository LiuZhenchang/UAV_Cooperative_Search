%*******************************************************************************************
% Discription:  Calculate the grid information covered by the UAV's field of view, 
% which is a rectangular area and pointing along the UAV nose with a forward displacement.
% input:        uav                 UAV structure
% output:       cover_grid          Grids covered by FOV
% output:       cover_num           Grids number covered by FOV
%*******************************************************************************************

function [cover_grid,cover_num] = Seeker_Forward(uav)
%% Set Initial parameters
distance_forward=40;                                                % Prepositive distance of FOV
cover_length=60;                                                    % Length of FOV
cover_width=40;                                                     % Width of FOV
exceed_flag=0;

%% Calculate FOV center position
x_c=uav.x+distance_forward*cos(uav.yaw);                            % FOV center x-coordinate 
y_c=uav.y+distance_forward*sin(uav.yaw);                            % FOV center y-coordinate 
map.x_min = uav.map_x_min;                                          % Minimum value in the x-direction of the map (meters)
map.x_max = uav.map_x_max;                                          % Maximum value in the x-direction of the map (meters)
map.y_min = uav.map_y_min;                                          % Minimum value in the y-direction of the map (meters)
map.y_max = uav.map_y_max;                                          % Maximum value in the y-direction of the map (meters)
map.grid_size = uav.map_grid_size;                                  % Square grid edge length (meters)

%% Reduce the filtering range of the FOV covered grid and determine if the FOV exceeds the boundary
%Generate the bounding circle and bounding square of the FOV, 
% and analyze whether the grid inside the square is covered by the FOV
radius=sqrt(cover_length^2+cover_width^2)/2;
grid_x_min=ceil((x_c-radius-map.x_min)/map.grid_size);
grid_x_max=ceil((x_c+radius-map.x_min)/map.grid_size);
grid_y_min=ceil((y_c-radius-map.y_min)/map.grid_size);
grid_y_max=ceil((y_c+radius-map.y_min)/map.grid_size);

% The situation where the square exceeds 
% the left boundary of the task area
if x_c-radius<map.x_min
    if x_c+radius<map.x_min                                         % All exceed the left boundary
        exceed_flag=1;                                              % Set the exceed_flag to 1
    else                                                            % Partially exceed the left boundary
        grid_x_min=1;                                               % Cut the detection area using the left boundary
    end
end

% The situation where the square exceeds 
% the right boundary of the task area
if x_c+radius>map.x_max
    if x_c-radius>map.x_max                                         % All exceed the right boundary
        exceed_flag=1;                                              % Set the exceed_flag to 1
    else                                                            % Partially exceed the right boundary
        grid_x_max=ceil((map.x_max-map.x_min)/map.grid_size);       % Cut the detection area using the right boundary
    end
end

% The situation where the square exceeds 
% the upper boundary of the task area
if y_c-radius<map.y_min
    if y_c+radius<map.y_min                                         % All exceed the upper boundary
        exceed_flag=1;                                              % Set the exceed_flag to 1
    else                                                            % Partially exceed the upper boundary
        grid_y_min=1;                                               % Cut the detection area using the upper boundary             
    end
end

% The situation where the square exceeds 
% the bottom boundary of the task area
if y_c+radius>map.y_max
    if y_c-radius>map.y_max                                         % All exceed the bottom boundary
        exceed_flag=1;                                              % Set the exceed_flag to 1
    else                                                            % Partially exceed the bottom boundary
        grid_y_max=ceil((map.y_max-map.y_min)/map.grid_size);       % Cut the detection area using the bottom boundary
    end
end

%% Determine whether the grid within the square area is covered by FOV
if exceed_flag                                                      
    cover_grid=zeros(1,2);
    cover_num=0;
else
    test_num=(grid_x_max-grid_x_min)*(grid_y_max-grid_y_min);       % Count the number of grids in the square area within the task area
    cover_grid_temp=zeros(test_num,2);
    cover_num=0;
    for i=grid_x_min:grid_x_max
        for j=grid_y_min:grid_y_max
            x_t=map.x_min+i*map.grid_size-0.5*map.grid_size;        % Calculate the x-coordinate of the grid center
            y_t=map.y_min+j*map.grid_size-0.5*map.grid_size;        % Calculate the y-coordinate of the grid center
            v1=[x_c-x_t,y_c-y_t];                                   % Calculate vector from grid center to FOV center
            v2=[x_c-uav.x,y_c-uav.y];                               % Calculate vector from UAV to FOV center
            l=abs((v1(1)*v2(1)+v1(2)*v2(2))/norm(v2));              % Calculate the projection length from v1 to v2 using the cosine theorem
            w=abs((v1(1)*v2(2)-v1(2)*v2(1))/norm(v2));              % Calculate the vertical distance from the grid to v2 using the sine theorem
            if l<0.5*cover_length&&w<0.5*cover_width                % Determine whether the grid is within the FOV
                cover_num=cover_num+1;                              % If so, number of covered grids+1
                cover_grid_temp(cover_num,1)=i;                     % Save the x-coordinate of the covered grid
                cover_grid_temp(cover_num,2)=j;                     % Save the y-coordinate of the covered grid
            end
        end
    end
    cover_grid=zeros(cover_num,2);
    for i=1:cover_num
        cover_grid(i,:)=cover_grid_temp(i,:);
    end
end

end

