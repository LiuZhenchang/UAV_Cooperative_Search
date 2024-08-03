%*******************************************************************************************
% Discription:  Calculate the grid information covered by the UAV's field of view, 
% which is a square area centered on the drone with a known outward radius.
% input:        uav                 UAV structure
% output:       cover_grid          Grids covered by FOV
% output:       cover_num           Grids number covered by FOV
%*******************************************************************************************

function [cover_grid,cover_num] = Seeker_Center(uav)
search_grid=5;                                      % Grid detection radius outside the grid where the UAV is located
grid_size_standard=4;                               % Reference grid size
search_grid=floor(search_grid*...
    grid_size_standard/uav.map_grid_size);
map_grid=size(uav.proba);                           % Obtain the number of grids in the x and y directions
uav_grid=[uav.xg,uav.yg];                           % Obtain the UAV position in grid coordinate

% Calculate the minimum x-position of the FOV in 
% grid coordinate and determine if it exceeds the map boundary
if uav_grid(1)-search_grid<1
    cover_x_min=1;
else
    cover_x_min=uav_grid(1)-search_grid;
end

% Calculate the maximum x-position of the FOV in 
% grid coordinate and determine if it exceeds the map boundary
if uav_grid(1)+search_grid>map_grid(1)
    cover_x_max=map_grid(1);
else
    cover_x_max=uav_grid(1)+search_grid;
end

% Calculate the minimum y-position of the FOV in 
% grid coordinate and determine if it exceeds the map boundary
if uav_grid(2)-search_grid<1
    cover_y_min=1;
else
    cover_y_min=uav_grid(2)-search_grid;
end

% Calculate the maximum y-position of the FOV in 
% grid coordinate and determine if it exceeds the map boundary
if uav_grid(2)+search_grid>map_grid(2)
    cover_y_max=map_grid(2);
else
    cover_y_max=uav_grid(2)+search_grid;
end

index=1;
cover_num=(cover_x_max-cover_x_min+1)*...           % Calculate the number of covered grids
    (cover_y_max-cover_y_min+1);
if cover_num<=0||...
        ((cover_x_max-cover_x_min+1)<0&&(cover_y_max-cover_y_min+1)<0)                                     
    cover_num=0;
    cover_grid=0;
else
    cover_grid=zeros(cover_num,2);
    for i=cover_x_min:cover_x_max
        for j=cover_y_min:cover_y_max
            cover_grid(index,1)=i;                  % Save the x-coordinate of the covered grid
            cover_grid(index,2)=j;                  % Save the y-coordinate of the covered grid
            index=index+1;                          % Update the index of the covered grid
        end
    end
end

end

