%*******************************************************************************************
% Discription:  Calculate all grid information detected by the UAV on the predicted path
% input:        uav                 UAV structure
% input:        position_series     Predictive position sequence
% input:        yaw_series          Predictive yaw angle sequence
% output:       detect_table        Table for storing detected grid information
%*******************************************************************************************

function detect_table_output = Detect_Cover_Grid(uav,position_series,yaw_series)

map.x_min = uav.map_x_min;                                              % Minimum value in the x-direction of the map (meters)
map.x_max = uav.map_x_max;                                              % Maximum value in the x-direction of the map (meters)
map.y_min = uav.map_y_min;                                              % Minimum value in the y-direction of the map (meters)
map.y_max = uav.map_y_max;                                              % Maximum value in the y-direction of the map (meters)
map.grid_size = uav.map_grid_size;                                      % Square grid edge length (meters)

[step_num,~]=size(position_series);                                     % Obtain the predicted steps in grids
uav_perdict=uav;                                                        % Initialize the predicted state of the UAV
detect_num_max=2000;                                                    % Set the maximum number of grids for statistics
detect_table=zeros(detect_num_max,4)-1;

%% Statistics on the changes in search information after each detection of the UAV following the predicted path
for i=1:step_num
    uav_perdict.xg=position_series(i,1);                                % predictive state sequence in grid x-coordinate
    uav_perdict.yg=position_series(i,2);                                % predictive state sequence in grid y-coordinate
    uav_perdict.x=map.x_min+(uav_perdict.xg-0.5)*map.grid_size;         % predictive state sequence in x-coordinate      
    uav_perdict.y=map.y_min+(uav_perdict.yg-0.5)*map.grid_size;         % predictive state sequence in y-coordinate 
    uav_perdict.yaw=yaw_series(i);                                      % predictive yaw sequence 
    [cover_grid,cover_num]=Seeker(uav_perdict);                         % Grids covered by the field of view
    if cover_num<=0
        break;
    end
    for n=1:cover_num
        % Check if the currently covered grid overlaps with the previously covered grid
        % and add the non overlapping grid to the detect_table
        for j=1:detect_num_max
            
            % If it overlaps, terminate the current detection 
            % and start detecting the next grid
            if detect_table(j,1)==cover_grid(n,1)&&...
                    detect_table(j,2)==cover_grid(n,2)
                break;
            end

            % If no overlap is found after detection, 
            % record the grid information in the table
            if detect_table(j,1)==-1                                    % The value of the unrecorded table is -1
                detect_table(j,1)=cover_grid(n,1);                      % Record the grid x-coordinate
                detect_table(j,2)=cover_grid(n,2);                      % Record the grid y-coordinate
                if uav.revisit==0&&...                                  % If the target is found, the existence probability is set as 0
                        uav.proba(cover_grid(n,1),cover_grid(n,2))>0.8
                    detect_table(j,3)=0;
                else
                    detect_table(j,3)=...                               % Record the target existence probability of the grid
                    uav.proba(cover_grid(n,1),cover_grid(n,2));
                end
                detect_table(j,4)=...                                   % Record the environment uncertainty of the grid
                    uav.uncer(cover_grid(n,1),cover_grid(n,2));
                break;
            end
        end
    end
end
for detect_num=1:detect_num_max                                         % Count the number of covered grids
    if detect_table(detect_num,1)==-1||...
            detect_table(detect_num,2)==-1
        break;
    end
end
detect_num=detect_num-1;
if detect_num==0
    detect_table_output=zeros(1,4);
else
    detect_table_output=zeros(detect_num,4);                            % Initialize output matrix
    for i=1:detect_num
        detect_table_output(i,:)=detect_table(i,:);                     % Assign values to the output matrix
    end
end

end

