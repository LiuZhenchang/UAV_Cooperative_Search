%*******************************************************************************************
% Discription:  Using the artificial potential field method to calculate the value of potential function
%               between UAVs as a partial result of the objective function
% input:        uav                     UAV structure
% input:        position_series         Predictive position sequence
% output:       APF_value               Value of artificial potential field
%*******************************************************************************************
function APF_value = J_C(uav,position_series)
[pt_num,~]=size(position_series);                                   % Obtain predictive sequence length 
[uav_num,~]=size(uav.teammates);                                    % Obtain UAVs number
d_max=uav.APF_distance;                                             % effective range of artificial potential field
miu=uav.APF_param1;                                                 % Artificial potential field parameters miu
k=uav.APF_param2;                                                   % Artificial potential field parameters k
APF_value=uav_num-1;
for i=1:pt_num
    x_p=uav.map_x_min+...
        position_series(i,1)*uav.map_grid_size-...
        0.5*uav.map_grid_size;
    y_p=uav.map_y_min+...
        position_series(i,2)*uav.map_grid_size-...
        0.5*uav.map_grid_size;
    field_value=zeros(1,2);
    mate_num=0;                                                     % Initialize the number of wingmen
    for j=1:uav_num
        % If the index is not 0, the teammate information is valid
        if uav.teammates(j,1)~=0                
            mate_num=mate_num+1;                                    % Count the number of wingmen
            x_tm=uav.teammates(j,2);                                % Obtain the x-coordinate of the wingman's position
            y_tm=uav.teammates(j,3);                                % Obtain the y-coordinate of the wingman's position
            d=sqrt((x_p-x_tm)^2+(y_p-y_tm)^2);                      % Calculate the distance between the predicted position and the wingman
            vector=[x_p-x_tm,y_p-y_tm];                             % Calculate the direction of the potential field
            if d<d_max
                field_value=...
                    field_value+k*exp(-miu*d)*vector/d;             % Accumulate APF value
            end
        end
    end
    % Avoiding the impact of different prediction length on Jc results
    APF_value=APF_value-norm(field_value)/pt_num;
end
end


