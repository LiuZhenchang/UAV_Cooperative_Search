%*****************************************************************
% Discription:  Programs running in UAV Companion computer
% input:        time                time structure
% input:        i                   UAV index
% input:        UAV                 UAVs structure array
% input:        TAR                 Targets structure array
% input:        OBS                 obstacles structure array
% output:       uav                 UAV structure
%*****************************************************************

function uav = UAV_Companion_Computer(time,i,UAV,TAR,OBS)
uav=UAV(i);
uav.search_count=uav.search_count+1;

%% Determine whether the UAV is surviving
if uav.search_num>uav.life_span
    return;
end

%% Conduct UAV search missions
if uav.search_count*time.delta>=uav.search_interval         % Determine whether the UAV has started searching
    uav.search_num=uav.search_num+1;                        % Update UAV search number
    if uav.expert_system
        uav=Expert_System(uav,TAR);                         % Update UAV search parameters using expert systems
    end
    if uav.task==1
        [uav,tar_find]=Search_Grid(uav,TAR);                % Perform a detection at the current position
        uav=Update_Search_Self(uav,time,tar_find);          % Update the search information of the UAV itself
    end
    tic;
    [obj_data,best_individual,uav]=...                      % Calculate action sequence
        RH_GA(uav,OBS);                                     % RH - Receding Horizon, GA - Genetic Algorithim

    uav.history_calc_time(time.step,2)=toc;                 % Save time spent on GA calculations
    uav.history_calc_time(time.step,3)=...                  % Save current gene length
        uav.param_GA.gene_length;     

    uav=Update_Search_CMD(uav,best_individual);             % Update UAV search command to determine the next point to go to
    uav.obj_data=obj_data;                                  % Save current objective function value
    uav.search_count=0;                                     % After executing a search, reset the counter to zero
    uav.perdict_pos_other=uav.perdict_pos_other*0-1e5;      % Clear the pre planned search position of other UAVs
    uav=UAV_Recorder(uav,best_individual);                  % Save the current search information of the UAV
    uav=Com_UAV(i,uav,UAV);                                 % Integrate information from current UAV and UAVs within communication range

    %% Generate warning and error messages
    xg_max=uav.map_x_max/uav.map_grid_size;
    yg_max=uav.map_y_max/uav.map_grid_size;

    % If the UAV approaches the boundary and generates warning information
    if uav.xg==1||uav.xg==xg_max||uav.yg==1||uav.yg==yg_max
        [position_series,yaw_series,~]=...
            Generate_Route(uav,best_individual);
        fprintf('Boundary AlERT\n');
        fprintf('UAV Index = %d\n',i);
        fprintf('UAV Position = %d , %d\n',uav.xg,uav.yg);
        fprintf('Positon & Yaw Series = \n');
        disp([position_series,yaw_series]);
    end
    
    % If the UAV exceeds the boundary and generates an alarm message
    if uav.xg<1||uav.xg>xg_max||uav.yg<1||uav.yg>yg_max
        fprintf('UAV Exceed Boundary\n');
    end
end
end

