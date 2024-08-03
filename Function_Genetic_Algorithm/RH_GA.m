%**********************************************************************************
% Discription:  Calculate action sequence of UAVs based on genetic algorithm
% input:        uav                     UAV structure
% input:        OBS                     Obstacles structure array
% output:       obj_data                Matrix to record objective function value
% output:       best_individual         Optimal action sequence
% output:       uav                     UAV structure
%***********************************************************************************

function [obj_data,best_individual,uav] = RH_GA(uav,OBS)
mapgrid=size(uav.proba);                                    % Obtain grid map rows and columns number
param=uav.param_GA;                                         % Obtain genetic algorithm parameters
pop_num=param.population_num;                               % Obtain population number
obj_data=zeros(4,4);                                        % Matrix to record objective function value
w_JA1=uav.w_JA1*uav.k_wJA1;                                 % Correction coefficient of JA1
w_JA2=uav.w_JA2*uav.k_wJA2;                                 % Correction coefficient of JA2
w_JB=uav.w_JB;                                              % Correction coefficient of JB
w_JC=uav.w_JC*uav.k_wJC;                                    % Corriecton coefficient of JC
JA1_max=-1000;
JA1_min=1000;
JA2_max=-1000;
JA2_min=1000;
JB_max=-1000;
JB_min=1000;
JC_max=-1000;
JC_min=1000;
best_fit=-1000;
prior_action=[0,-1,1];

%% Using genetic algorithm to solve action sequence
while best_fit<0
    switch uav.task
        %% Calculate the fixed-wing UAV action sequence without considering search revenue
        case 0
            iteration=100;
            if uav.search_jump<6
                gene_length=uav.search_jump*4;
            elseif uav.search_jump<=8&&uav.search_jump>=6
                gene_length=uav.search_jump*2;
            else
                gene_length=uav.search_jump*1;
            end
            pop_num=gene_length*6;
            pop=zeros(pop_num,gene_length);

            % Initialize the population and reduce computation time by
            % designing a reasonable initial population structure
            for i=1:6
                for j=1:gene_length
                    row=gene_length*(i-1)+j;
                    switch i
                        case 1
                            pop(row,1:(gene_length-j+1))=prior_action(1);
                            if j>1
                                pop(row,(gene_length-j+2):gene_length)=prior_action(2);
                            end
                        case 2
                            pop(row,1:(gene_length-j+1))=prior_action(1);
                            if j>1
                                pop(row,(gene_length-j+2):gene_length)=prior_action(3);
                            end
                        case 3
                            pop(row,1:(gene_length-j+1))=prior_action(2);
                            if j>1
                                pop(row,(gene_length-j+2):gene_length)=prior_action(1);
                            end
                        case 4
                            pop(row,1:(gene_length-j+1))=prior_action(2);
                            if j>1
                                pop(row,(gene_length-j+2):gene_length)=prior_action(3);
                            end
                        case 5
                            pop(row,1:(gene_length-j+1))=prior_action(3);
                            if j>1
                                pop(row,(gene_length-j+2):gene_length)=prior_action(1);
                            end
                        case 6
                            pop(row,1:(gene_length-j+1))=prior_action(3);
                            if j>1
                                pop(row,(gene_length-j+2):gene_length)=prior_action(2);
                            end
                    end
                end
            end

            for i=1:iteration
                obj_value=zeros(pop_num,1);                                     % Initialize the objective function value of every individual
                for j=1:pop_num
                    individual=pop(j,:);                                        % Obtain individual gene
                    [position_series,~,flag]=...                                % Generate state sequence based on individual gene
                        Generate_Route(uav,individual);

                    % Check state security
                    for m=1:gene_length
                        xg=position_series(m,1);
                        yg=position_series(m,2);

                        % Calculate the cost of map boundary distance
                        % Determine if it exceeds the map boundary
                        if xg<1||xg>mapgrid(1)||yg<1||yg>mapgrid(2)
                            flag=1;                                             % If the state exceeds the map range, set the flag to 1
                            obj_value(j,1)=obj_value(j,1)-1000;
                            break;
                        end

                        % Determine whether it exceeds the safety boundary
                        if (xg>=1&&xg<uav.safe_xg)||...
                                (xg>(mapgrid(1)-uav.safe_xg)&&xg<=mapgrid(1))||...
                                (yg>=1&&yg<uav.safe_yg)||...
                                (yg>(mapgrid(2)-uav.safe_yg)&&yg<=mapgrid(2))
                            flag=1;
                            obj_value(j,1)=obj_value(j,1)-1;
                        end

                        xr=uav.map_x_min+xg*uav.map_grid_size-0.5*uav.map_grid_size;
                        yr=uav.map_y_min+yg*uav.map_grid_size-0.5*uav.map_grid_size;
                        [~,obs_num]=size(OBS);

                        % Calculate the cost of obstacle distance
                        for k=1:obs_num
                            d=sqrt((xr-OBS(k).x)^2+(yr-OBS(k).y)^2);            % Calculate the distance from the grid to the obstacle
                            if d<OBS(k).r
                                flag=1;
                                obj_value(j,1)=obj_value(j,1)-100;
                                break;
                            end
                            if d>=OBS(k).r&&d<(OBS(k).r+uav.safe_obs)
                                if uav.search_jump>4
                                    flag=1;
                                    obj_value(j,1)=obj_value(j,1)-1;
                                end
                            end
                        end
                    end
                    % There is a state sequence that does not leave the boundary and does not touch obstacles
                    if flag==0
                        best_individual=individual;
                        return;
                    end
                end
                [best_fit,index]=max(obj_value);                                % Select the best individual genes
                if best_fit>-100
                    best_individual=pop(index,:);                               % Update the best individual genes
                    return;
                elseif uav.search_jump<8
                    pop=Selection_Champion(pop,obj_value);
                    pop=Mutation(pop,param.mp);
                    pop=Crossover(pop,param.cp);
                end
                if i==iteration&&uav.search_jump<=5&&best_fit>-1000
                    best_individual=pop(index,:);
                    return;
                end
            end
            if best_fit<=-100
                uav.search_jump=uav.search_jump-2;
            end
            %% Calculate the rotor UAV action sequence
        case 1
            pop=randi([-1,1],pop_num,param.gene_length);
            param=uav.param_GA;
            gene_length=param.gene_length;

            % Initialize the population and reduce computation time by
            % designing a reasonable initial population structure
            for i=1:6
                for j=1:gene_length
                    row=gene_length*(i-1)+j;
                    switch i
                        case 1
                            pop(row,1:(gene_length-j+1))=prior_action(1);
                            if j>1
                                pop(row,(gene_length-j+2):gene_length)=prior_action(2);
                            end
                        case 2
                            pop(row,1:(gene_length-j+1))=prior_action(1);
                            if j>1
                                pop(row,(gene_length-j+2):gene_length)=prior_action(3);
                            end
                        case 3
                            pop(row,1:(gene_length-j+1))=prior_action(2);
                            if j>1
                                pop(row,(gene_length-j+2):gene_length)=prior_action(1);
                            end
                        case 4
                            pop(row,1:(gene_length-j+1))=prior_action(2);
                            if j>1
                                pop(row,(gene_length-j+2):gene_length)=prior_action(3);
                            end
                        case 5
                            pop(row,1:(gene_length-j+1))=prior_action(3);
                            if j>1
                                pop(row,(gene_length-j+2):gene_length)=prior_action(1);
                            end
                        case 6
                            pop(row,1:(gene_length-j+1))=prior_action(3);
                            if j>1
                                pop(row,(gene_length-j+2):gene_length)=prior_action(2);
                            end
                    end
                end
            end

            for i=1:param.iteration
                obj_value=zeros(pop_num,1);                                     % Initialize the objective function value of every individual
                for j=1:pop_num
                    individual=pop(j,:);                                        % Obtain individual gene
                    [position_series,yaw_series,flag]=...                       % Generate state sequence based on individual gene
                        Generate_Route(uav,individual);
                    % Check state security
                    for m=1:param.gene_length
                        xg=position_series(m,1);
                        yg=position_series(m,2);
                        % Calculate the cost of map boundary distance
                        % Determine whether it exceeds the safety boundary
                        if xg<1||xg>mapgrid(1)||yg<1||yg>mapgrid(2)
                            flag=1;
                            break;
                        end
                        % Determine whether it exceeds the safety boundary
                        if (xg>=1&&xg<uav.safe_xg)||...
                                (xg>(mapgrid(1)-uav.safe_xg)&&xg<=mapgrid(1))||...
                                (yg>=1&&yg<uav.safe_yg)||...
                                (yg>(mapgrid(2)-uav.safe_yg)&&yg<=mapgrid(2))
                            obj_value(j,1)=obj_value(j,1)-5;
                        end
                        xr=uav.map_x_min+xg*uav.map_grid_size-0.5*uav.map_grid_size;
                        yr=uav.map_y_min+yg*uav.map_grid_size-0.5*uav.map_grid_size;

                        % Calculate the cost of obstacle distance
                        [~,obs_num]=size(OBS);
                        for k=1:obs_num
                            d=sqrt((xr-OBS(k).x)^2+(yr-OBS(k).y)^2);            % Calculate the distance from the grid to the obstacle
                            if d<OBS(k).r
                                flag=1;
                                break;
                            end
                            if d>=OBS(k).r&&d<(OBS(k).r+uav.safe_obs)
                                obj_value(j,1)=obj_value(j,1)-5;
                            end
                        end

                        % Calculate the cost of distance between UAVs
                        if uav.foresee_traj
                            [total_foresee,~]=size(uav.perdict_pos_other);
                            for k=1:total_foresee
                                d=sqrt((xr-uav.perdict_pos_other(k,1))^2 ...
                                    +(yr-uav.perdict_pos_other(k,2))^2);
                                if d<uav.safe_uav
                                    obj_value(j,1)=obj_value(j,1)-1;
                                end
                            end
                        end
                    end
                    % Calculate state sequence revenue
                    if flag==1
                        obj_value(j,1)=-1000;
                    else
                        detect_table=Detect_Cover_Grid...                       % Obtain grids covered by FOV
                            (uav,position_series,yaw_series);
                        [JA1,JA2]=J_A(detect_table);                            % Calculate probability and uncertainty
                        JB=J_B(individual);                                     % Calculate maneuver cost
                        JC=J_C(uav,position_series);                            % Calculate potential function Value

                        obj_value(j,1)=obj_value(j,1)+...                       % Calculate objective function value
                            w_JA1*JA1+w_JA2*JA2+w_JB*JB+w_JC*JC;

                        [JA1_min,JA1_max]=Min_Max(JA1,JA1_min,JA1_max);
                        [JA2_min,JA2_max]=Min_Max(JA2,JA2_min,JA2_max);
                        [JB_min,JB_max]=Min_Max(JB,JB_min,JB_max);
                        [JC_min,JC_max]=Min_Max(JC,JC_min,JC_max);
                    end
                    %Plot_UAV_Detect(map,uav,detect_table)
                end

                %% Processing of population calculation results
                [best_fit_new,index]=max(obj_value);                            % Select the best individual genes
                if best_fit_new>best_fit
                    best_fit=best_fit_new;
                    best_individual=pop(index,:);                               % Update the best individual genes
                end

                %% Output the optimal result or continue iterating
                % If there is not much difference in the optimal results
                % between two iterations of the genetic algorithm
                if (i==param.iteration||...
                        abs(best_fit_new-best_fit)<0.2)&&best_fit>=-900
                    [position_series,yaw_series,~]=...                          % Generate state sequence based on individual gene
                        Generate_Route(uav,best_individual);
                    detect_table=Detect_Cover_Grid...                           % Obtain grids covered by FOV
                        (uav,position_series,yaw_series);
                    [JA1,JA2]=J_A(detect_table);                                % Calculate probability and uncertainty
                    JB=J_B(best_individual);                                    % Calculate maneuver cost
                    JC=J_C(uav,position_series);                                % Calculate objective function value

                    % Calculate decision trend
                    if sum(best_individual)>0
                        uav.decision_trend=1;
                    elseif sum(best_individual)<0
                        uav.decision_trend=-1;
                    else
                        uav.decision_trend=0;
                    end
                    % If the state exceeds the boundary without calculating the results
                    if JA1_min==1000||JA2_min==1000||JC_min==1000
                        JA1_max=JA1;JA1_min=JA1;
                        JA2_max=JA2;JA2_min=JA2;
                        JC_max=JC;JC_min=JC;
                    end

                    obj_data=[JA1,  JA1_min,    JA1_max,    w_JA1;...
                        JA2,  JA2_min,    JA2_max,    w_JA2;...
                        JB,   JB_min,     JB_max,     w_JB;...
                        JC,   JC_min,     JC_max,     w_JC];
                    break;
                elseif uav.search_jump<=8
                    pop=Selection_Champion(pop,obj_value);
                    pop=Mutation(pop,param.mp);
                    pop=Crossover(pop,param.cp);
                end

            end
            if best_fit>-1000&&best_fit<0&&uav.search_jump<=2
                return;
            end
            if best_fit<=0
                uav.search_jump=uav.search_jump-2;
            end

    end

    if uav.search_jump<1
        error('GA Solve failed');
    end
end
end

