%*******************************************************************************************
% Discription:  Plot environment uncertainty
% input:        time                time structure
% input:        map                 Map structure
% input:        property            property structure
% input:        UAV_Coordinate      UAVs structure array
% input:        TAR                 Targets structure array
% output:       uncer_general       Global environment uncertainty array
%*******************************************************************************************
function uncer_general = Plot_Data_Uncer(time,map,property,UAV_Coordinate,TAR)
UAV_temp=Creat_UAV_Coordinate(property,map);                                % Initialize temporary UAV structure array
GS = Creat_Ground_Station(time,property,map,TAR);                           % Initialize the ground station
UAV_temp=Init_UAV_Map(UAV_temp,GS);                                         % Initialize the search information for each UAV from the ground station
uncer_uav=zeros(property.uav.num,time.step_max);                            % Initialize the environment uncertainty array of each UAV
uncer_general=zeros(1,time.step_max);                                       % Initialize the global environment uncertainty array
grid_size_standard=4;                                                       % Set standard grid size
k_grid=(map.grid_size/grid_size_standard)^2;                                % Calculate grid size ratio
print_count=0;

%% Reproduce the cooperative search process of UAVs based on search history
for search_num=1:time.step_max                                              % Traverse and reproduce each step of the search history
    time.step=search_num;                                                   % Update the number of time steps for subsequent calculations
    print_count=print_count+1;                                              % Update print counter for progress display
    if print_count==60
        fprintf('Step number = %d, finised %d percent\n',...                % Display reproduce progress
            search_num,search_num/time.step_max*100);
        print_count=0;                                                      % Counter reset to zero
    end
    % Traverse each UAV and store the calculated results of the UAV 
    % in a temporary UAV structure for current search results replication
    for i=1:property.uav.num
        UAV_temp(i).search_num=search_num;
        UAV_temp(i).search_jump=...
            UAV_Coordinate(i).history_jump(search_num,1);
        UAV_temp(i).xg=UAV_Coordinate(i).history_grid(search_num,1);
        UAV_temp(i).yg=UAV_Coordinate(i).history_grid(search_num,2);
        UAV_temp(i).x=UAV_Coordinate(i).history_position(search_num,1);
        UAV_temp(i).y=UAV_Coordinate(i).history_position(search_num,2);
        UAV_temp(i).yaw=UAV_Coordinate(i).history_yaw(search_num,1);
        if search_num>UAV_temp(i).life_span
            continue;
        end
        if UAV_temp(i).task==1                                              % If the UAV is performing a search task
            [UAV_temp(i),tar_find]=Search_Grid(UAV_temp(i),TAR);            % Perform a detection at the current location
            UAV_temp(i)=Update_Search_Self(UAV_temp(i),time,tar_find);      % Update the UAV's self search information storage
        end
        UAV_temp(i)=Com_UAV(i,UAV_temp(i),UAV_temp);                        % Integrate information from current UAV and UAVs within communication range
        uncer_uav(i,search_num)=sum(UAV_temp(i).uncer(:));                  % Calculate the environment uncertainty locally
    end
    GS=Com_Station(time,UAV_temp,GS);                                       % Update ground station information
    uncer_general(search_num)=sum(GS.uncertainty(:));                       % Calculate the environment uncertainty globally                      
end

%% Plot environment uncertainty results
figure('name','UAV uncertainty data');
hold on;
lto=plot(uncer_general*k_grid,'r');
l1=plot(uncer_uav(1,:)*k_grid,'k');
l2=plot(uncer_uav(2,:)*k_grid,'--k');
l3=plot(uncer_uav(3,:)*k_grid,'b');
l4=plot(uncer_uav(4,:)*k_grid,'--b');
set(gcf,'unit','inches','position',[0,0,4,3]);
set(gca,'FontName','Times New Roman','FontSize',10);ylim([10000,20000]);
xlabel('Search time (s)');
ylabel('Uncertainty');
xlim([0,time.step_max]);
ylim([40000,80000]);
lgd=legend([lto,l1,l2,l3,l4],...
    {'Entirety','UAV1','UAV2','UAV3','UAV4'});
lgd.Location='southwest';
lgd.NumColumns=1;
lgd.FontSize=10;
grid on;
box on;
end

