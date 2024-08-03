%*****************************************************************
% Discription:  Record search status inside the UAV
% input:        uav                 UAV structure
% input:        individual          Current action sequence 
% output:       uav                 UAV structure
%*****************************************************************

function uav = UAV_Recorder(uav,individual)
search_num=uav.search_num;                                              % Obtian current search number
uav.history_gene_length(search_num,1)=uav.param_GA.gene_length;         % Save gene length
uav.history_jump(search_num,1)=uav.search_jump;                         % Save jump grid number
uav.history_grid(search_num,1)=uav.xg;                                  % Save UAV grid x-coordinate
uav.history_grid(search_num,2)=uav.yg;                                  % Save UAV grid y-coordinate
uav.history_position(search_num,1)=uav.x;                               % Save UAV x-coordinate
uav.history_position(search_num,2)=uav.y;                               % Save UAV y-coordinate
uav.history_yaw(search_num,1)=uav.yaw;                                  % Save UAV yaw angle

%% Save the optimal gene sequence results
[~,gene_length]=size(individual);
for i=1:gene_length                                        
    uav.history_gene_best(search_num,i)=individual(1,i);
end

%% Save discovered target discovery number
tar_find=0;
[tar_num,~]=size(uav.target_info);                                      % Obtian targets number
for i=1:tar_num
    if uav.target_info(i,1)~=0
        tar_find=tar_find+1;
    end
end
uav.history_target(search_num,1)=tar_find;
end

