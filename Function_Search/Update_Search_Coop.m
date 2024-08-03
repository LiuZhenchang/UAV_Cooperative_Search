%************************************************************************************************
% Discription:  Update self search information matrix based on search information from other UAVs
% input:        uav1                Self UAV structure
% input:        uav2                Other UAV structure
% output:       uav1                Self UAV structure
% output:       update_info         Data update time of UAVs
%************************************************************************************************

function [uav1,update_info] = Update_Search_Coop(uav1,uav2)
[row_num1,clm_num1]=size(uav1.search_coop);                         % Obtain the size of the search_comp matrix for the UAV1
[row_num2,clm_num2]=size(uav2.search_coop);                         % Obtain the size of the search_comp matrix for the UAV2
id2=uav2.index;                                                     % Obtain the index of other UAV
uav_num=clm_num1/4;                                                 % Calculate UAV number
update_info=zeros(uav_num,2);

% If the search_coop matrix sizes of UAV 1 and 
% UAV 2 are different, output an error message
if (row_num1~=row_num2)&&(clm_num1~=clm_num2)
    error("Message length not match");
end

%% Update search information
switch uav2.com_type
    case 0                                                          % Only update the UAV2 information from UAV 2
        update_info(id2,1)=...
            max(uav1.search_coop(:,4*(id2-1)+1));
        update_info(id2,2)=1;

        % Traverse each row of search_comop in drone 1, 
        % find the corresponding column and update it 
        % with the information of drone 2
        for i=1:row_num1
            uav1.search_coop(i,4*(id2-1)+1)=...
                uav2.search_coop(i,4*(id2-1)+1);                    % 1 Update seach time
            uav1.search_coop(i,4*(id2-1)+2)=...
                uav2.search_coop(i,4*(id2-1)+2);                    % 2 Update UAV x-position in grid coordinate
            uav1.search_coop(i,4*(id2-1)+3)=...
                uav2.search_coop(i,4*(id2-1)+3);                    % 3 Update UAV y-position in grid coordinate
            uav1.search_coop(i,4*(id2-1)+4)=...
                uav2.search_coop(i,4*(id2-1)+4);                    % 4 Update UAV yaw angle
        end
    case 1
        for j=1:uav_num                                             % Update all information from UAV 2
            if j== uav1.index
                continue;
            end
            time1=max(uav1.search_coop(:,4*(j-1)+1));
            time2=max(uav2.search_coop(:,4*(j-1)+1));

            % If the update time of drone 2 is greater than 
            % the update time of drone 1, start updating
            if time2>time1
                update_info(j,1)=time1;
                update_info(j,2)=1;
                for i=1:row_num1
                    uav1.search_coop(i,4*(j-1)+1)=...
                        uav2.search_coop(i,4*(j-1)+1);
                    uav1.search_coop(i,4*(j-1)+2)=...
                        uav2.search_coop(i,4*(j-1)+2);
                    uav1.search_coop(i,4*(j-1)+3)=...
                        uav2.search_coop(i,4*(j-1)+3);
                    uav1.search_coop(i,4*(j-1)+4)=...
                        uav2.search_coop(i,4*(j-1)+4);
                end
            end
        end
end

%% Update target information
[tar_num,~]=size(uav1.target_info);
for i=1:tar_num
    % If the target info update time of drone 2 is greater than 
    % the target info update time of drone 1, start updating
    if uav2.target_info(i,1)>uav1.target_info(i,1)
        uav1.target_info(i,1)=uav2.target_info(i,1);
        uav1.target_info(i,2)=uav2.target_info(i,2);
        uav1.target_info(i,3)=uav2.target_info(i,3);
        uav1.target_info(i,4)=uav2.target_info(i,4);
    end
end

%% Update pre planned search location for other UAVs
if uav2.task==1
    for i=1:uav2.foresee_num
        uav1.perdict_pos_other((uav2.index-1)*uav1.foresee_num+i,:)...
            =uav2.perdict_pos_self(i,:);
    end
end
end

