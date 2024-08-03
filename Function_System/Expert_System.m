%*****************************************************************
% Discription:  Implement expert system functionality to update weight coefficients based on search status
% input:        uav                 UAV structure
% input:        TAR                 Targets structure array
% output:       uav                 UAV structure
%*****************************************************************
function [uav] = Expert_System(uav,TAR)
% Determine whether to use an expert system based on the type of UAV task
switch uav.task
    case 0
        uav.search_jump=12;                                 % Set a large jump grid number for fixed-wing UAV
    case 1
        %% Calculate the input of expert system 1 
        [~,tar_num]=size(TAR);                              % Obtain targets number
        dmin=1e8;                                           % Initialize the minimum distance between the UAV and targets
        for i=1:tar_num
            d=sqrt((uav.x-TAR(i).x)^2+(uav.y-TAR(i).y)^2);  % Calculate the distance between the UAV and the target
            if d<dmin
                dmin=d;                                     % Update the minimum distance
            end
        end
        E1=dmin/uav.d_tar;                                  % Calculate the input of expert system 1 
        uav.E1=E1;                                          % Save the input of expert system 1 in UAV

        %% Calculate the input of expert system 2 
        tar_find=0;
        [row,clm]=size(uav.proba);
        for i=1:row
            for j=1:clm
                if uav.proba(i,j)>0.8
                    tar_find=tar_find+1;                    % Count the number of targets discovered by UAVs
                end
            end
        end
        E2=tar_find/tar_num;                                % Calculate the input of expert system 2 
        uav.E2=E2;                                          % % Save the input of expert system 2 in UAV

        %% get output of expert system 1 
        if E1<1                                             % When approaching the target
            uav.search_jump=2;                              % Decrease the jump grid number 
            uav.k_wJC=2;                                    % Increase the collision weight coefficient
        elseif E1>=1&&E1<2
            uav.search_jump=4;
            uav.k_wJC=1;
        else
            uav.search_jump=6;
            uav.k_wJC=0.8;
        end

        %% get output of expert system 2 
        if E2<0.75
            uav.k_wJA1=1;
            uav.k_wJA2=1;
            uav.param_GA.gene_length=8;
        elseif E2>=0.75&&E2<1
            uav.k_wJA1=0.8;
            uav.k_wJA2=1.2;
            uav.param_GA.gene_length=10;
        else
            uav.k_wJA1=0.4;
            uav.k_wJA2=1.6;
            uav.param_GA.gene_length=12;
            uav.search_jump=4;
        end

        %% Adjust for near boundary conditions
        kbx=0.05;kby=0.1;                                   % Boundary coefficient
        if uav.x<kbx*(uav.map_x_max-uav.map_x_min)...
                ||uav.x>uav.map_x_max-kbx*(uav.map_x_max-uav.map_x_min)...
                ||uav.y<kby*(uav.map_y_max-uav.map_y_min)...
                ||uav.y>uav.map_y_max-kby*(uav.map_y_max-uav.map_y_min)
            uav.param_GA.gene_length=10;
            uav.k_wJC=0;
        end
end

end

