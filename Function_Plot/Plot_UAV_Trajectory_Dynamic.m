%********************************************************************************************
% Discription:  Creat video for UAV cooperative search results
% input:        map                 Map structure
% input:        GS                  Ground station information
% input:        UAV                 UAVs structure array
% input:        TAR                 Targets structure array
% input:        OBS                 obstacles structure array
% output:       F                   Struct containing information for each frame of the image
%********************************************************************************************
% Discription:  Creat video for UAV cooperative search results
% F=Plot_UAV_Trajectory_Dynamic(map,GS,UAV_Coordinate,TAR,OBS)
% h=figure
% set(gcf,'unit','inches','position',[0,0,14,6]);
% movie(h,F,1,10);
% v=VideoWriter('Trajectory.mp4','MPEG-4');
% v.FrameRate=10;
% v.Quality=100;
% open(v);
% writeVideo(v,F);
% close(v);

function F = Plot_UAV_Trajectory_Dynamic(map,GS,UAV,TAR,OBS)
%% Initialize figure information, and follow this setting for each subsequent frame attribute
figure('name','Dynamic Figure','Color',[1 1 1]);                            % Set figure name and color
set(gcf,'unit','inches','position',[0,0,14,6]);                             % Set figure size
set(0,'defaultfigurecolor','w');                                            % Set default figure color as white
F(GS.search_num) = struct('cdata',[],'colormap',[]);                        % Initialize the structure to store the image of each frame
[~,uav_num]=size(UAV);                                                      % Obtain the number of UAVs
scale=1;                                                                    % Set the drawing scale
print_count=0;                                                              % Initialization progress display counter
for search_num=1:GS.search_num
    newplot;                                                                % Start drawing according to the content set by gca and gcf
    hold on;
    print_count=print_count+1;
    % Display plot progress
    if print_count==30
        fprintf('Step number = %d, finised %d percent\n',...
            search_num,search_num/GS.search_num*100);
        print_count=0;
    end
    %% Plot UAV positions
    for k=1:uav_num
        x_u=zeros(search_num,1);
        y_u=zeros(search_num,1);
        z_u=zeros(search_num,1);
        for n=1:search_num
            x_u(n,1)=GS.search_grid(k,1,n);                                 % Obtain the x-coordinates of all UAVs
            y_u(n,1)=GS.search_grid(k,2,n);                                 % Obtain the y-coordinates of all UAVs
            z_u(n,1)=30;
            if UAV(k).task==0
                x_u(n,1)=GS.UAV_path(k,1,n);
                y_u(n,1)=GS.UAV_path(k,2,n);
                z_u(n,1)=200;
            end
        end

        % Plot fixed-wing UAV's footprint
        if UAV(k).task==0
            dot_search5=scatter3(x_u*scale,y_u*scale,z_u,45,...
                'MarkerFaceColor','b','MarkerEdgeColor','b',...
                'MarkerFaceAlpha',0.5);
            continue;
        end
        % Plot rotor UAVs' footprint
        switch k
            case 1
                dot_search1=scatter3(x_u*scale,y_u*scale,z_u,45,...
                    'MarkerFaceColor','c','MarkerEdgeColor','k',...
                    'MarkerFaceAlpha',0.5);
            case 2
                dot_search2=scatter3(x_u*scale,y_u*scale,z_u,45,...
                    'MarkerFaceColor','y','MarkerEdgeColor','k',...
                    'MarkerFaceAlpha',0.5);
            case 3
                dot_search3=scatter3(x_u*scale,y_u*scale,z_u,45,...
                    'MarkerFaceColor','g','MarkerEdgeColor','k',...
                    'MarkerFaceAlpha',0.5);
            case 4
                dot_search4=scatter3(x_u*scale,y_u*scale,z_u,45,...
                    'MarkerFaceColor','M','MarkerEdgeColor','k',...
                    'MarkerFaceAlpha',0.5);
            case 5
                dot_search5=scatter3(x_u*scale,y_u*scale,z_u,45,...
                    'MarkerFaceColor','b','MarkerEdgeColor','b',...
                    'MarkerFaceAlpha',0.5);
        end
    end

    %% Plot searched area
    for k=1:uav_num
        if UAV(k).task==0
            continue;
        end
        x_c_temp=zeros(search_num*100,1);
        y_c_temp=zeros(search_num*100,1);
        count=0;
        for n=1:search_num
            uav_temp=UAV(k);                                                % Temporary UAV structure
            uav_temp.x=GS.search_grid(k,1,n);
            uav_temp.y=GS.search_grid(k,2,n);
            uav_temp.xg=ceil((uav_temp.x-map.x_min)/map.grid_size);         % Update grid x-coordinate
            uav_temp.yg=ceil((uav_temp.y-map.y_min)/map.grid_size);         % Update grid y-coordinate
            uav_temp.yaw=GS.search_yaw(k,n);
            [cover_grid,cover_num]=Seeker(uav_temp);

            % Concatenate the coordinates of the detected grids into an array and output it uniformly
            if cover_num>0
                for i=1:cover_num
                    x_c_temp(count+i,1)=map.x_min+...
                        cover_grid(i,1)*map.grid_size-0.5*map.grid_size;
                    y_c_temp(count+i,1)=map.y_min+...
                        cover_grid(i,2)*map.grid_size-0.5*map.grid_size;
                end
                count=count+cover_num;
            end
        end
        x_c=x_c_temp(1:count,1);                                            % Extract an array with stored information
        y_c=y_c_temp(1:count,1);

        % Plot each UAV searched area
        switch k
            case 1
                dot_cover1=scatter(x_c*scale,y_c*scale,80,'s',...
                    'MarkerFaceColor','c','MarkerFaceAlpha',0.1,...
                    'MarkerEdgeColor','c','MarkerEdgeAlpha',1);
            case 2
                dot_cover2=scatter(x_c*scale,y_c*scale,80,'s',...
                    'MarkerFaceColor','y','MarkerFaceAlpha',0.1,...
                    'MarkerEdgeColor','y','MarkerEdgeAlpha',1);
            case 3
                dot_cover3=scatter(x_c*scale,y_c*scale,80,'s',...
                    'MarkerFaceColor','g','MarkerFaceAlpha',0.1,...
                    'MarkerEdgeColor','g','MarkerEdgeAlpha',1);
            case 4
                dot_cover4=scatter(x_c*scale,y_c*scale,80,'s',...
                    'MarkerFaceColor','M','MarkerFaceAlpha',0.1,...
                    'MarkerEdgeColor','M','MarkerEdgeAlpha',1);
        end
    end

    %% Plot target positions
    [~,target_num]=size(TAR);                                               % Obtain the targets number
    x_t=zeros(1,target_num);
    y_t=zeros(1,target_num);
    z_t=zeros(1,target_num)+3;
    for k=1:target_num
        x_t(k)=TAR(k).x;                                                    % Obtain the x-coordinates of all targets
        y_t(k)=TAR(k).y;                                                    % Obtain the x-coordinates of all targets
    end
    dot_tar=scatter3(x_t*scale,y_t*scale,z_t,100,'d',...                    % Plot targets positon
        'MarkerFaceColor','r','MarkerEdgeColor','k');

    %% Plot obstacle positions
    [~,obs_num]=size(OBS);
    for i=1:obs_num
        [xo,yo,zo]=cylinder(OBS(i).r*scale);
        x_obs=OBS(i).history_position(search_num,1);
        y_obs=OBS(i).history_position(search_num,2);
        xo=xo+x_obs*scale;
        yo=yo+y_obs*scale;
        zo=zo*100;
        cyl_obs=surf(xo,yo,zo,'FaceAlpha',0.1);
        s=sprintf('%d',i);
        text(x_obs*scale,y_obs*scale,110,s);
    end

    %% Plot UAV trajectories
    for k=1:uav_num
        Traj_x=zeros(search_num,1);
        Traj_y=zeros(search_num,1);
        Traj_z=zeros(search_num,1);
        for i=1:search_num
            Traj_x(i)=GS.UAV_path(k,1,i);
            Traj_y(i)=GS.UAV_path(k,2,i);
            if UAV(k).task==0
                Traj_z(i)=200;
            else
                Traj_z(i)=30;
            end
        end
        if UAV(k).task==0
            line_traj=plot3(Traj_x*scale,Traj_y*scale,Traj_z,...
                'k-','LineWidth',2);
        else
            line_traj=plot3(Traj_x*scale,Traj_y*scale,Traj_z,...
                'k','LineWidth',2);
        end
    end

    %% Figure setting
    set(gca,'FontName','Times New Roman','FontSize',16);
    set(0,'defaultfigurecolor','w');
    xlabel('$X/m$','Interpreter','latex');
    ylabel('$Y/m$','Interpreter','latex');
    zlabel('$Z/m$','Interpreter','latex');
    xlim([map.x_min*scale,map.x_max*scale]);
    ylim([map.y_min*scale,map.y_max*scale]);
    view(-10,60);

    switch uav_num
        case 4
            lgd=legend([dot_search1,dot_search2,dot_search3,dot_search4,...
                dot_cover1,dot_cover2,dot_cover3,dot_cover4,...
                line_traj,dot_tar,cyl_obs],...
                {'UAV1 search point','UAV2 search point',...
                'UAV3 search point','UAV4 search point',...
                'UAV1 search area','UAV2 search area',...
                'UAV3 search area','UAV4 search area',...
                'UAV trajectory','Target','Obstacle'});
        case 5
            lgd=legend([dot_search1,dot_search2,dot_search3,dot_search4,dot_search5,...
                dot_cover1,dot_cover2,dot_cover3,dot_cover4,...
                line_traj,dot_tar,cyl_obs],...
                {'UAV1 search point','UAV2 search point',...
                'UAV3 search point','UAV4 search point','UAV5 decision point',...
                'UAV1 search area','UAV2 search area',...
                'UAV3 search area','UAV4 search area',...
                'UAV trajectory','Target','Obstacle'});
    end

    lgd.Location='eastoutside';
    lgd.Orientation='vertical';
    lgd.NumColumns=1;
    lgd.FontSize=16;

    grid on;
    box on;
    drawnow;                                                                % Plot immediately
    g=gca;
    f=gcf;
    g.NextPlot='replacechildren';
    f.NextPlot='replacechildren';
    F(search_num)=getframe(f);
end
end

