%**********************************************************************************
% Discription:  Generate state sequence according to action sequence
% input:        uav                     UAV structure
% input:        individual              Action sequence
% output:       position_series         State sequence
% output:       yaw_series              Yaw sequence
% output:       flag                    Exceed boundary flag
%***********************************************************************************

function [position_series,yaw_series,flag]=Generate_Route(uav,individual)

[~,gene_num]=size(individual);                          % Obtain gene length (sequence length)
position_series=zeros(gene_num,2);                      % Initial state sequence
yaw_series=zeros(gene_num,1);                           % Initial yaw sequence
yaw_new=uav.yaw;
xg_new=uav.xg;
yg_new=uav.yg;
flag=0;                                                 % Exceed boundary flag
k=uav.search_jump;                                      % Obtain jump grid number
for i=1:gene_num
    yaw_new=yaw_new+individual(1,i)*(2*pi/(8*k));       % Calculate yaw angle corrsponding to  current gene (action)
    %% Limit the yaw angle within the range of - pi to pi
    if yaw_new>pi
        yaw_new=yaw_new-2*pi;
    elseif yaw_new<-pi
        yaw_new=yaw_new+2*pi;
    end
    yaw_series(i,:)=yaw_new;                            % Save yaw angle corrsponding to  current gene
    n=ceil((yaw_new-2*pi/(16*k))/(2*pi/(8*k)));         % Calculate grid number

    %% Calculate the x-direction displacement of the grid
    if n>=-4*k && n<=-3*k
        d_xg=-k;
    elseif n>=-3*k && n<=-k
        d_xg=2*k+n;
    elseif n>=-k && n<=k
        d_xg=k;
    elseif n>=k && n<=3*k
        d_xg=2*k-n;
    elseif n>=3*k && n<=4*k
        d_xg=-k;
    else
        error('Generate Route X grid error');
    end
    %% Calculate the y-direction displacement of the grid
    if n>=-4*k && n<=-3*k
        d_yg=-n-4*k;
    elseif n>=-3*k && n<=-k
        d_yg=-k;
    elseif n>=-k && n<=k
        d_yg=n;
    elseif n>=k && n<=3*k
        d_yg=k;
    elseif n>=3*k && n<=4*k
        d_yg=-n+4*k;
    else
        error('Generate Route Y grid error');
    end
    %% Update and save grid coordinates
    xg_new=xg_new+d_xg;
    yg_new=yg_new+d_yg;
    position_series(i,:)=[xg_new,yg_new];
end
end