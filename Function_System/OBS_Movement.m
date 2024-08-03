%*****************************************************************
% Discription:  Update obstacle positions
% input:        time                time structure
% input:        OBS                 obstacles structure array
% input:        map                 Map structure
% output:       OBS                 obstacles structure array
%*****************************************************************
function OBS = OBS_Movement(OBS,time,map)
[~,obs_num]=size(OBS);
for i=1:obs_num
    %% Update obstacle positions
    OBS(i).history_position(time.step,1)=OBS(i).x;
    OBS(i).history_position(time.step,2)=OBS(i).y;
    OBS(i).x=OBS(i).x+OBS(i).speed*cos(OBS(i).psi)*time.delta;
    OBS(i).y=OBS(i).y+OBS(i).speed*sin(OBS(i).psi)*time.delta;
    
    %% Detect whether obstacles collide with the boundary
    if OBS(i).x>=map.x_max||OBS(i).x<=map.x_min
        OBS(i).psi=-(OBS(i).psi+pi);
        if OBS(i).psi>pi
            OBS(i).psi=OBS(i).psi-2*pi;
        elseif OBS(i).psi<-pi
            OBS(i).psi=OBS(i).psi+2*pi;
        end
    end
    if OBS(i).y>=map.y_max||OBS(i).y<=map.y_min
        OBS(i).psi=-OBS(i).psi;
    end
end
end

