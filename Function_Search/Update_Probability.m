%***********************************************************************
% Discription:  Update target existance probability
% input:        p                   Target existance probability
% input:        pd                  Target detection probability
% input:        pf                  Target false alarm probability
% input:        b                   Target flag
% output:       p_update            Updated target existance probability
%************************************************************************

function p_update = Update_Probability(p,pd,pf,b)
p_update=0;
if b==1
    p_update=pd*p/((pd*p)+(pf*(1-p)));          % Detedted target
end
if b==0
    p_update=(1-pd)*p/((1-pd)*p+(1-pf)*(1-p));  % Not detedted target
end
end

