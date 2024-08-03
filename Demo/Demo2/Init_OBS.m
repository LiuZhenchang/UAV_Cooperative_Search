%*****************************************************************
% Discription:  Initialize obstacles
% input:        OBS             Obstacles structure array
%*****************************************************************

function OBS = Init_OBS(OBS)
OBS(1).x=200*2;
OBS(1).y=100*2;
OBS(1).psi=100/180*pi;
OBS(1).speed=0;
OBS(1).r=30*2;

OBS(2).x=350*2;
OBS(2).y=200*2;
OBS(2).psi=-130/180*pi;
OBS(2).speed=0;
OBS(2).r=40*2;

OBS(3).x=450*2;
OBS(3).y=100*2;
OBS(3).psi=60/180*pi;
OBS(3).speed=0;
OBS(3).r=20*2;

OBS(4).x=550*2;
OBS(4).y=200*2;
OBS(4).psi=-20/180*pi;
OBS(4).speed=0;
OBS(4).r=30*2;

OBS(5).x=600*2;
OBS(5).y=300*2;
OBS(5).psi=-60/180*pi;
OBS(5).speed=0;
OBS(5).r=20*2;

OBS(6).x=650*2;
OBS(6).y=100*2;
OBS(6).psi=180/180*pi;
OBS(6).speed=0;
OBS(6).r=40*2;
end

