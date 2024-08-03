%*****************************************************************
% Discription:  Calculate the coverage grid information of FOV
% input:        uav                 UAV structure
% output:       cover_grid          Grids covered by FOV
% output:       cover_num           Grids number covered by FOV
%*****************************************************************
function [cover_grid,cover_num] = Seeker(uav)
switch uav.seeker_type
    case 1
        [cover_grid,cover_num]=Seeker_Center(uav);      % Field of View (square)
    case 2
        [cover_grid,cover_num]=Seeker_Forward(uav);     % Field of View (forward rectangle)
end
end

