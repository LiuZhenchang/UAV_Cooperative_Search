%***********************************************************************
% Discription:  Calculate maneuver cost according to action sequence
%               as a partial result of the objective function
% input:        individual          Current action sequence 
% output:       sum_maneu           Maneuver cost
%***********************************************************************
function sum_maneu = J_B(individual)
sum_maneu=0;
[~,m]=size(individual);
for i=1:m-1
    % Accumulate the change value of each maneuver
    sum_maneu=sum_maneu+abs(individual(i+1)-individual(i));
end
sum_maneu=sum_maneu/(m-1);
end

