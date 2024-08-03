%***************************************************************************************************
% Discription:  Calculate the sum of the probability and uncertainty in the UAV FOV coverage area
%               as a partial result of the objective function
% input:        JA_table            Information of all grids covered by the FOV in the predicted path
% output:       JA1                 Sum of target existance probability
% output:       JA2                 Sum of environment uncertainty
%***************************************************************************************************

function [JA1,JA2] = J_A(JA_table)
sum_proba=0;
sum_uncer=0;
[num,~]=size(JA_table);                                         % Obtain the number of covered grid
%% Statistical the results in the JA_table
for j=1:num
    if JA_table(j,3)>0&&JA_table(j,4)>0
        % sum the target existence probability in the detection area
        sum_proba=sum_proba+JA_table(j,3);
        % sum the environment uncertainty in the detection area
        sum_uncer=sum_uncer+JA_table(j,4);
    end
end
JA1=sum_proba;
JA2=sum_uncer;
end


