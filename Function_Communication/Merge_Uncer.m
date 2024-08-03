%********************************************************************
% Discription:  Merging the environment uncertainty
% input:        uncer1              UAV1 (Self)
% input:        uncer2              UAV2 (source of information)
% putput:       uncer_m             Merged environment uncertainty
%********************************************************************
function uncer_m = Merge_Uncer(uncer1,uncer2)
[x_num,y_num]=size(uncer1);
uncer_m=zeros(x_num,y_num);
for i=1:x_num
    for j=1:y_num
        % Take the minimum uncertainty value within each UAV's grid during merging
        uncer_m(i,j)=min(uncer1(i,j),uncer2(i,j));
    end
end
end

