%*****************************************************************
% Discription:  Update the maximum and minimum values of input data
% input:        num                 current datat
% input:        min                 minimum data
% input:        max                 maximun data
% output:       min                 updated minimum data
% output:       max                 updated maximun data
%*****************************************************************
function [min,max] = Min_Max(num, min, max)
if num<min
    min=num;
end
if num>max
    max=num;
end
end

