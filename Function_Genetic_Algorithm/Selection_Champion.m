%**********************************************************************************
% Discription:  Using tournament algorithm to select individuals
% input:        pop                     Population Matrix
% input:        fitvalue                The fitness of each individual corresponds 
%                                       to the result of the objective function
% output:       newpop                  New population after gene selection
%***********************************************************************************

function newpop=Selection_Champion(pop,fitvalue)
newpop=pop;
[m,~]=size(fitvalue);
n=ceil(0.5*m);
for i=1:m
    % Generate an array of n * 1 with random numbers ranging from 1 to m
    seletion_arr=randi([1,m],n,1);
    seletion_value=zeros(n,1);
    for j=1:n
        % Obtain fitness of n randomly selected individuals
        seletion_value(j,1)=fitvalue(seletion_arr(j));
    end
    % Obtain the individual number corresponding to the maximum fitness
    [~,b]=max(seletion_value);
    % Store the individual with the highest fitness to a new population
    newpop(i,:)=pop(seletion_arr(b,1),:);
end
end
