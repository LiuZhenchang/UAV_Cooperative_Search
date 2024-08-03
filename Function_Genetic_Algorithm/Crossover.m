%**********************************************************************************
% Discription:  Realize gene crossover in genetic algorithms
% input:        pop                     Population Matrix
% input:        pc                      cross probability
% output:       newpop                  New population after gene crossover
%***********************************************************************************

function newpop=Crossover(pop,pc)
[a,b]=size(pop);                                % Obtain population number and gene length 
newpop=pop;                                     % Initial new population
% Take adjacent individuals for gene exchange
for i=1:2:a-1
    if(rand<pc)                                 % Determine whether crossover happens
        cpoint=round(rand*b);                   % Generate crossover position randomly
        newpop(i,:)=...
            [reshape(pop(i,1:cpoint),[],1);reshape(pop(i+1,cpoint+1:b),[],1)]';
        newpop(i+1,:)=...
            [reshape(pop(i+1,1:cpoint),[],1);reshape(pop(i,cpoint+1:b),[],1)]';
    end
end
end