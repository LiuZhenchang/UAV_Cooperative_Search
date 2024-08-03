%**********************************************************************************
% Discription:  Realize gene mutation in genetic algorithms
% input:        pop                     Population Matrix
% input:        pm                      mutation probability
% output:       newpop                  New population after gene mutation
%***********************************************************************************

function newpop=Mutation(pop,pm)
[a,b]=size(pop);                            % Obtain population number and gene length
newpop=pop;                                 % Initial new population
for i=1:a
    if rand<pm                              % Determine whether mutation happens      
        mpoint=round(rand*b);               % Generate mutation position randomly
        if mpoint<=0
            mpoint=1;
        end
        newpop(i,mpoint)=randi([-1,1]);     % Generate mutation gene randomly     
    end
end
end