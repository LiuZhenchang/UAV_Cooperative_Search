%**********************************************************************************
% Discription:  Using roulette algorithm to select individuals
% input:        pop                     Population Matrix
% input:        fitvalue                The fitness of each individual corresponds 
%                                       to the result of the objective function
% output:       newpop                  New population after gene selection
%***********************************************************************************
function newpop=Selection_Roulette(pop,fitvalue)

    toltalfit=sum(fitvalue);                    % Calculate the total fitness of all individuals
    fitvalue=fitvalue/toltalfit;                % Calculate the average fitness of all individuals
    fitvalue=cumsum(fitvalue);                  % Accumulate fitness in array order sequentially
    [m,~]=size(fitvalue);                       % Obtain population number
    [a,b]=size(pop);                            % Obtain population number and gene length
    newpop=zeros(a,b);                          % Initial new population
    ms=sort(rand(m,1));
    fitin=1;
    newin=1;
    % Using roulette algorithm to generate new population
    while newin<=m
        if ms(newin)<fitvalue(fitin)
            newpop(newin,:)=pop(fitin,:);
            newin=newin+1;
        else
            fitin=fitin+1;
        end
    end
end