%% Add path
addpath Function_System;
addpath Function_Initiate;
addpath Function_Plot;
addpath Function_Communication;
addpath Function_Search;
addpath Function_Genetic_Algorithm;
addpath Function_Objective;

% demo_index=1, simulation of 4 homogeneous UAVs, with communication constraints and stationary obstacles
% demo_index=2, simulation of 5 heterogeneous UAVs, with communication constraints and stationary obstacles
% demo_index=3, simulation of 5 heterogeneous UAVs, with communication constraints and dynamic obstacles

demo_index=1;
demo_path=['Demo/Demo' sprintf('%d',demo_index)];
demo_name=['demo' sprintf('%d',demo_index) '.m'];
addpath(demo_path);                     
run(demo_name)
