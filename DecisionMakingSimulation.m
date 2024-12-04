clear; clc; 
close all;

addpath('./decisionMaking', './HighDExperiment');

%% Initialization
initialParams   = highwayParamsHighD();
highwaySim      = Highway(initialParams);

%% Simulation
if_goal     = 0;
while (~if_goal)
    tic
    highwaySim.simulationSingleStep();
    if_goal     = (highwaySim.Vehicles{highwaySim.ego_id}.state(2) >= 0.49*highwaySim.LaneParams.lane_width);
%     if_goal     = (highwaySim.Vehicles{highwaySim.ego_id}.state(2) >= 0.49*highwaySim.LaneParams.lane_width) &&...
%         (highwaySim.Vehicles{highwaySim.ego_id}.state(1) >= highwaySim.LaneParams.rightBound);
    toc
end

save('./simulationHistory/highwayEgo.mat','highwaySim');
generateFilteringHistory('./simulationHistory/highwayEgo.mat');
generateAnimation('./simulationHistory/highwayEgo.mat');
