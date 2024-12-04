clear; clc; 
close all;

addpath('./decisionMaking', './HighDExperiment');
if isfile('./HighDExperiment/HighDResults/DecisionMakingResults/statistics.mat')
    load('./HighDExperiment/HighDResults/DecisionMakingResults/statistics.mat', 'statistics', 'statsTable');
else
    statistics.totalCase    = 0;
    statistics.succCase     = 0;
    statistics.maxSteps     = 5;
    statistics.name         = {};
    statistics.isSucc       = [];
    statistics.FailType     = {};
    statistics.stepsUsed    = [];
end

for scene_i = [58, 59, 60]
    %% Initialization Parameters
    scene_num   = num2str(scene_i);
    which_side  = 'upper';
    [params, vizHistory]    = preprocessingHighD(scene_num, which_side, true);
    % ---------------------------------------------------------------------
    % -------------------------------------------------------------       
    % the simParams for ego and for the env vehicle shall be the 
    % same if you need to used the behavior model with Attention
    % ------------------------------------------------------------- 
    % --------------------------------------------------------------------- 
    params.simParams.dtSim      = 13*params.simParams.dt; 
    params.simParams.dtCtrl     = params.simParams.dtSim; 
    params.simParams.dtLane     = 8*params.simParams.dtSim; 
    % -------------------------------------------------------------
    % Parameters for Ego Decision
    % -------------------------------------------------------------
    params.EgoParams.laneChangeAug      = true; % consider acc and dec in lane change
    params.EgoParams.laneChangeAbort    = false; % consider lane change abortion
    params.EgoParams.prediction_threshold.params_p  = 1/22 - 1e-4; % cut the search tree using probability (parameter cmbns)
    params.EgoParams.prediction_threshold.actions_p = 1/225 - 1e-4; % cut the search tree using probability (action seq lists)
    params.EgoParams.prediction_threshold.actions_n = 5; % cut the search tree with fixed remaining branch (action seq lists)
    params.EgoParams.simParams          = params.simParams;
    params.EgoParams.simParams.dtCtrl   = params.simParams.dtCtrl; % faster ego decision
    params.EgoParams.simParams.dtBayes  = params.simParams.dtCtrl; % faster ego decision

    params.rewardParams.collisionMargin = [3, 0.5]; % this is used to counter some model error in prediction highway vehicles
    
    params.mpcParams.HorizonLane    = 2; % number of a MPC prediction horizon for lane change
    params.mpcParams.nHorizon       = params.mpcParams.HorizonLane + 1; % number of a MPC prediction horizon
    params.mpcParams.dtHorizon      = params.simParams.dtLane / params.mpcParams.HorizonLane; % time interval for a MPC prediction horizon
    % ---------------------------------------------------------------------
    car_num     = size(vizHistory, 2);
    weight_num  = 3 + (~params.rewardParams.enforceCollision);
    dkSim       = round(params.simParams.dtSim/params.simParams.dt);
    mergeData.car_id        = [];
    mergeData.startFrame    = [];

    minSteps    = floor(params.mpcParams.dtHorizon / params.simParams.dtCtrl); 
    nStepsLane  = minSteps * params.mpcParams.HorizonLane; % numbers of control steps needed for a lane change
    
    %% Search for Ramp Vehicles
    for car_i = 1:car_num
        startFrame  = find(cellfun(@isempty, vizHistory(:, car_i)) == 0, 1);    
        state_car_i = vizHistory{startFrame, car_i};
        lane_id     = ceil(state_car_i(2) / params.LaneParams.lane_width);
        if lane_id == 0
            mergeData.car_id        = [mergeData.car_id; car_i];
            mergeData.startFrame    = [mergeData.startFrame; startFrame];
        end
    end
    
    %% Perform unit test  
    timeCount       = 0; tic;  
    progressBar     = waitbar(0, sprintf('Testing: %s %s', scene_num, which_side));
    for test_i = 1:length(mergeData.car_id) % find(mergeData.car_id == 629)
        car_i               = mergeData.car_id(test_i);
        car_i_startFrame    = mergeData.startFrame(test_i);
        waitbar(test_i/length(mergeData.car_id),...
                progressBar, sprintf('Testing: %s %s ( car %d / %0.2f sec)', scene_num, which_side, car_i, timeCount)); 
        test_i_name     = sprintf('%s_car%d_F%d', scene_num, car_i, car_i_startFrame);
        if any(strcmp(statistics.name, test_i_name))
            timeCount       = toc;
            save('./HighDExperiment/HighDResults/DecisionMakingResults/statistics.mat', 'statistics', 'statsTable');
            continue;
        end
        % -------------------------------------------------------------
        % Fill in states for unobservable vehicle (constant speed)
        % ------------------------------------------------------------- 
        car_i_endFrame  = statistics.maxSteps*dkSim + car_i_startFrame;
        vizHistoryAug   = vizHistory;
        for opp_i = 1:car_num
            if opp_i == car_i
                continue;
            end
            opp_i_presented_frames  = find(cellfun(@isempty, vizHistory(:, opp_i)) == 0);
            if opp_i_presented_frames(1) < car_i_startFrame
                continue
            end
            if opp_i_presented_frames(1) > car_i_endFrame && ...
                    vizHistory{opp_i_presented_frames(1), opp_i}(1) < params.LaneParams.rampStartX
                continue            
            end
            for frame_i = (opp_i_presented_frames(1)-1):-1:car_i_startFrame                
                FutureState     = vizHistoryAug{frame_i+1, opp_i};
                FutureState(4)  = 0;
                PresentState    = FutureState;
                PresentState(1) = FutureState(1) - FutureState(3)*params.simParams.dt;
                vizHistoryAug(frame_i, opp_i)   = {PresentState};
            end
        end
        % -------------------------------------------------------------
        % Initialize the parameter
        % -------------------------------------------------------------
        X_raw                               = vizHistoryAug(car_i_startFrame, :);
        X_raw(1, cellfun(@isempty, X_raw))  = {[-inf, 0, 0, 0]}; % vehicle not yet appear                
        X0              = cell2mat(X_raw')';
        X0(4,:)         = [];
        X0(2, car_i)    = -0.5*params.LaneParams.lane_width; % put vehicle in the middle of a lane
        Input           = cell(car_num, 3);
        for ii = 1:car_num
            if ii == car_i
                Input(ii,:)     = {X0(:,ii), 'EGO', [0, zeros(1, weight_num)]};
            else
                Input(ii,:)     = {X0(:,ii), 'SVO', [0, zeros(1, weight_num)]};
            end
        end
        initialParams               = params;
        initialParams.x0_vehicles   = Input;
        highwaySim                  = Highway(initialParams);
        highwaySim.vizHistory       = reshape(X0, [], 1);
        % -------------------------------------------------------------
        % Simulate the virtual ego vehicle
        % -------------------------------------------------------------
        if_goal         = 0;
        is_collision    = false;
        is_outofbounds  = false;
        fail_type_i     = "na";
        step_i          = 1;
        test_i_success  = true;
        while (~if_goal) 
            % Simulation main
            frame_i     = (step_i-1)*dkSim + car_i_startFrame;
            X_raw                           = vizHistoryAug(frame_i:min(frame_i+dkSim, size(vizHistoryAug, 1)),:);
            X_raw(cellfun(@isempty, X_raw)) = {[-inf, 0, 0, 0]}; % vehicle not yet appear                
            actual_states                   = cell2mat(X_raw)';
            highwaySim  = simulation_Ego_SingleStep(highwaySim, actual_states);
            state_car_i = highwaySim.Vehicles{highwaySim.ego_id}.state;
            lane_id     = ceil(state_car_i(2) / params.LaneParams.lane_width);
            pre_act     = highwaySim.Vehicles{highwaySim.ego_id}.prev_actions;
            if_goal     = (highwaySim.Vehicles{highwaySim.ego_id}.state(2) >= 0.49*highwaySim.LaneParams.lane_width);
            % Check collision
            car_x_all   = highwaySim.vizHistory((3*highwaySim.ego_id-2):(3*highwaySim.ego_id), :);
            car_dim     = [highwaySim.Vehicles{highwaySim.ego_id}.vecLength; highwaySim.Vehicles{highwaySim.ego_id}.vecWidth];
            for opp_i = 1:car_num
                if opp_i == car_i || isequal(highwaySim.Vehicles{opp_i}.state, [-inf;0;0])
                    continue;
                end
                opp_x_all   = highwaySim.vizHistory((3*opp_i-2):(3*opp_i), :);
                opp_dim     = [highwaySim.Vehicles{opp_i}.vecLength; highwaySim.Vehicles{opp_i}.vecWidth];
                if collision_vehicles(car_x_all, car_dim, opp_x_all, opp_dim, [0.1, 0.1])
                    is_collision    = true;
                    fail_type_i     = "collision";  
                    break;
                elseif collision_out_of_bounds(car_x_all, car_dim, highwaySim.Vehicles{highwaySim.ego_id}.on_ramp, params.LaneParams)
                    is_outofbounds  = true;
                    fail_type_i     = "out of bounds";
                    break;
                end
            end
            if is_collision || is_outofbounds
                break;
            end
            % Check merging failure
            if step_i > statistics.maxSteps * minSteps && ~if_goal
                fail_type_i = "exceed maxSteps";         
                break;
            end
            if ~if_goal
                step_i      = step_i + 1;
            end
        end
        % -------------------------------------------------------------
        % Generate animation and store test data
        % -------------------------------------------------------------
        statistics.totalCase    = statistics.totalCase + 1;
        statistics.succCase     = statistics.succCase + test_i_success;
        statistics.name         = cat(1, statistics.name, test_i_name);
        statistics.isSucc       = [statistics.isSucc; test_i_success];
        statistics.FailType     = cat(1, statistics.FailType, fail_type_i);
        statistics.stepsUsed    = [statistics.stepsUsed; step_i];
        statsTable      = table(statistics.name, statistics.isSucc, statistics.FailType,...
            statistics.stepsUsed, 'VariableNames', ["Name"; "isSucc"; "FailType"; "stepsUsed"]);
        simTraj         = cell(1, car_num);
        simTraj{car_i}  = [repmat([-inf;0;0], 1, car_i_startFrame - 1), highwaySim.vizHistory((car_i*3-2):(car_i*3),:)];

        if ~if_goal || ~strcmp(fail_type_i, 'na')
            aniFileName     = sprintf('./HighDExperiment/HighDResults/DecisionMakingResults/FailCases/highD_%s_%s_car%d_F%d.gif',...
                scene_num, which_side, car_i, car_i_startFrame);
            simFileName     = sprintf('./HighDExperiment/HighDResults/DecisionMakingResults/FailCases/highD_%s_%s_car%d_F%d.mat',...
                scene_num, which_side, car_i, car_i_startFrame);
            fprintf('Testing Failed: %s %s ( car %d due to %s)\n', scene_num, which_side, car_i, fail_type_i);
        else
            aniFileName     = sprintf('./HighDExperiment/HighDResults/DecisionMakingResults/highD_%s_%s_car%d_F%d.gif',...
                scene_num, which_side, car_i, car_i_startFrame);
            simFileName     = sprintf('./HighDExperiment/HighDResults/DecisionMakingResults/highD_%s_%s_car%d_F%d.mat',...
                scene_num, which_side, car_i, car_i_startFrame);
        end
        generateHighDComparisonAnimation(params, vizHistoryAug, simTraj,...
            car_i_startFrame, car_i_startFrame + size(highwaySim.vizHistory, 2) - 1, aniFileName);
        save(simFileName, 'highwaySim')
        timeCount       = toc;
        save('./HighDExperiment/HighDResults/DecisionMakingResults/statistics.mat', 'statistics', 'statsTable');
    end
    close(progressBar); 
end