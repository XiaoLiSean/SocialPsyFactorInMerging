function [bestParams, bestParamsIdx, predictionScores, trajectories, S_set, W_set] = SearchOptimalParametersHighD(params, vizHistory, startFrame, endFrame)

    weight_num  = 3 + (~params.rewardParams.enforceCollision);
    Importance  = permn(0:1, weight_num);
    Weights     = Importance./repmat(sum(Importance,2), 1, weight_num);
    W_set       = unique(Weights, 'rows');
    W_set(any(isnan(W_set), 2), :)  = [];
    S_set       = [90, 45, 0, -45]; 
    
    num_veh             = size(vizHistory, 2);
    dkSim               = round(params.simParams.dtSim/params.simParams.dt);
    bestParams          = cell(num_veh, 1);
    bestParamsIdx       = cell(num_veh, 1);
    predictionScores    = cell(num_veh, 1);
    trajectories        = cell(num_veh, length(S_set), size(W_set, 1));
    for car_i = 1:num_veh
        if sum(cellfun(@isempty, vizHistory(startFrame:endFrame, car_i))) == (endFrame-startFrame+1)
            continue;
        end
        [predictionScores_car_i, trajectories_car_i]    =...
            SearchOptimalParametersHighD_Car_i(car_i, params, vizHistory,...
            startFrame, endFrame, W_set, S_set, num_veh, dkSim);
        predictionScores{car_i}     = predictionScores_car_i;
        trajectories(car_i, :, :)   = trajectories_car_i;
        maxScoreIDx                 = find(predictionScores{car_i} == max(predictionScores{car_i}, [], 'all'));
        [s_best, w_best]            = ind2sub(size(predictionScores{car_i}), maxScoreIDx);
        bestParams{car_i}           = [S_set(s_best)', W_set(w_best, :)];
        bestParamsIdx{car_i}.svo    = s_best;
        bestParamsIdx{car_i}.w      = w_best;
        save('./HighDExperiment/HighDResults/searchResults.mat', 'trajectories', 'predictionScores', 'bestParams', 'bestParamsIdx', 'S_set', 'W_set');    
    end 
end