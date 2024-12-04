function [predictionScores, trajectories] = SearchOptimalParametersHighD_Car_i(car_i, params, vizHistory, startFrame, endFrame, W_set, S_set, num_veh, dkSim)   

    progressBar     = waitbar(0, sprintf('Searching: car %d', car_i));
    waitBarCount    = 0; 
    timeCount       = 0;
    tic

    predictionScores        = zeros(length(S_set), size(W_set, 1));
    trajectories            = cell(length(S_set), size(W_set, 1));
    car_i_presented_frames  = find(cellfun(@isempty, vizHistory(:, car_i)) == 0);
    car_i_startFrame        = max(startFrame, car_i_presented_frames(1));
    timeSteps               = ceil((endFrame-car_i_startFrame)/dkSim);
    stepsCtrl               = round(params.simParams.dtCtrl/params.simParams.dtSim);

    if length(vizHistory{car_i_presented_frames(1), car_i}) ~= 4
        error('yVelocity must be provided in vizHistory');
    end
    for s_i = 1:length(S_set)
        waitbar(waitBarCount/length(S_set),...
                progressBar, sprintf('Searching: car %d / %0.2f sec', car_i, timeCount));   
        % =================================================================
        % Multi-threading
        % =================================================================
        svo         = S_set(s_i);               
        traj_tmp    = cell(1, size(W_set, 1));
        score_tmp   = predictionScores(s_i, :);            
        if svo == 90
            nontrivial_w_num    = 1; % altruistic don't care self reward
        else
            nontrivial_w_num    = size(W_set, 1);
        end

        parfor w_i = 1:nontrivial_w_num   
            % -------------------------------------------------------------          
            % Initialize highway
            % -------------------------------------------------------------
            weight      = W_set(w_i, :); 
            SVOs        = svo.*ones(num_veh, 1);
            Weights     = repmat(weight, num_veh, 1);
            
            X_raw                               = vizHistory(car_i_startFrame,:);
            X_raw(1, cellfun(@isempty, X_raw))  = {[-inf, 0, 0, 0]}; % vehicle not yet appear                
            X0          = cell2mat(X_raw')';
            Input       = cell(num_veh, 3);
            for ii = 1:num_veh
                Input(ii,:)     = {X0(1:3,ii), 'SVO', [SVOs(ii), Weights(ii,:)]};
            end
            initialParams               = params;
            initialParams.x0_vehicles   = Input;
            highwaySim                  = Highway(initialParams);
            highwaySim.vizHistory       = reshape(X0(1:3,:), [], 1);
            % -------------------------------------------------------------
            % Simulate each individual time step
            % -------------------------------------------------------------
            traj_tmp{w_i}   = repmat([-inf; 0; 0], 1, car_i_startFrame - 1);
            traj_tmp{w_i}   = cat(2, traj_tmp{w_i}, X0(1:3, car_i));
            for step_i = 1:timeSteps    
                frame_i     = (step_i-1)*dkSim + car_i_startFrame;
                if isempty(vizHistory{frame_i, car_i})
                    state_i_pred    = repmat([-inf; 0; 0], 1, dkSim);
                else
                    X_raw                           = vizHistory(frame_i:min(frame_i+dkSim, size(vizHistory, 1)),:);
                    X_raw(cellfun(@isempty, X_raw)) = {[-inf, 0, 0, 0]}; % vehicle not yet appear                
                    actual_states                   = cell2mat(X_raw)';
                    actual_yVelocity                = actual_states(4:4:end, :);
                    actual_states(4:4:end, :)       = [];
                    updateCtrl      = (mod(step_i-1, stepsCtrl) == 0);
                    highwaySim      = simulation_i_SingleStep(highwaySim, car_i, actual_states, actual_yVelocity, params, updateCtrl);
                    state_i_pred    = highwaySim.vizHistory((car_i*3-2):(car_i*3), max(1, size(highwaySim.vizHistory, 2)-dkSim+1):end);
                end
                traj_tmp{w_i}   = cat(2, traj_tmp{w_i}, state_i_pred);
            end
            % -------------------------------------------------------------                
            % Calculate scores of trajectory prediction
            % -------------------------------------------------------------
            car_i_simulated_frames  = find(traj_tmp{w_i}(1,:) ~= -inf);
            if isempty(car_i_simulated_frames)
                continue;
            end
            eval_start_frame        = max(car_i_presented_frames(1), car_i_simulated_frames(1));
            eval_end_frame          = min(car_i_presented_frames(end), car_i_simulated_frames(end));
            if eval_start_frame < eval_end_frame
                actual_traj_car_i   = cell2mat(vizHistory(eval_start_frame:eval_end_frame, car_i))';
                pred_traj_car_i     = traj_tmp{w_i}(:, eval_start_frame:eval_end_frame);
                score_tmp(w_i)      = similarityMetric(actual_traj_car_i(1:3,:), pred_traj_car_i, params);
            else
                error('no usable evaluable frames');
            end
        end
        trajectories(s_i, :)        = traj_tmp;
        predictionScores(s_i, :)    = score_tmp;  
        waitBarCount    = waitBarCount + 1;
        timeCount       = toc;
    end  
    close(progressBar);      
end