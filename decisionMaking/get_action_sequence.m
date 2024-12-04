function Actions_set = get_action_sequence(simParams, mpcParams, actionTypeList, previous_actions, laneChangeAugmented, laneChangeAbort)
    % -------------------------------------------------------------
    % Vehicle's action:
    % 1: slow down
    % 2: maintain lane and keep speed
    % 3: speed up 
    % 4: change lane left 
    % 5: change lane right 
    % Ego Vehicle's action:
    % 6: change lane left while decelerating 
    % 7: change lane right while decelerating  
    % 8: change lane left while accelerating  
    % 9: change lane right while accelerating  
    % -------------------------------------------------------------
    % numbers of control steps needed for a horizon: the control signals 
    % of length "minSteps" within a horizon are assumed to be identical
    minSteps    = floor(mpcParams.dtHorizon / simParams.dtCtrl); 
    nStepsLane  = minSteps * mpcParams.HorizonLane; % numbers of control steps needed for a lane change

    previous_actions    = [0, previous_actions]; % append zero in case this become empty in the following process
    % -------------------------------------------------------------
    % Remove longitudinal behavior from lane change
    actionTypeList  = setdiff(actionTypeList, 6:9);
    previous_actions(ismember(previous_actions, [6 8])) = 4;
    previous_actions(ismember(previous_actions, [7 9])) = 5;
    
    % -------------------------------------------------------------
    % Generate reference/truncated action sets and remove abnormal lane changes
    neededMoreSteps             = 0;
    if ismember(previous_actions(end), [4 5]) 
        laneChangeDirection         = previous_actions(end);
        consecutiveLaneChanges      = 0;
        for k = length(previous_actions):-1:1
            if previous_actions(k) == laneChangeDirection
                consecutiveLaneChanges  = consecutiveLaneChanges + 1;
            else
                break;
            end
            
        end
        if mod(consecutiveLaneChanges, nStepsLane) ~= 0
            neededMoreSteps = nStepsLane - mod(consecutiveLaneChanges, nStepsLane);
        end

    
        % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        % search if there is an lane change aborting, if there is any, is
        % it completed/active? if no we could add lane change abortion
        % later (shall be a complete lane change abortion)
        thereIsAbortion = false;
        if  nargin > 4 && laneChangeAbort
            subprevious_actions = previous_actions(1:end-consecutiveLaneChanges); % action before the most recent lane change
            if ismember(subprevious_actions(end), setdiff([4 5], previous_actions(end)) ) % different consecutive lane changes
                subConsecutiveLaneChanges   = 0;
                for k = length(subprevious_actions):-1:1
                    if subprevious_actions(k) == subprevious_actions(end)
                        subConsecutiveLaneChanges  = subConsecutiveLaneChanges + 1;
                    else
                        break;
                    end
                    
                end
                if mod(subConsecutiveLaneChanges, nStepsLane) ~= 0 % there is a lane change abortion (imcomplete lane change)
                    % case subConsecutiveLaneChanges > nStepsLane, i.e., two consecutive lane change
                    subConsecutiveLaneChanges   = mod(subConsecutiveLaneChanges, nStepsLane); 
                    thereIsAbortion     = true;
                    
                    if mod(abs(sum(subprevious_actions == 4) - sum(subprevious_actions == 5)), nStepsLane) == 0
                        abortionComplete    = true;
                    else
                        abortionComplete    = (consecutiveLaneChanges >= subConsecutiveLaneChanges);
                        if abortionComplete % we can proceed with current lane change if it's incomplete
                            consecutiveLaneChanges  = consecutiveLaneChanges - subConsecutiveLaneChanges;        
                            if mod(consecutiveLaneChanges, nStepsLane) ~= 0
                                neededMoreSteps = nStepsLane - mod(consecutiveLaneChanges, nStepsLane);
                            else
                                neededMoreSteps = 0;
                            end
                        else % current abortion is incomplete
                            neededMoreSteps = subConsecutiveLaneChanges - consecutiveLaneChanges;
                        end
                    end 
                end
            end
        end
        % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    end

    % -------------------------------------------------------------
    % fill the truncated action sets, tail actions of lane change are filled with 2 (const speed)
    Actions_set = fillInTruncatedSets(actionTypeList, mpcParams, minSteps, previous_actions, neededMoreSteps, previous_actions(end));

    % -------------------------------------------------------------
    % add lane change abortion if needed: we assume the aborting must be 
    % completed, otherwise it will be unneccessarily complicated for coding
    if  nargin > 4 && laneChangeAbort && ismember(previous_actions(end), [4 5]) && (neededMoreSteps ~= 0) % abortion could be initiated
        if ~thereIsAbortion || (thereIsAbortion && abortionComplete) % previous has no abortion or abortion completed  
            laneChangeDir       = setdiff([4 5], previous_actions(end));
            Actions_Abort_Aug   = fillInTruncatedSets(actionTypeList, mpcParams, minSteps, previous_actions, nStepsLane - neededMoreSteps, laneChangeDir);
            Actions_set         = [Actions_set; Actions_Abort_Aug];
        end
    end

    % -------------------------------------------------------------
    % add longitudinal behavior into lane change if needed
    if  nargin > 4 && laneChangeAugmented
        for horizon_i = 1:mpcParams.nHorizon
            Actions_set_tmp = Actions_set;
            for seq_i = 1:size(Actions_set, 1)
                act_seg     = Actions_set(seq_i, (1+(horizon_i-1)*minSteps):(horizon_i*minSteps));

                leftChangeIdxs  = (horizon_i-1)*minSteps + find(act_seg == 4);
                rightChangeIdxs = (horizon_i-1)*minSteps + find(act_seg == 5);

                if ~isempty(leftChangeIdxs)
                    actions     = repmat([6; 8], 1, length(leftChangeIdxs));
                    actionsAug  = repmat(Actions_set(seq_i, :), size(actions, 1), 1);
                    actionsAug(:, leftChangeIdxs)   = actions;
                    Actions_set_tmp     = [Actions_set_tmp; actionsAug];
                end
                
                if ~isempty(rightChangeIdxs)
                    actions     = repmat([7; 9], 1, length(rightChangeIdxs));
                    actionsAug  = repmat(Actions_set(seq_i, :), size(actions, 1), 1);
                    actionsAug(:, rightChangeIdxs)   = actions;
                    Actions_set_tmp     = [Actions_set_tmp; actionsAug];          
                end
            end
            Actions_set     = Actions_set_tmp;
        end
    end

end

function Actions_set = fillInTruncatedSets(actionTypeList, mpcParams, minSteps, previous_actions, neededMoreSteps, laneChangeDir)
    Actions_set_truncated   = permn(actionTypeList, mpcParams.nHorizon - ceil(neededMoreSteps / minSteps)); 
    Actions_set_truncated   = removeAbnormalLaneChanging(Actions_set_truncated, 0, actionTypeList, mpcParams.HorizonLane);
    if ismember(previous_actions(end), [4 5]) && neededMoreSteps ~= 0
        Actions_set = laneChangeDir*ones(size(Actions_set_truncated, 1), neededMoreSteps);
        for j = 1:size(Actions_set_truncated, 2)
            Actions_set = [Actions_set, repmat(Actions_set_truncated(:, j), 1, minSteps)];
        end
        defict_size = max(minSteps*mpcParams.nHorizon - size(Actions_set, 2), 0);
        Actions_set = [Actions_set(:, 1:neededMoreSteps),...
            2*ones(size(Actions_set_truncated, 1), defict_size),...
            Actions_set(:, (1+neededMoreSteps):end)];
    else
        Actions_set     = [];
        for j = 1:size(Actions_set_truncated, 2)
            Actions_set = [Actions_set, repmat(Actions_set_truncated(:, j), 1, minSteps)];
        end
    end
end

function Actions_set = removeAbnormalLaneChanging(Actions_set_tmp, previous_actions, actionTypeList, dkLane)
    
    Actions_set     = Actions_set_tmp;
    remove_idx      = []; % indexes that need to be removed for illegal lane change
    horizon         = size(Actions_set_tmp, 2);
    % ===========================================================
    % This program is used to remove illegal action sequence
    % to enforce that lane change share be exactly dkLane steps
    % ===========================================================
    % decisions for half-changing lanes is illegal: remove the case where
    % driver abort the lane change     
    for seq_i = 1:size(Actions_set_tmp, 1)
        
        act_seq     = Actions_set_tmp(seq_i, :); % row vector
        act_seq_aug = act_seq;
        
        % -----------------------------------------------------------
        % Case 1: abort lane changing that is happening (ongoing lane change)
        % -----------------------------------------------------------
        if ismember(previous_actions(end), [4,5]) % this means we do have lane change previously
            laneChangeDirection     = previous_actions(end);
            % this means the lane change is not ended
            
            consecutiveLaneChanges  = 0;
            for k = length(previous_actions):-1:1
                if previous_actions(k) == laneChangeDirection
                    consecutiveLaneChanges  = consecutiveLaneChanges + 1;
                else
                    break;
                end
            end
            
            if mod(consecutiveLaneChanges, dkLane) == 0
                neededMoreSteps = 0;
            else                
                neededMoreSteps = dkLane - mod(consecutiveLaneChanges, dkLane);
            end
            
            if ~isequal(laneChangeDirection*ones(1, neededMoreSteps), act_seq(1, 1:neededMoreSteps))
                remove_idx  = [remove_idx, seq_i];
                continue;
            else
                % remove feasible on-going merging part for later check
                act_seq_aug(1, 1:neededMoreSteps)   = 0; 
            end
        
        end 
        
        % -----------------------------------------------------------
        % Case 2: abort lane changing within prediction
        % -----------------------------------------------------------
        % Remove the tail lane change sequence, cuz it's always feasible
        if ismember(act_seq(end), [4,5])
            for k = length(act_seq):-1:1
                if act_seq_aug(k) == act_seq(end)
                    act_seq_aug(k) = 0;
                else
                    break;
                end
            end
        end
        % used to consider the window shifting of the first few anchors
        act_seq_aug     = [zeros(1, dkLane-1), act_seq_aug]; 
        
        % -----------------------------------------------------------
        % -----------------------------------------------------------        
        is_seq_i_legit  = true;
        for laneChangeDirection = 4:5
            if ~ismember(laneChangeDirection, actionTypeList)
                continue;
            end
            % check if all the window of size dkLane that contains the
            % anchor_i'th action having illegal lane change
            for anchor_i = dkLane:(dkLane+horizon-1)
                if act_seq_aug(anchor_i) ~= laneChangeDirection
                    continue; % no lane change in this anchor
                end
                % check all the windows of length dkLane that contains the 
                % anchor_i'th action, if there exist exactly one good
                % window that is a complete lane change                
                window_i_legit_count    = 0;
                for window_i = 1:min(dkLane, horizon+dkLane-anchor_i)
                    act_in_window_i     = act_seq_aug(1, (anchor_i-dkLane+window_i):(anchor_i+window_i-1));
                    if isequal(act_in_window_i, laneChangeDirection*ones(1, dkLane))
                        window_i_legit_count    = window_i_legit_count + 1;
                    end
                end
                if window_i_legit_count ~= 1
                    is_seq_i_legit  = false;
                    break;
                end
            end
            if ~is_seq_i_legit
                remove_idx  = [remove_idx, seq_i];
                break;
            end
        end
        
    end
    
    idxes   = unique(remove_idx);
    Actions_set(idxes, :)   = [];
end