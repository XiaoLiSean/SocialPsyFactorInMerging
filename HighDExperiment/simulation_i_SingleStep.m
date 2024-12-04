function hw = simulation_i_SingleStep(hw, sim_i, actual_states, actual_yVelocity, params, updateCtrl)
    states_all  = [];
    action_all  = [];

    % Simulate policy and temporarily store the states
    states_tmp_sim      = cell(1,length(hw.Vehicles));
    actionTypes_tmp_sim = zeros(1,length(hw.Vehicles));
    for car_i = 1:length(hw.Vehicles)
        if car_i == sim_i
            % Assign initial state if car_i not presented yet
            if isequal(hw.Vehicles{car_i}.state, [-inf; 0; 0])
                hw.Vehicles{car_i}.state    = actual_states((car_i*3-2):(car_i*3), 1);
            end 
            % -------------------------------------------------------------
            % Append previous actions if car_i is changing lane when
            % presenting to the simulation
            % -------------------------------------------------------------
            dkLaneCtrl      = round(params.mpcParams.HorizonLane * params.mpcParams.dtHorizon / params.simParams.dtCtrl);
            CurLaneId       = ceil(hw.Vehicles{car_i}.state(2) / params.LaneParams.lane_width);
            MissDistance    = abs(hw.Vehicles{car_i}.state(2) - params.LaneParams.lane_width*(CurLaneId - 0.5));
            if abs(actual_yVelocity(car_i, 1)) >= params.LaneParams.highDMinVy &&...
                    isempty(hw.Vehicles{car_i}.prev_actions) &&...
                    MissDistance > params.LaneParams.epsilon_lc % case just starting lane change
                % ---------------------------------------------------------
                changeDir           = 4.5 - 0.5*sign(actual_yVelocity(car_i, 1));
                bestMissDistance    = 0.5*params.LaneParams.lane_width;
                bestPrevActions     = hw.Vehicles{car_i}.prev_actions;
                for k_cur = 1:(dkLaneCtrl-1)
                    prevActions         = [hw.Vehicles{car_i}.prev_actions, changeDir*ones(1, k_cur)];
                    LaneChangeEndState  = hw.Vehicles{car_i}.state;
                    for addedLaneChangeStep = 1:(dkLaneCtrl-k_cur)
                        [LaneChangeSim, ~]  = LaneChangeTraj(LaneChangeEndState(1), LaneChangeEndState(2),...
                            LaneChangeEndState(3), changeDir, [prevActions, changeDir*ones(1, k_cur-1)],...
                            params.actionParams.v_lim, params.actionParams.a_lim,...
                            params.LaneParams.lane_width, dkLaneCtrl,...
                            params.simParams.dtCtrl, params.simParams.dt);
                        LaneChangeEndState  = LaneChangeSim(:, end);
                    end
                    CurLaneId           = ceil(LaneChangeEndState(2) / params.LaneParams.lane_width);
                    MissDistance        = abs(LaneChangeEndState(2) - params.LaneParams.lane_width*(CurLaneId - 0.5));
                    if MissDistance < bestMissDistance
                        bestMissDistance    = MissDistance;
                        bestPrevActions     = prevActions;
                    end
                end
                hw.Vehicles{car_i}.prev_actions = bestPrevActions;
                numLaneChangeActions            = sum(bestPrevActions == changeDir) * round(params.simParams.dtCtrl / params.simParams.dtSim);
                hw.actHistory                   = [hw.actHistory, zeros(length(hw.Vehicles), numLaneChangeActions)];
                hw.actHistory(car_i, (end-numLaneChangeActions+1):end)  = changeDir;
            end
            % -------------------------------------------------------------
            % Modify actionTypeLists: e.g., car (13 in 59 upper highD)
            % change from lane 2 to 3, at new filtering step, it's
            % already in lane 3, thus the action 4 won't be in the 
            % actionTypeLists. But lane changing is imcomplete, we need to
            % append actionType = 4 into the list for the policy search
            % -------------------------------------------------------------
            if ~isempty(hw.Vehicles{car_i}.prev_actions) &&...
                    ismember(hw.Vehicles{car_i}.prev_actions(end), [4,5]) &&...
                    ~ismember(hw.Vehicles{car_i}.prev_actions(end), hw.Vehicles{car_i}.actionTypeLists)
                laneChangeDirection     = hw.Vehicles{car_i}.prev_actions(end);
                consecutiveLaneChanges  = 0;
                for k = length(hw.Vehicles{car_i}.prev_actions):-1:1
                    if hw.Vehicles{car_i}.prev_actions(k) == laneChangeDirection
                        consecutiveLaneChanges  = consecutiveLaneChanges + 1;
                    else
                        break;
                    end
                end
                if mod(consecutiveLaneChanges, dkLaneCtrl) ~= 0
                    hw.Vehicles{car_i}.actionTypeLists  = [hw.Vehicles{car_i}.actionTypeLists, laneChangeDirection];
                end
            end
            % -------------------------------------------------------------
            x0      = hw.Vehicles{car_i}.state;
            if updateCtrl
                action  = hw.maximum_expectation_policy(car_i);
            else
                action  = hw.Vehicles{car_i}.prev_actions(end);
            end
            CarPrevActDt.dtAct      = params.simParams.dtSim;
            if ~isempty(hw.actHistory)
                CarPrevActDt.prevActs   = hw.actHistory(car_i, :);
            else
                CarPrevActDt.prevActs   = [];
            end
            [~, state_sim, ~]       = hw.simVehicle(x0, 'EM', action, CarPrevActDt);
        else
            action      = 0;
            state_sim   = actual_states((car_i*3-2):(car_i*3), :);
        end
        states_tmp_sim{car_i}       = state_sim(:, end);
        actionTypes_tmp_sim(car_i)  = action;
        if size(states_all, 2) == 0 % case vizHistory Ends
            states_all  = [states_all; state_sim];
        else
            states_all  = [states_all; state_sim(:, 1:size(states_all, 2))];
        end
        action_all  = [action_all; action];
    end
    
    hw.vizHistory   = [hw.vizHistory, states_all(:,2:end)]; % remove initial state
    hw.actHistory   = [hw.actHistory, action_all];
    
    % Update all state at the same time, to avoid modifying state
    % during policy making    
    for car_i = 1:length(hw.Vehicles)        
        hw.Vehicles{car_i}.state        = states_tmp_sim{car_i};
        if updateCtrl
            hw.Vehicles{car_i}.prev_actions = [hw.Vehicles{car_i}.prev_actions, actionTypes_tmp_sim(car_i)];
        end
    end
    hw.updateVehicleParams();
end