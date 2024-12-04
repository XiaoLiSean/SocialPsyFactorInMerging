function hw = simulation_Ego_SingleStep(hw, actual_states)
    states_all  = [];
    action_all  = [];

    % Simulate policy and temporarily store the states
    states_tmp_sim      = cell(1,length(hw.Vehicles));
    actionTypes_tmp_sim = zeros(1,length(hw.Vehicles));
    for car_i = 1:length(hw.Vehicles)
        if car_i == hw.ego_id
            x0      = hw.Vehicles{car_i}.state;
            action  = hw.Vehicles{car_i}.ego_policy();
            CarPrevActDt.dtAct      = hw.simParams.dtSim;
            CarPrevActDt.prevActs   = hw.Vehicles{car_i}.prev_actions;
            [~, state_sim, ~]       = hw.simVehicle(x0, 'EM', action, CarPrevActDt);
        else
            state_sim       = actual_states((car_i*4-3):(car_i*4-1), :);
            actual_Vy       = actual_states(car_i*4, end);
            bestPrevActs    = highDInterpPrevActions(state_sim(:, end), actual_Vy, [0, 0],...
                hw.mpcParams, hw.simParams, hw.LaneParams, hw.actionParams);
            action          = bestPrevActs(end);
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
        hw.Vehicles{car_i}.prev_actions = [hw.Vehicles{car_i}.prev_actions, actionTypes_tmp_sim(car_i)];
    end
    hw.updateVehicleParams();
end