function [t_sim, state_sim, theta_sim] = VehicleSimulator(x0, actionType, prevActsAndDt, actionParams, simParams, mpcParams, LaneParams)
    dtAction    = prevActsAndDt.dtAct; % time intervals for controls
    prevActions = prevActsAndDt.prevActs; % previous controls
    dt          = simParams.dt;
    dkLane      = floor(mpcParams.dtHorizon / dtAction) * mpcParams.HorizonLane; % numbers of sim steps needed for a lane change
    lane_width  = LaneParams.lane_width;   
    a_lim       = actionParams.a_lim;
    v_lim       = actionParams.v_lim;
    t_sim       = 0:dt:dtAction;
    [x, y, v]   = deal(x0(1), x0(2), x0(3));
    
    theta_sim   = zeros(size(t_sim));
    if actionType == 2 % Maitain lane and keep speed
        state_sim   = [x * ones(size(t_sim)) + v*t_sim; y * ones(size(t_sim)); v * ones(size(t_sim))];
    elseif actionType == 3 % Accelerate
        a           = min(a_lim(2), (v_lim(2) - v)/dtAction);
        state_sim   = [x * ones(size(t_sim)) + v*t_sim + 1/2 * a * t_sim.^2; ...
            y * ones(size(t_sim)); v * ones(size(t_sim)) + a * t_sim];
    elseif actionType == 1 % Decelerate
        a           = max(a_lim(1), (v_lim(1) - v)/dtAction);
        state_sim   = [x * ones(size(t_sim)) + v*t_sim + 1/2 * a * t_sim.^2; ...
            y * ones(size(t_sim)); v * ones(size(t_sim)) + a * t_sim];
    elseif ismember(actionType, 4:9) % Change Lane
        [state_sim, theta_sim]  = LaneChangeTraj(x, y, v, actionType, prevActions,...
            v_lim, a_lim, lane_width, dkLane, dtAction, dt);
    end
end