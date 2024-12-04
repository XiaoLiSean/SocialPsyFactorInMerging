function bestPrevActions = highDInterpPrevActions(state, actual_yVelocity, prev_actions, mpcParams, simParams, LaneParams, actionParams)
    % -------------------------------------------------------------
    % Append previous actions if car_i is changing lane when
    % presenting to the simulation
    % -------------------------------------------------------------
    dkLaneCtrl      = round(mpcParams.HorizonLane * mpcParams.dtHorizon / simParams.dtCtrl);
    CurLaneId       = ceil(state(2) / LaneParams.lane_width);
    MissDistance    = abs(state(2) - LaneParams.lane_width*(CurLaneId - 0.5));
    bestPrevActions = prev_actions;
    if abs(actual_yVelocity) >= LaneParams.highDMinVy &&...
            sum(prev_actions) == 0 &&...
            MissDistance > LaneParams.epsilon_lc % case just starting lane change
        % ---------------------------------------------------------
        changeDir           = 4.5 - 0.5*sign(actual_yVelocity);
        bestMissDistance    = 0.5*LaneParams.lane_width;
        for k_cur = 1:(dkLaneCtrl-1)
            prevActions         = [prev_actions, changeDir*ones(1, k_cur)];
            LaneChangeEndState  = state;
            for addedLaneChangeStep = 1:(dkLaneCtrl-k_cur)
                [LaneChangeSim, ~]  = LaneChangeTraj(LaneChangeEndState(1), LaneChangeEndState(2),...
                    LaneChangeEndState(3), changeDir, [prevActions, changeDir*ones(1, addedLaneChangeStep-1)],...
                    actionParams.v_lim, actionParams.a_lim,...
                    LaneParams.lane_width, dkLaneCtrl,...
                    simParams.dtCtrl, simParams.dt);
                LaneChangeEndState  = LaneChangeSim(:, end);
            end
            CurLaneId           = ceil(LaneChangeEndState(2) / LaneParams.lane_width);
            MissDistance        = abs(LaneChangeEndState(2) - LaneParams.lane_width*(CurLaneId - 0.5));
            if MissDistance < bestMissDistance
                bestMissDistance    = MissDistance;
                bestPrevActions     = prevActions;
            end
        end
    end
end