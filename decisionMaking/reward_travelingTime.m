function reward = reward_travelingTime(state_car, on_ramp, RampGoal, LaneParams)
    lane_width  = LaneParams.lane_width;
    lane_length = LaneParams.lane_length;
    x_car       = state_car(1); 
    y_car       = state_car(2);
    
    % Rewards for ramp vehicle
    if on_ramp                   
        distance_x  = abs(min(x_car-RampGoal(1), 0));
        rx          = (lane_length - distance_x)/lane_length;
        ry          = 2 - exp(log(2)/lane_width*abs(y_car-RampGoal(2)));
        weights     = [0; 1]; % emphasize more on lane change
        reward      = dot(weights./sum(weights), [rx; ry]);
    else
        % reward highway vehicle to reach the goal faster, note since only
        % the ego will change lane, thus collision is in ego vehicle's
        % responsibility and no penalty term for highway vehicle
        distance_x  = abs(min(x_car-RampGoal(1), 0));
        rx          = (lane_length - distance_x)/lane_length;
        reward      = rx;
    end
end