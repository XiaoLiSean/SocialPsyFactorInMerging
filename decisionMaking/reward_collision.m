function reward = reward_collision(state_car, state_opp, dim_car, dim_opp, on_ramp, collisionMargin, LaneParams)
    reward  = 1;   
    if (check_if_collision(state_car, dim_car, state_opp, dim_opp, collisionMargin, on_ramp, LaneParams) == 1)
        reward  = 0;
        return
    end
end