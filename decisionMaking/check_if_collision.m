function if_collision = check_if_collision(states_car, dim_car, states_opp, dim_opp, collisionMargin, on_ramp, LaneParams)

    vec_collision   = collision_vehicles(states_car, dim_car, states_opp, dim_opp, collisionMargin);
    out_of_bounds   = collision_out_of_bounds(states_car, dim_car, on_ramp, LaneParams);
    if_collision    = (vec_collision || out_of_bounds);
    
end