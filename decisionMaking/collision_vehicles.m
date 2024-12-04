function if_collision = collision_vehicles(states_car, dim_car, states_opp, dim_opp, collisionMargin)

    if_collision    = 0;
    for i = 1:size(states_car, 2)
        x_car   = states_car(1,i); 
        y_car   = states_car(2,i);
        x_opp   = states_opp(1,i); 
        y_opp   = states_opp(2,i);
        collisionDiffThreshold  = (dim_car(:) + dim_opp(:)).*0.5 + collisionMargin(:);
    
        % Check collision
        if abs(x_car - x_opp) >= collisionDiffThreshold(1) || abs(y_car - y_opp) >= collisionDiffThreshold(2)
            if_collision    = 0;
        else
            if_collision    = 1;
            return
        end
    end

end