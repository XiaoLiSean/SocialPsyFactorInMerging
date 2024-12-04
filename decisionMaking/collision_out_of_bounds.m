function if_collision = collision_out_of_bounds(states_car, dim_car, on_ramp, LaneParams)

    if_collision    = 0;
    car_length      = dim_car(1);
    car_width       = dim_car(2);
    for i = 1:size(states_car, 2)
        x_car   = states_car(1,i); 
        y_car   = states_car(2,i);

        % out of upper road bounds
        if y_car >= LaneParams.RoadBound(1, 2)
            if_collision    = 1;
            return
        end
        
        % out of lower most road bounds
        if ismember(0, LaneParams.lane_ids)
            if (x_car <= LaneParams.rampMidX) && (y_car <= -LaneParams.lane_width)
                if_collision    = 1;
                return
            end
            if (x_car >= (LaneParams.rampMidX + LaneParams.rampEndX)/2) && (y_car <= 0)
                if_collision    = 1;
                return
            end
        else
            if y_car <= 0
                if_collision    = 1;
                return
            end
        end
        
        % prematuer merging
        if on_ramp && (x_car <= LaneParams.rampStartX + 0.5*car_length) && (y_car >= 0 - 0.5*car_width)
            if_collision    = 1;
            return
        end
    end

end