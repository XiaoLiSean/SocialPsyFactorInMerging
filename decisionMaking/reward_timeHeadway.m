function reward = reward_timeHeadway(state_car, state_opp, dim_car, dim_opp)
    t_max   = 2; % customized maximum time headway
    t_min   = 0.1; % 0.1 minimum human reaction time
    x_car   = state_car(1);
    y_car   = state_car(2);
    v_car   = state_car(3);
    l_car   = dim_car(1);
    w_car   = dim_car(2);
    x_opp   = state_opp(1);
    y_opp   = state_opp(2);
    v_opp   = state_opp(3);
    l_opp   = dim_opp(1);
    w_opp   = dim_opp(2);
    
    if abs(y_car - y_opp) >= (w_car + w_opp)/2 % not interfare at all
        reward  = 1;        
    else % have overlaps in the y axis, possible to collide
        if abs(x_car - x_opp) < (l_car + l_opp)/2
            reward  = 0; % collide, thus zero time headway
        elseif x_car >= x_opp + (l_car + l_opp)/2 % car is not after opp
            reward  = 1;
        elseif (x_car <= x_opp - (l_car + l_opp)/2) && (v_car <= v_opp) % car is after opp, but slower or equal fast
            reward  = 1;
        elseif (x_car <= x_opp - (l_car + l_opp)/2) && (v_car > v_opp) % car is after opp, but faster
            timeHeadway     = (x_opp - x_car - l_car - l_opp)/(v_car - v_opp);
            clamped_time    = min(max(timeHeadway, t_min), t_max);
            normalized_time = (clamped_time-t_min)/(t_max-t_min); % distance to [0,1]
            reward          = normalized_time;
        end
    end
end