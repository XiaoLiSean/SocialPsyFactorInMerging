function reward = reward_controlEffort(car_action)
    % -------------------------------------------------------------
    % Vehicle's action:
    % 1: slow down
    % 2: maintain lane and keep speed
    % 3: speed up 
    % 4: change lane left (obj.simParams.dkLane step)
    % 5: change lane right (obj.simParams.dkLane step)
    % Ego Vehicle's action:
    % 6: change lane left while decelerating (obj.simParams.dkLane step)
    % 7: change lane right while decelerating  (obj.simParams.dkLane step)
    % 8: change lane left while accelerating  (obj.simParams.dkLane step)
    % 9: change lane right while accelerating  (obj.simParams.dkLane step)
    % -------------------------------------------------------------
    switch car_action
        case {1, 3, 4, 5}
            reward  = 0.5;
        case {6, 7, 8, 9}
            reward  = 0;
        case 2            
            reward  = 1;          
        otherwise
            error('incorrect car action index');
    end
end