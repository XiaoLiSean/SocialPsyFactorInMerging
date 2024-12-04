classdef Highway < matlab.mixin.Copyable
    properties
        Vehicles;
        latent_weights;
        ego_id;
        ego_type;
        
        EgoParams;
        LaneParams;        
        simParams;     
        mpcParams;
        actionParams;
        rewardParams;  
        
        vizHistory;
        actHistory;

        t_current;
    end
    methods        
        % =================================================================
        % Initialization
        % =================================================================
        function obj = Highway(params)            
            obj.EgoParams       = params.EgoParams;
            obj.LaneParams      = params.LaneParams;
            obj.simParams       = params.simParams;
            obj.mpcParams       = params.mpcParams;
            obj.actionParams    = params.actionParams;
            obj.rewardParams    = params.rewardParams;
            
            obj.vizHistory      = [];
            obj.actHistory      = [];
            obj.t_current       = 0;
            
            obj.latent_weights  = params.latent_weights;
            obj.Vehicles        = cell(1, size(params.x0_vehicles, 1));
            obj.ego_id          = 0;
            obj.updateVehicleParams(params.x0_vehicles, params.vehiclesDims)
        end
        
        % =================================================================
        % Main Simulation function 
        % =================================================================
        function simulationSingleStep(obj)
            action_all      = [];  
            states_all      = [];
            trajectory_all  = [];          
            obj.updateVehicleParams();
            % -------------------------------------------------------------
            % -------------------------------------------------------------
            % Update control signal for environment vehicles
            EnvUpdateCtrl   = (mod(obj.t_current, obj.simParams.dtCtrl) < obj.simParams.dt);
            EgoUpdateCtrl   = (mod(obj.t_current, obj.EgoParams.simParams.dtCtrl) < obj.simParams.dt);
            for car_i = 1:length(obj.Vehicles)
                if EgoUpdateCtrl && obj.ego_id ~= 0 && car_i == obj.ego_id                    
                    actionType  = obj.Vehicles{car_i}.ego_policy();
                elseif EnvUpdateCtrl && ~isempty(obj.Vehicles{car_i}.IDM) && isempty(obj.Vehicles{car_i}.SVO)
                    [actionType, acc]                   = obj.IDM_policy(car_i);
                    obj.Vehicles{car_i}.acceleration    = acc;
                elseif EnvUpdateCtrl && ~isempty(obj.Vehicles{car_i}.SVO) && isempty(obj.Vehicles{car_i}.IDM)
                    actionType  = obj.maximum_expectation_policy(car_i);
                elseif ~EnvUpdateCtrl || ~EgoUpdateCtrl
                    actionType  = obj.Vehicles{car_i}.prev_actions(end);
                end     

                action_all  = [action_all; actionType];
            end

            % -------------------------------------------------------------
            % Simulate policy and temporarily store the states
            for car_i = 1:length(obj.Vehicles)
                x0          = obj.Vehicles{car_i}.state;
                actionType  = action_all(car_i);
                customParams.dtAct  = obj.simParams.dtSim;
                
                if isempty(obj.actHistory)
                    customParams.prevActs   = 0;
                else
                    customParams.prevActs   = obj.actHistory(car_i, :);
                end

                if ~isempty(obj.Vehicles{car_i}.IDM) && isempty(obj.Vehicles{car_i}.SVO)
                    customParams.acc    = obj.Vehicles{car_i}.acceleration;
                    [~, state_sim, theta_sim]   = obj.simVehicle(x0, 'IDM', actionType, customParams); 
                elseif ~isempty(obj.Vehicles{car_i}.SVO) && isempty(obj.Vehicles{car_i}.IDM)
                    [~, state_sim, theta_sim]   = obj.simVehicle(x0, 'EM', actionType, customParams);
                elseif obj.ego_id ~= 0 && car_i == obj.ego_id
                    [~, state_sim, theta_sim]   = obj.simVehicle(x0, 'EM', actionType, customParams);
                else
                    error('Vehicle parameter type error');
                end                
                
                states_all          = [states_all, state_sim(:, end)];
                trajectory_all      = [trajectory_all; state_sim; theta_sim];
            end

            % -------------------------------------------------------------
            % Update all state at the same time, to avoid modifying state
            % during policy making since: 
            % -------------------------------------------------------------
            % decision results can only be updated into "highway" and 
            % "vehicle" classes after all the ctrls have updated, otherwise
            % it will affect the decision-making process.
            % meanwhile, the prev_actions inside "vehicles" only updated 
            % when there are new control signals after "dtCtrl"
            % -------------------------------------------------------------
            obj.actHistory      = [obj.actHistory, action_all]; 
            obj.vizHistory      = [obj.vizHistory, trajectory_all];
            for car_i = 1:length(obj.Vehicles)  
                obj.Vehicles{car_i}.state   = states_all(:, car_i);
                if EgoUpdateCtrl && obj.ego_id ~= 0 && car_i == obj.ego_id
                    obj.Vehicles{car_i}.prev_actions    = [obj.Vehicles{car_i}.prev_actions, action_all(car_i)];
                    obj.Vehicles{car_i}.envCarPreActs   = [obj.Vehicles{car_i}.envCarPreActs, action_all];
                    continue;
                end
                if EnvUpdateCtrl
                    obj.Vehicles{car_i}.prev_actions    = [obj.Vehicles{car_i}.prev_actions, action_all(car_i)];
                end
            end

            % -------------------------------------------------------------
            % -------------------------------------------------------------
            obj.t_current   = obj.t_current + obj.simParams.dtSim;
            obj.updateVehicleParams();
        end
        
        % =================================================================
        % Policy using IDM (highway)
        % =================================================================
        function [actionType, acc] = IDM_policy(obj, car_i)
            x_car   = obj.Vehicles{car_i}.state(1);
            v_car   = obj.Vehicles{car_i}.state(3);
            IDM_i   = obj.Vehicles{car_i}.IDM;
            
            [~, leader_i]   = obj.AdjacentVehicleIdxs(car_i);
            
            if ~isempty(leader_i)
                x_leader        = obj.Vehicles{leader_i}.state(1);
                v_leader        = obj.Vehicles{leader_i}.state(3);
                d_gap   = x_leader - x_car;
                dv      = v_car - v_leader;            
                d_des   = IDM_i.d_min + max(0, IDM_i.T*v_car + (v_car*dv)/(2*sqrt(IDM_i.a_max*IDM_i.b)) );
                acc_tmp = IDM_i.a_max*( 1 - (v_car/IDM_i.v_des)^4 -(d_des/d_gap)^2 );
            else                
                acc_tmp = IDM_i.a_max*( 1 - (v_car/IDM_i.v_des)^4 );
            end

            % limits the velocity and the acceleration
            acc_clamped     = min(max(acc_tmp, obj.actionParams.a_lim(1)), obj.actionParams.a_lim(2));
            if acc_tmp > 0 % speed up
                acc = min(acc_clamped, obj.actionParams.v_lim(2) - v_car);
            elseif acc_tmp < 0 % Slow down
                acc = max(acc_clamped, obj.actionParams.v_lim(1) - v_car);
            else % Keep speed
                acc = acc_clamped;
            end
            
            actionType  = 2 + sign(acc) * (abs(acc) >= obj.actionParams.epsilon_acc);
        end
        
        % =================================================================
        % Policy using Maximum Expectation Method
        % =================================================================
        function action = maximum_expectation_policy(obj, car_i)            
            prev_actions    = obj.Vehicles{car_i}.prev_actions;
            actionTypeList  = obj.Vehicles{car_i}.actionTypeLists;
            num_actions     = length(actionTypeList);
            
            car_actions_index   = get_action_sequence(obj.simParams, obj.mpcParams, actionTypeList, prev_actions);
            rewards             = zeros(1, size(car_actions_index, 1));
            
            % Compute expected pair game rewards
            sum_p   = 0;
            for i = 1:size(car_actions_index, 1)
                rewards(i)  = obj.Q_value_i(car_i, car_actions_index(i,:));
                if isinf(exp(rewards(i)))
                    error('Infinite reward')
                end
                sum_p       = sum_p + exp(rewards(i));    
            end

            action_distribution     = zeros(1, length(rewards));
            for i = 1:length(rewards)
                action_distribution(i)  = exp(rewards(i))/sum_p;
            end
            
            averaging_weights   = zeros(1, num_actions);
            for i = 1:num_actions
                averaging_weights(i)    = 1./sum(car_actions_index(:,1) == actionTypeList(i));
            end
            
            action_distribution_one_step    = zeros(1, num_actions);
            for i = 1:length(action_distribution)
                actionTypes     = car_actions_index(i, 1);
                action_idx      = find(actionTypeList == actionTypes);
                action_distribution_one_step(action_idx)    = ...
                    action_distribution_one_step(action_idx) + ...
                    averaging_weights(action_idx)*action_distribution(i);
            end 
            
            % action      = randsample(num_actions, 1, true, action_distribution_one_step);
            [~, action_idx] = max(action_distribution_one_step);
            action          = actionTypeList(action_idx);
        end
        
        function Q_value = Q_value_i(obj, car_i, actions_index_i)     
            horizon             = length(actions_index_i);
            [adjVehicles, ~]    = obj.AdjacentVehicleIdxs(car_i);
            Q_value             = 0;
            CarPrevActDt.dtAct  = obj.simParams.dtCtrl;
            OppPrevActDt.dtAct  = obj.simParams.dtCtrl;
            % -----------------------------------------
            % Case there is no surrounding vehicle
            % -----------------------------------------
            if isempty(adjVehicles)
                opp_x       = [inf; inf; 0]; % create an imaginary opponent
                opp_dim     = [0;0]; % create an imaginary opponent
                car_x_tmp               = obj.Vehicles{car_i}.state;
                car_prev_actions_tmp    = obj.Vehicles{car_i}.prev_actions;
                for k = 1:horizon
                    car_action              = actions_index_i(k);
                    CarPrevActDt.prevActs   = car_prev_actions_tmp;
                    [~, car_x_all_tmp, ~]   = obj.simVehicle(car_x_tmp, 'EM', car_action, CarPrevActDt);  
                    car_x_tmp               = car_x_all_tmp(:,end);
                    car_prev_actions_tmp    = [car_prev_actions_tmp, car_action];
                    
                    single_r    = obj.personalized_reward(car_i, car_x_tmp, car_action, opp_x, opp_dim, obj.Vehicles{car_i}.SVO.weights);
                    Q_value     = Q_value + obj.rewardParams.gamma^(k - 1) * single_r;
                end
                return
            end            
            % -----------------------------------------
            % Case there are interacting vehicles
            % -----------------------------------------
            car_state   = obj.Vehicles{car_i}.state;
            car_dim     = [obj.Vehicles{car_i}.vecLength, obj.Vehicles{car_i}.vecWidth]; 
            theta_1     = obj.Vehicles{car_i}.SVO.theta_1;
            theta_2     = obj.Vehicles{car_i}.SVO.theta_2;
            for opp_i = adjVehicles          
                opp_actions_index   = get_action_sequence(obj.simParams, obj.mpcParams,...
                    obj.Vehicles{opp_i}.actionTypeLists, obj.Vehicles{opp_i}.prev_actions);
                opp_state   = obj.Vehicles{opp_i}.state;
                opp_dim     = [obj.Vehicles{opp_i}.vecLength, obj.Vehicles{opp_i}.vecWidth]; 
                % Compute rewards
                r_over_opp_vehicle  = zeros(1, size(opp_actions_index,1));
                for j = 1:size(opp_actions_index,1)
                    car_x_tmp   = car_state;
                    opp_x_tmp   = opp_state;
                    car_prev_actions_tmp    = obj.Vehicles{car_i}.prev_actions;
                    opp_prev_actions_tmp    = obj.Vehicles{opp_i}.prev_actions;
                    for k = 1:horizon
                        car_action  = actions_index_i(k);
                        opp_action  = opp_actions_index(j,k);
                        CarPrevActDt.prevActs   = car_prev_actions_tmp;
                        OppPrevActDt.prevActs   = opp_prev_actions_tmp;
                        
                        [~, car_x_all_tmp, ~]   = obj.simVehicle(car_x_tmp, 'EM', car_action, CarPrevActDt);                        
                        [~, opp_x_all_tmp, ~]   = obj.simVehicle(opp_x_tmp, 'EM', opp_action, OppPrevActDt);
                        car_x_tmp   = car_x_all_tmp(:,end);
                        opp_x_tmp   = opp_x_all_tmp(:,end);
                        car_prev_actions_tmp        = [car_prev_actions_tmp, car_action];
                        opp_prev_actions_tmp        = [opp_prev_actions_tmp, opp_action];
                        
                        if obj.rewardParams.enforceCollision && ...
                                check_if_collision(car_x_all_tmp, car_dim, opp_x_all_tmp, opp_dim,...
                                obj.rewardParams.collisionMargin, obj.Vehicles{car_i}.on_ramp, obj.LaneParams)
                            r_over_opp_vehicle(j)   = 0;
                            break;
                        else
                            single_r    = theta_1*obj.reward(car_i, car_x_tmp, car_action, opp_i, opp_x_tmp, false) +...
                                theta_2*obj.reward(opp_i, opp_x_tmp, opp_action, car_i, car_x_tmp, obj.latent_weights);
                            r_over_opp_vehicle(j)   = r_over_opp_vehicle(j) + obj.rewardParams.gamma^(k - 1) * single_r;
                        end

                    end
                end
                % averaged reward over all adjacent vehicle and its
                % possible action sequences
                Q_value = Q_value + mean(r_over_opp_vehicle) / length(adjVehicles); 
            end
        end

        % =================================================================
        % Reward functions
        % =================================================================
        function r_i = reward(obj, car_i, car_x, car_action, opp_i, opp_x, unknow_weight) 
            if unknow_weight % assume the weights of other vehicles are unknown, thus (1,1,1,1)/4
                weight_num  = 3 + (~obj.rewardParams.enforceCollision);
                weights     = ones(1, weight_num)./weight_num;
            else % assume the weights of other vehicles are known
                weights = obj.Vehicles{car_i}.SVO.weights;
            end
            dim_opp = [obj.Vehicles{opp_i}.vecLength, obj.Vehicles{opp_i}.vecWidth]; 
            r_i     = obj.personalized_reward(car_i, car_x, car_action, opp_x, dim_opp, weights);
        end

        function reward = personalized_reward(obj, car_i, state_car, car_action, state_opp, dim_opp, weights)
            
            dim_car = [obj.Vehicles{car_i}.vecLength, obj.Vehicles{car_i}.vecWidth]; 
            on_ramp = obj.Vehicles{car_i}.on_ramp;

            if (0.5*(dim_car(2) + dim_opp(2)) + obj.rewardParams.collisionMargin(2)) > obj.LaneParams.lane_width
                error("Width of collision cushion is too large");
            end
            
            r1  = reward_timeHeadway(state_car, state_opp, dim_car, dim_opp);
            r2  = reward_travelingTime(state_car, on_ramp, obj.rewardParams.RampGoal, obj.LaneParams);
            r3  = reward_controlEffort(car_action);
            r4  = reward_collision(state_car, state_opp, dim_car, dim_opp, on_ramp, obj.rewardParams.collisionMargin, obj.LaneParams);
            if obj.rewardParams.enforceCollision && length(weights) == 3
                reward  = obj.rewardParams.maxReward*dot(weights, [r1; r2; r3]);
            elseif ~obj.rewardParams.enforceCollision && length(weights) == 4            
                reward  = obj.rewardParams.maxReward*dot(weights, [r1; r2; r3; r4]);
            else
                error('Weight dimension not match with rewardParams.enforceCollision');
            end
        end

        % =================================================================
        % Simulate the policy
        % =================================================================
        function [t_sim, state_sim, theta_sim] = simVehicle(obj, x0, ModelType, actionType, customParams)
            if strcmp(ModelType, 'EM') == 1 % using EM policy
                prevActsAndDt       = customParams;
                [t_sim, state_sim, theta_sim]   = VehicleSimulator(x0, actionType, prevActsAndDt,...
                    obj.actionParams, obj.simParams, obj.mpcParams, obj.LaneParams);
            elseif strcmp(ModelType, 'IDM') == 1 % using IDM policy
                accAndDt    = customParams;
                [t_sim, state_sim]      = IDMSimulator(x0, accAndDt.acc, accAndDt.dtAct, obj.simParams);
                theta_sim   = zeros(1, length(t_sim));
            else
                error('Incorrect input arguement numbers');                
            end
        end   
 
        % =================================================================
        % Update vehicle parameters as the position changes
        % -------------------------------------------------------------
        % Vehicle's action:
        % 1: slow down
        % 2: maintain lane and keep speed
        % 3: speed up 
        % 4: change lane left 
        % 5: change lane right 
        % Ego Vehicle's action:
        % 6: change lane left while decelerating 
        % 7: change lane right while decelerating  
        % 8: change lane left while accelerating  
        % 9: change lane right while accelerating  
        % -------------------------------------------------------------
        % =================================================================
        function updateVehicleParams(obj, x0_vehicles, vehiclesDims)
            is_initialization   = (nargin == 3);
            for i = 1:length(obj.Vehicles)
                if is_initialization
                    state           = x0_vehicles{i,1};     
                else
                    state       = obj.Vehicles{i}.state;
                end
                y_car_i         = state(2);
                lane_id         = ceil(y_car_i / obj.LaneParams.lane_width);
                on_ramp         = (lane_id == 0);           
                if ~ismember(lane_id, obj.LaneParams.lane_ids) && state(1) ~= -inf 
                    % -inf is used to case temporarily unpresents in HighD
                    error('Y Coordinates is not in all lanes range')
                end 
                % ---------------------------------------------------------
                % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                % Note: this pre-defined actionTypeLists will limit the
                % action seq. within N prediction horizon. cuz change lane
                % then change back is a totally legit seq. for [1 2 3 4],
                % though this types of action is not very useful ...
                % use action such as overtake takes more steps which is 
                % complecated to consider here 
                % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                % ---------------------------------------------------------
                % Case at the right most lane, can only turn to left lane
                actionTypeLists = [];
                if lane_id == obj.LaneParams.lane_ids(1)
                    if length(obj.LaneParams.lane_ids) >= 2
                        actionTypeLists = [1, 2, 3, 4];
                    else
                        actionTypeLists = [1, 2, 3];
                    end
                end
                    
                % Case at the left most lane, can only turn right if there
                % is non-closed-ramp or highway lane
                if lane_id == obj.LaneParams.lane_ids(end) 
                    if length(obj.LaneParams.lane_ids) >= 2
                        if (obj.LaneParams.lane_ids(end-1) == 0 && ~obj.LaneParams.rampClosed) ||...
                            obj.LaneParams.lane_ids(end-1) ~= 0
                            actionTypeLists = [1, 2, 3, 5];
                        else                            
                            actionTypeLists = [1, 2, 3];
                        end
                    else                        
                        actionTypeLists = [1, 2, 3];
                    end
                end
                    
                % Case in the middle lanes
                if lane_id > obj.LaneParams.lane_ids(1) && lane_id < obj.LaneParams.lane_ids(end)
                    if obj.LaneParams.rampClosed && obj.LaneParams.lane_ids(1) == 0 && ...
                            lane_id == 1 % case closed end ramp
                        actionTypeLists = [1, 2, 3, 4];
                    else
                        actionTypeLists = [1, 2, 3, 4, 5];
                    end
                end   

                % this means we do have lane change previously and it's not
                % in the actionTypeLists. e.g., [4 4 5] and vehicle in the
                % middle of two lanes
                if ~is_initialization && ~isempty(obj.actHistory) ...
                    && ismember(obj.actHistory(i, end), [4,5]) ...
                    && ~ismember(obj.actHistory(i, end), actionTypeLists)

                    laneChangeDirection     = obj.actHistory(i, end);               
                    consecutiveLaneChanges  = 0;
                    for k = length(obj.actHistory(i, :)):-1:1
                        if obj.actHistory(i, k) == laneChangeDirection
                            consecutiveLaneChanges  = consecutiveLaneChanges + 1;
                        else
                            break;
                        end
                    end                    
                    % this means the lane change is not ended  
                    dkLane  = floor(obj.mpcParams.dtHorizon / obj.simParams.dtSim) * obj.mpcParams.HorizonLane; % numbers of sim steps needed for a lane change
                    if mod(consecutiveLaneChanges, dkLane) ~= 0
                        actionTypeLists = [actionTypeLists, laneChangeDirection];                        
                    end
                end

                % ---------------------------------------------------------
                % Update info about environment vehicle
                % ---------------------------------------------------------                
                if is_initialization
                    paramType       = x0_vehicles{i,2};
                    params          = x0_vehicles{i,3};
                    vecDim          = vehiclesDims(:,i);
                    if strcmp(paramType, 'EGO') 
                        obj.ego_id      = i;
                        obj.ego_type    = paramType;
                        ego_args        = {state, vecDim, on_ramp, lane_id, actionTypeLists, paramType, params};
                    end
                    obj.Vehicles{i} = Vehicle(state, vecDim, on_ramp, lane_id, actionTypeLists, paramType, params);
                else
                    obj.Vehicles{i}.on_ramp         = on_ramp;
                    obj.Vehicles{i}.lane_id         = lane_id;
                    obj.Vehicles{i}.actionTypeLists = actionTypeLists;
                end
                % ---------------------------------------------------------
                % ---------------------------------------------------------
            end
            % -------------------------------------------------------------
            % Update Ego Vehicle Info
            % -------------------------------------------------------------
            if obj.ego_id ~= 0
                if is_initialization && obj.ego_id ~= 0
                    if strcmp(obj.ego_type, 'EGO') 
                        obj.Vehicles{obj.ego_id}    = VehicleEgoNormal(obj.ego_id, ego_args, obj);
                    else
                        error('Wrong ego type')
                    end
                else
                    obj.Vehicles{obj.ego_id}.updateHighwayInfo(obj)
                end
            end
            % ------------------------------------------------------------- 
        end
        
        % =================================================================
        % Obtain the adjacent vehicle
        % =================================================================
        function [idxs, leader_cur] = AdjacentVehicleIdxs(obj, car_i)
            % determine the maximum interaction distance
            distance_max    = obj.mpcParams.nHorizon*obj.mpcParams.dtHorizon*...
                (obj.actionParams.v_lim(2) - obj.actionParams.v_lim(1));

            idxCell = cell(3,3);
            x_car   = obj.Vehicles{car_i}.state(1);
            y_car   = obj.Vehicles{car_i}.state(2); 
            l_car   = obj.Vehicles{car_i}.vecLength;
            w_car   = obj.Vehicles{car_i}.vecWidth;
            for opp_i = 1:length(obj.Vehicles)
                if opp_i == car_i
                    continue
                end

                % case vehicles are not adjacent
                if abs(obj.Vehicles{car_i}.lane_id - obj.Vehicles{opp_i}.lane_id) > 1
                    continue;
                end
                
                x_opp   = obj.Vehicles{opp_i}.state(1);
                y_opp   = obj.Vehicles{opp_i}.state(2); 
                l_opp   = obj.Vehicles{opp_i}.vecLength;
                w_opp   = obj.Vehicles{opp_i}.vecWidth;

                % case longitudinal distance is too far
                if abs(x_opp - x_car) >= distance_max
                    continue
                end
                
                % too far to be considered as environment vehicle
                if x_opp < obj.LaneParams.leftBound || x_opp > obj.LaneParams.rightBound
                    continue
                end
                
                % CORA toolbox is used here
                left_intersects     = (x_car-l_car/2 >= x_opp-l_opp/2) && (x_car-l_car/2 <= x_opp+l_opp/2);
                right_intersects    = (x_car+l_car/2 >= x_opp-l_opp/2) && (x_car+l_car/2 <= x_opp+l_opp/2);
                is_intersecting_x   = (left_intersects) || (right_intersects);

                bottom_intersects   = (y_car-w_car/2 >= y_opp-w_opp/2) && (y_car-w_car/2 <= y_opp+w_opp/2);
                top_intersects      = (y_car+w_car/2 >= y_opp-w_opp/2) && (y_car+w_car/2 <= y_opp+w_opp/2);
                is_intersecting_y   = (bottom_intersects) || (top_intersects);

%                 is_intersecting_x   = isIntersecting(interval(x_opp-l_opp/2, x_opp+l_opp/2),...
%                     interval(x_car-l_car/2, x_car+l_car/2));
%                 is_intersecting_y   = isIntersecting(interval(y_opp-w_opp/2, y_opp+w_opp/2),...
%                     interval(y_car-w_car/2, y_car+w_car/2)); 
                
                i_idx   = 2 + sign(y_car-y_opp) * (~is_intersecting_y);
                j_idx   = 2 + sign(x_opp-x_car) * (~is_intersecting_x);
                
                if isempty(idxCell{i_idx, j_idx}) % if empty                    
                    idxCell{i_idx, j_idx}   = opp_i;
                else % not empty 
                    if ismember(j_idx, [1,3]) % is far away from the car_i
                        x_occupied  = obj.Vehicles{idxCell{i_idx, j_idx}}.state(1);
                        if abs(x_opp-x_car) < abs(x_occupied-x_car)
                            idxCell{i_idx, j_idx}   = opp_i;
                        end
                    else % is near the car_i
                        idxCell{i_idx, j_idx}   = [idxCell{i_idx, j_idx}, opp_i];
                    end
                end
            end            
            leader_cur      = idxCell{2,end};
            follower_cur    = idxCell{2,1};
            idxs            = [idxCell{:}];
            % leader_new      = idxArr([1,3],end);
            % follower_new    = idxArr([1,3],1);
        end
    end
end