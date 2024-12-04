classdef VehicleEgoNormal < Vehicle   
    properties               
        car_id;
        laneChangeAug; % augment lane change with acc and dec for ego
        laneChangeAbort; % allow lane change abortion for ego
        prediction_threshold; % (negative) penalize collision in one pair game so that the penalty will affect the decision over other pairs
        
        simParams; % separate updating time from the highway vehicles
        envCarPreActs; % previous action of environment vehicles at a ego updating freq.

        filter;
        filterUpdated;
        filterHistory;

        highway;
        highway_info_updated;
        TimeElapsed;
    end
    methods
        function obj = VehicleEgoNormal(car_id, ego_args, highway)
            obj             = obj@Vehicle(ego_args{:});
            obj.car_id      = car_id;

            obj.laneChangeAug           = highway.EgoParams.laneChangeAug; 
            obj.laneChangeAbort         = highway.EgoParams.laneChangeAbort;
            obj.prediction_threshold    = highway.EgoParams.prediction_threshold;

            obj.simParams               = highway.EgoParams.simParams;
            obj.envCarPreActs           = [];

            weight_num  = 3 + (~highway.rewardParams.enforceCollision);
            Importance  = permn(0:1, weight_num);
            Weights     = Importance./repmat(sum(Importance,2), 1, weight_num);
            W_set       = unique(Weights, 'rows');
            W_set(any(isnan(W_set), 2), :)  = [];
            obj.filter.W_set    = W_set;
            obj.filter.S_set    = [90, 45, 0, -45];             
            obj.filter.likelihoods  = cell(length(highway.Vehicles), 1);
            obj.filterUpdated       = [];
            likelihoods             = ones(length(obj.filter.S_set), size(obj.filter.W_set, 1));
            likelihoods(1, 2:end)   = 0; % altruistic don't care self reward
            obj.filter.likelihoods(:)   = {likelihoods./sum(likelihoods, 'all')};

            obj.TimeElapsed.Inference   = [];
            obj.TimeElapsed.Prediction  = [];
            obj.TimeElapsed.Control     = [];

            obj.updateHighwayInfo(highway);
        end

        function updateHighwayInfo(obj, highway)
            obj.highway             = copy(highway); 
            obj.highway.simParams   = obj.simParams;
            
            % -------------------------------------------------------------
            % The prev_actions shall have an time interval of
            % simParams.dtCtrl, here we update the Ego stored vehicles with
            % prev_actions that follows the ego's dtCtrl variable for
            % future use of prediction and control game
            % -------------------------------------------------------------
            for car_i = 1:length(obj.highway.Vehicles)
                if car_i == obj.car_id
                    EgoCopy.vecLength   = obj.vecLength;
                    EgoCopy.vecWidth    = obj.vecWidth;
                    EgoCopy.state       = obj.state;
                    EgoCopy.on_ramp     = obj.on_ramp;
                    EgoCopy.lane_id     = obj.lane_id;
                    EgoCopy.actionTypeLists = obj.actionTypeLists;
                    EgoCopy.prev_actions    = obj.prev_actions;
                    obj.highway.Vehicles{car_i}     = EgoCopy;
                    continue;
                end
                obj.highway.Vehicles{car_i} = copy(highway.Vehicles{car_i});
            end

            if ~isempty(obj.envCarPreActs)
                for car_i = 1:length(obj.highway.Vehicles)
                    obj.highway.Vehicles{car_i}.prev_actions    = obj.envCarPreActs(car_i, :);
                end
            end
            obj.recursiveBayesianFilter();
            obj.highway_info_updated    = true;
        end
        % =================================================================
        % Update SVO and Weight belief
        % =================================================================
        function recursiveBayesianFilter(obj)
            dkSim       = (obj.simParams.dtSim / obj.simParams.dt) + 1;
            dkBayes     = (obj.simParams.dtBayes / obj.simParams.dtSim)*dkSim;
            if mod(size(obj.highway.vizHistory, 2), dkBayes) ~= 0
                return;
            end
            stepBayes   = (size(obj.highway.vizHistory, 2) / dkBayes);
            adjVehicles = obj.AdjacentVehicleIdxs();

            % Conditioned on repeated updates
            if size(obj.filterHistory, 2) ~= stepBayes 
                return;
            end
            
            % Conditioned on initialization
            if stepBayes == 0
                obj.filterHistory   = cat(2, obj.filterHistory, obj.filter.likelihoods);
                obj.filterUpdated   = ones(length(obj.highway.Vehicles), 1);
                return
            end
  
            T0_Inference    = tic;
            isCarIUpdated   = zeros(length(obj.highway.Vehicles), 1);
            for car_i = 1:length(obj.highway.Vehicles)
                if car_i == obj.car_id || ~ismember(car_i, adjVehicles)
                    continue;
                end
                
                % Condition unpresented vehicle
                if isequal(obj.highway.Vehicles{car_i}.state, [-inf; 0; 0]) 
                    continue
                end

                % ---------------------------------------------------------
                % Recursive filtering
                % ---------------------------------------------------------
                % gather un-updated trajectory to do inference (multiple steps)
                for step_i = 1:(stepBayes-1)
                    if obj.filterUpdated(car_i, step_i+1) == 0
                        [likelihoods, ~]    = obj.SearchOptimalParameters_Car_i(car_i, step_i);
                        obj.filter.likelihoods{car_i}   = obj.filter.likelihoods{car_i}.*likelihoods;
                        if sum(obj.filter.likelihoods{car_i}, 'all') == 0
                            error('Zero likelihood error');
                        end
                        obj.filter.likelihoods{car_i}       = obj.filter.likelihoods{car_i}./sum(obj.filter.likelihoods{car_i}, 'all');
                        obj.filterHistory{car_i, step_i+1}  = obj.filter.likelihoods{car_i};
                        obj.filterUpdated(car_i, step_i+1)  = 1;
                    else
                        break;
                    end
                end

                % previously updated, this time only update (one step)
                [likelihoods, ~]    = obj.SearchOptimalParameters_Car_i(car_i, stepBayes);
                obj.filter.likelihoods{car_i}   = obj.filter.likelihoods{car_i}.*likelihoods;
                if sum(obj.filter.likelihoods{car_i}, 'all') == 0
                    error('Zero likelihood error');
                end
                obj.filter.likelihoods{car_i}   = obj.filter.likelihoods{car_i}./sum(obj.filter.likelihoods{car_i}, 'all');
                isCarIUpdated(car_i)            = 1;
                % ---------------------------------------------------------
            end
            obj.TimeElapsed.Inference       = [obj.TimeElapsed.Inference, toc(T0_Inference)];
            obj.filterHistory   = cat(2, obj.filterHistory, obj.filter.likelihoods);
            obj.filterUpdated   = [obj.filterUpdated, isCarIUpdated];
        end

        function [predictionScores, trajectories] = SearchOptimalParameters_Car_i(obj, car_i, stepBayes) 
            num_veh     = length(obj.highway.Vehicles);
            num_states  = size(obj.highway.vizHistory, 1)/num_veh;
            S_set       = obj.filter.S_set;
            W_set       = obj.filter.W_set;
            dkSim       = (obj.simParams.dtSim / obj.simParams.dt) + 1;
            predictionScores    = zeros(length(S_set), size(W_set, 1));
            trajectories        = cell(length(S_set), size(W_set, 1));
            
            if mod(size(obj.highway.vizHistory, 2), dkSim) == 0 
                % case simulation env where the current initial state is 
                % previous final state and both two are in history (repeated)
                dkBayes     = (obj.simParams.dtBayes / obj.simParams.dtSim)*dkSim;
                startFrame  = (stepBayes-1)*dkBayes + 1;
                endFrame    = stepBayes*dkBayes;
            else
                % Case highD testing,  no repeated consecutive final and
                % intial states
                dkBayes     = (obj.simParams.dtBayes / obj.simParams.dt);
                startFrame  = (stepBayes-1)*dkBayes + 1;
                endFrame    = stepBayes*dkBayes + 1;
            end
            highwayCtrlIdx  = (stepBayes - 1)*(obj.simParams.dtBayes / obj.simParams.dtSim);
            vehicleCtrlIdx  = (stepBayes - 1)*(obj.simParams.dtBayes / obj.simParams.dtCtrl);

            vizHistory          = obj.highway.vizHistory;
            actHistory          = obj.highway.actHistory;

            params.EgoParams        = obj.highway.EgoParams;
            params.LaneParams       = obj.highway.LaneParams;
            params.simParams        = obj.highway.simParams;
            params.mpcParams        = obj.highway.mpcParams;
            params.actionParams     = obj.highway.actionParams;
            params.rewardParams     = obj.highway.rewardParams;
            params.latent_weights   = obj.highway.latent_weights;
            vehiclesDims            = zeros(2, num_veh);
            for i = 1:num_veh
                vehiclesDims(:,i)   = [obj.highway.Vehicles{i}.vecLength; obj.highway.Vehicles{i}.vecWidth];
            end
            params.vehiclesDims     = vehiclesDims;

            for s_i = 1:length(S_set) 
                % =================================================================
                % Multi-threading
                % =================================================================
                svo         = S_set(s_i);               
                traj_tmp    = cell(1, size(W_set, 1));
                score_tmp   = predictionScores(s_i, :);
                if svo == 90
                    nontrivial_w_num    = 1; % altruistic don't care self reward
                else
                    nontrivial_w_num    = size(W_set, 1);
                end
                
                parfor w_i = 1:nontrivial_w_num   
                    % -------------------------------------------------------------          
                    % Initialize highway
                    % -------------------------------------------------------------
                    weight      = W_set(w_i, :); 
                    SVOs        = svo.*ones(num_veh, 1);
                    Weights     = repmat(weight, num_veh, 1);
                    
                    X0          = vizHistory(:, startFrame);
                    if num_states == 4 % remove the theta
                        X0(4:4:end) = [];
                    end
                    Input       = cell(num_veh, 3);
                    for ii = 1:num_veh
                        Input(ii,:)     = {X0((ii*3-2):(ii*3), 1), 'SVO', [SVOs(ii), Weights(ii,:)]};
                    end
                    
                    initialParams               = params;
                    initialParams.x0_vehicles   = Input;
                    highwaySim                  = Highway(initialParams);
                    highwaySim.vizHistory       = vizHistory(:, 1:(startFrame-1));
                    highwaySim.actHistory       = actHistory(:, 1:highwayCtrlIdx);                    
                    for ii = 1:num_veh
                        highwaySim.Vehicles{ii}.prev_actions    = obj.highway.Vehicles{ii}.prev_actions(1:vehicleCtrlIdx);
                    end
                    
                    % -------------------------------------------------------------
                    % Simulate each individual time step
                    % -------------------------------------------------------------

                    actionType      = highwaySim.maximum_expectation_policy(car_i);
                    CarPrevActDt    = [];
                    CarPrevActDt.dtAct      = obj.simParams.dtCtrl;
                    CarPrevActDt.prevActs   = highwaySim.Vehicles{car_i}.prev_actions;
                    [~, state_sim, ~]       = VehicleSimulator(highwaySim.Vehicles{car_i}.state, actionType, CarPrevActDt,...
                        obj.highway.actionParams, obj.simParams, obj.highway.mpcParams, obj.highway.LaneParams);
                    traj_tmp{w_i}           = state_sim;

                    % -------------------------------------------------------------                
                    % Calculate scores of trajectory prediction
                    % -------------------------------------------------------------
                    actual_traj_car_i   = vizHistory(((car_i-1)*num_states+1):((car_i-1)*num_states+3), startFrame:endFrame);
                    score_tmp(w_i)      = similarityMetric(actual_traj_car_i, state_sim, initialParams);

                end
                trajectories(s_i, :)        = traj_tmp;
                predictionScores(s_i, :)    = score_tmp;  
            end     
        end

        % =================================================================
        % Policy using Maximum Expectation Method            
        % -----------------------------------------------------------------
        % Ego vehicle's action:
        % 1: slow down
        % 2: maintain lane and keep speed
        % 3: speed up 
        % 4: change lane left (obj.simParams.dkLane step)
        % 5: change lane right (obj.simParams.dkLane step)
        % 6: change lane left while decelerating (obj.simParams.dkLane step)
        % 7: change lane right while decelerating  (obj.simParams.dkLane step)
        % 8: change lane left while accelerating  (obj.simParams.dkLane step)
        % 9: change lane right while accelerating  (obj.simParams.dkLane step)
        % -----------------------------------------------------------------
        function action = ego_policy(obj)   
            if ~obj.highway_info_updated
                error('Highway infomation not updated in Ego.');
            end
            prev_actions    = obj.highway.Vehicles{obj.car_id}.prev_actions;
            actionTypeList  = obj.highway.Vehicles{obj.car_id}.actionTypeLists;
            
            if obj.laneChangeAug
                if ismember(4, actionTypeList)
                    actionTypeList  = [actionTypeList, 6, 8];
                end
                if ismember(5, actionTypeList)
                    actionTypeList  = [actionTypeList, 7, 9];
                end
            end

            car_actions_index   = get_action_sequence(obj.simParams, obj.highway.mpcParams, actionTypeList, prev_actions,...
                                    obj.laneChangeAug, obj.laneChangeAbort);
            car_actions_index   = obj.rmEgoOutofBoundsSeq(car_actions_index);
            rewards             = zeros(1, size(car_actions_index, 1));            
            actionTypeList      = unique(car_actions_index(:,1));
            num_actions         = length(actionTypeList);
            % -------------------------------------------------------------
            % Predict Adjacent Vehicles            
            % -------------------------------------------------------------
            T0_Prediction       = tic;
            adjVehicles         = obj.AdjacentVehicleIdxs();
            predictionResults   = [];
            if ~isempty(adjVehicles)
                actValueSet     = cell(1, length(adjVehicles));
                probValueSet    = cell(1, length(adjVehicles));
                for ith_opp = 1:length(adjVehicles)
                    opp_i   = adjVehicles(ith_opp);
                    [opp_actions_index, likelihoods]    = obj.highwayPrediction(opp_i);
                    actValueSet{ith_opp}    = opp_actions_index;
                    probValueSet{ith_opp}   = likelihoods;
                end
                if length(adjVehicles) == 1
                    predictionResults.opp_actions_index     = containers.Map(adjVehicles, actValueSet{:});
                    predictionResults.likelihoods           = containers.Map(adjVehicles, probValueSet{:});
                else
                    predictionResults.opp_actions_index     = containers.Map(adjVehicles, actValueSet);
                    predictionResults.likelihoods           = containers.Map(adjVehicles, probValueSet);
                end
            end
            obj.TimeElapsed.Prediction  = [obj.TimeElapsed.Prediction, toc(T0_Prediction)];

            % -------------------------------------------------------------
            % Decision Making: Compute expected pair game rewards
            % -------------------------------------------------------------
            T0_Control  = tic;
            sum_p   = 0;
            for i = 1:size(car_actions_index, 1)
                rewards(i)  = obj.Q_value(adjVehicles, predictionResults, car_actions_index(i,:));
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
            
            [~, action_idx] = max(action_distribution_one_step);
            action          = actionTypeList(action_idx);
            obj.highway_info_updated    = false;
            obj.TimeElapsed.Control     = [obj.TimeElapsed.Control, toc(T0_Control)];
        end
        
        % =================================================================
        % Q-value function
        % =================================================================      
        function Q_value = Q_value(obj, adjVehicles, predictionResults, actions_index)     
            if ~obj.highway_info_updated
                error('Highway infomation not updated in Ego.');
            end   
            horizon     = length(actions_index);
            Q_value     = 0;
            CarPrevActDt.dtAct  = obj.simParams.dtCtrl;
            OppPrevActDt.dtAct  = obj.simParams.dtCtrl;
            % -----------------------------------------
            % Case there is no surrounding vehicle
            % -----------------------------------------
            if isempty(adjVehicles)
                opp_x       = [inf; inf; 0]; % create an imaginary opponent
                opp_dim     = [0;0]; % create an imaginary opponent
                car_dim     = [obj.vecLength, obj.vecWidth]; 
                car_x_tmp               = obj.highway.Vehicles{obj.car_id}.state;
                car_prev_actions_tmp    = obj.highway.Vehicles{obj.car_id}.prev_actions;
                for k = 1:horizon
                    car_action              = actions_index(k);
                    CarPrevActDt.prevActs   = car_prev_actions_tmp;
                    [~, car_x_all_tmp, ~]   = VehicleSimulator(car_x_tmp, car_action, CarPrevActDt,...
                        obj.highway.actionParams, obj.simParams, obj.highway.mpcParams, obj.highway.LaneParams);
                    car_x_tmp               = car_x_all_tmp(:,end);
                    car_prev_actions_tmp    = [car_prev_actions_tmp, car_action];
                    
                    single_r    = obj.reward(car_x_tmp, car_dim, car_action, opp_x, opp_dim);                    
                    Q_value     = Q_value + obj.highway.rewardParams.gamma^(k - 1) * single_r;
                end
                return
            end            
            % -----------------------------------------
            % Case there are interacting vehicles
            % -----------------------------------------
            car_state   = obj.state;
            car_dim     = [obj.vecLength, obj.vecWidth]; 
            Q_over_opp_vehicles  = zeros(length(adjVehicles), 1);
            for ith_opp = 1:length(adjVehicles)
                opp_i               = adjVehicles(ith_opp);
                opp_actions_index   = predictionResults.opp_actions_index(opp_i);
                likelihoods         = predictionResults.likelihoods(opp_i);
                opp_state   = obj.highway.Vehicles{opp_i}.state;
                opp_dim     = [obj.highway.Vehicles{opp_i}.vecLength, obj.highway.Vehicles{opp_i}.vecWidth];
                % Compute rewards
                r_over_opp_vehicle  = zeros(1, size(opp_actions_index,1));
                
                parfor j = 1:size(opp_actions_index,1)
                    car_x_tmp   = car_state;
                    opp_x_tmp   = opp_state;
                    CarPrevActDt_loc        = CarPrevActDt; % local variable for parfor
                    OppPrevActDt_loc        = OppPrevActDt; % local variable for parfor
                    car_prev_actions_tmp    = obj.prev_actions;
                    opp_prev_actions_tmp    = obj.highway.Vehicles{opp_i}.prev_actions;
                    for k = 1:horizon
                        car_action  = actions_index(k);
                        opp_action  = opp_actions_index(j,k);   

                        CarPrevActDt_loc.prevActs   = car_prev_actions_tmp;
                        OppPrevActDt_loc.prevActs   = opp_prev_actions_tmp;

                        [~, car_x_all_tmp, ~]   = VehicleSimulator(car_x_tmp, car_action, CarPrevActDt_loc,...
                            obj.highway.actionParams, obj.simParams, obj.highway.mpcParams, obj.highway.LaneParams);                    
                        [~, opp_x_all_tmp, ~]   = VehicleSimulator(opp_x_tmp, opp_action, OppPrevActDt_loc,...
                            obj.highway.actionParams, obj.simParams, obj.highway.mpcParams, obj.highway.LaneParams);
                        car_x_tmp   = car_x_all_tmp(:,end);
                        opp_x_tmp   = opp_x_all_tmp(:,end);
                        car_prev_actions_tmp        = [car_prev_actions_tmp, car_action];
                        opp_prev_actions_tmp        = [opp_prev_actions_tmp, opp_action];
                        
                        if check_if_collision(car_x_all_tmp, car_dim, opp_x_all_tmp, opp_dim,...
                                obj.highway.rewardParams.collisionMargin, obj.highway.Vehicles{obj.car_id}.on_ramp, obj.highway.LaneParams)
                            % ---------------------------------------------
                            % this should be picked large but not too large
                            % : for dense traffic, pick larger value to 
                            % penalize collision; it might not be good to 
                            % pick -inf, cuz it will rule out all the 
                            % branch gamma_j of ui if one gamma_j cause 
                            % collision in the future
                            % ---------------------------------------------
                            r_over_opp_vehicle(j)   = -length(adjVehicles)*horizon*obj.highway.rewardParams.maxReward; 
                            break;
                        else                            
                            single_r                = obj.reward(car_x_tmp, car_dim, car_action, opp_x_tmp, opp_dim);
                            r_over_opp_vehicle(j)   = r_over_opp_vehicle(j) + obj.highway.rewardParams.gamma^(k - 1) * single_r;
                        end
                        
                    end
                end
                Q_over_opp_vehicles(ith_opp)   = dot(r_over_opp_vehicle, likelihoods);
            end
            % averaged reward over all adjacent vehicle and its
            % possible action sequences
            Q_value = sum(Q_over_opp_vehicles)/length(adjVehicles); 
        end

        % =================================================================
        % Predict highway vehicles actions
        % =================================================================
        function [opp_actions_index, likelihoods] = highwayPrediction(obj, opp_i)   
            if ~obj.highway_info_updated
                error('Highway infomation not updated in Ego.');
            end

            opp_actions_index   = get_action_sequence(obj.simParams, obj.highway.mpcParams, ...
                obj.highway.Vehicles{opp_i}.actionTypeLists, obj.highway.Vehicles{opp_i}.prev_actions);
            likelihoods = zeros(size(opp_actions_index, 1), 1);

            if sum(obj.filter.likelihoods{opp_i} > obj.prediction_threshold.params_p, 'all') == 0
                error('obj.prediction_threshold.params_p too large');
            end

            for s_i = 1:length(obj.filter.S_set)
                likelihoods_incre       = cell(size(obj.filter.W_set, 1), 1);
                likelihoods_incre(:)    = {zeros(1, size(opp_actions_index, 1))};

                parfor  w_i = 1:size(obj.filter.W_set, 1)
                    if obj.filter.likelihoods{opp_i}(s_i, w_i) < obj.prediction_threshold.params_p 
                        continue;
                    end

                    hw_tmp  = copy(obj.highway);                
                    hw_tmp.Vehicles{opp_i}  = copy(obj.highway.Vehicles{opp_i});

                    hw_tmp.Vehicles{opp_i}.SVO.svo     = obj.filter.S_set(s_i);
                    hw_tmp.Vehicles{opp_i}.SVO.weights = obj.filter.W_set(w_i, :);            
                    hw_tmp.Vehicles{opp_i}.updateThetas();
                    hw_tmp.simParams    = obj.simParams; % make it the same as the ego for eval Q_value

                    rewards     = zeros(1, size(opp_actions_index, 1));            
                    % Compute expected pair game rewards
                    sum_p   = 0;
                    for i = 1:size(opp_actions_index, 1)
                        rewards(i)  = hw_tmp.Q_value_i(opp_i, opp_actions_index(i,:));
                        if isinf(exp(rewards(i)))
                            error('Infinite reward')
                        end
                        sum_p       = sum_p + exp(rewards(i));    
                    end
                    % Compute likelihood of get this action
                    % conditioned on parameters
                    for i = 1:size(opp_actions_index, 1)
                        action_likelihood           = exp(rewards(i))/sum_p;
                        likelihoods_incre{w_i}(i)   = obj.filter.likelihoods{opp_i}(s_i, w_i) * action_likelihood;
                    end                    
                end
                likelihoods_incre   = cell2mat(likelihoods_incre);
                for i = 1:size(opp_actions_index, 1)
                    likelihoods(i)  = likelihoods(i) + sum(likelihoods_incre(:, i), 'all');
                end
            end
            likelihoods = likelihoods./sum(likelihoods, 'all');
            rmv_idx     = [];
            for i = 1:length(likelihoods)
                if likelihoods(i) < obj.prediction_threshold.actions_p
                    rmv_idx = [rmv_idx, i];
                end
            end
            likelihoods(rmv_idx)            = [];
            opp_actions_index(rmv_idx, :)   = [];
            if isempty(likelihoods)
                error('obj.prediction_threshold.actions_p too large');
            end
            likelihoods = likelihoods./sum(likelihoods, 'all');
            if length(likelihoods) > obj.prediction_threshold.actions_n
                % random sample might lead to collision since some action
                % of highest probability can be ruled out
                % sampled_idx = randsample(length(likelihoods), obj.prediction_threshold.actions_n, true, likelihoods);
                [~, sampled_idx]    = maxk(likelihoods,obj.prediction_threshold.actions_n);
                likelihoods = likelihoods(sampled_idx);
                opp_actions_index   = opp_actions_index(sampled_idx, :);
            end
            likelihoods = likelihoods./sum(likelihoods, 'all');
        end

        % =================================================================
        % Define Ego reward function
        % =================================================================
        function R = reward(obj, state_car, dim_car, car_action, state_opp, dim_opp)
            if (0.5*(dim_car(2) + dim_opp(2)) + obj.highway.rewardParams.collisionMargin(2)) > obj.highway.LaneParams.lane_width
                error("Width of collision cushion is too large");
            end
            
            % This is used to further encourage merging completion when
            % we allow ego vehicle to abort lane change. If we use on_ramp
            % instead, then the ego can have possibility to choose aborting
            % lane change between two lanes (when the on_ramp flag is off),
            % sicne there is only longitudinal reward rx which is
            % undesirable.
            mergeCompleted  = (obj.state(2) >= (0.5 - 1e-2)*obj.highway.LaneParams.lane_width);

            rh  = reward_timeHeadway(state_car, state_opp, dim_car, dim_opp);
            rt  = reward_travelingTime(state_car, ~mergeCompleted, obj.highway.rewardParams.RampGoal, obj.highway.LaneParams);
            rc  = reward_controlEffort(car_action);

            % multiplier "r" is used to enforce collision avoidance
            R   = obj.highway.rewardParams.maxReward*rt;
        end

        % =================================================================
        % Remove out of bounds trajectories
        % =================================================================
        function car_actions_idx_out = rmEgoOutofBoundsSeq(obj, car_actions_idx_in)
            car_actions_idx_out     = [];
            for seq_i = 1:size(car_actions_idx_in, 1)
                actions_index           = car_actions_idx_in(seq_i, :);
                seq_i_OutofBounds       = false;
                car_x_tmp               = obj.state;
                car_prev_actions_tmp    = obj.prev_actions;
                CarPrevActDt.dtAct      = obj.simParams.dtCtrl;
                for k = 1:length(actions_index)
                    car_action              = actions_index(k);
                    CarPrevActDt.prevActs   = car_prev_actions_tmp;
                    [~, car_x_all_tmp, ~]   = VehicleSimulator(car_x_tmp, car_action, CarPrevActDt,...
                        obj.highway.actionParams, obj.simParams, obj.highway.mpcParams, obj.highway.LaneParams);
                    car_x_tmp               = car_x_all_tmp(:,end);
                    car_prev_actions_tmp    = [car_prev_actions_tmp, car_action];

                    if collision_out_of_bounds(car_x_all_tmp, [obj.vecLength, obj.vecWidth],...
                            obj.on_ramp, obj.highway.LaneParams)
                        seq_i_OutofBounds   = true;
                        break;
                    end
                end
                if ~seq_i_OutofBounds
                    car_actions_idx_out     = [car_actions_idx_out; actions_index];
                end
            end
        
        end

        % =================================================================
        % Obtain the adjacent vehicle
        % =================================================================
        function idxs = AdjacentVehicleIdxs(obj)
            % determine the maximum interaction distance
            distance_max    = obj.highway.mpcParams.nHorizon*obj.highway.mpcParams.dtHorizon*...
                (obj.highway.actionParams.v_lim(2) - obj.highway.actionParams.v_lim(1));

            idxCell = cell(3,3);
            x_car   = obj.highway.Vehicles{obj.car_id}.state(1);
            y_car   = obj.highway.Vehicles{obj.car_id}.state(2); 
            l_car   = obj.highway.Vehicles{obj.car_id}.vecLength;
            w_car   = obj.highway.Vehicles{obj.car_id}.vecWidth;
            for opp_i = 1:length(obj.highway.Vehicles)  
                if opp_i == obj.car_id || isequal(obj.highway.Vehicles{opp_i}.state, [-inf; 0; 0]) 
                    continue;
                end
                
                % case vehicles are not adjacent
                if abs(obj.highway.Vehicles{obj.car_id}.lane_id - obj.highway.Vehicles{opp_i}.lane_id) > 1
                    continue;
                end

                x_opp   = obj.highway.Vehicles{opp_i}.state(1);
                y_opp   = obj.highway.Vehicles{opp_i}.state(2); 
                l_opp   = obj.highway.Vehicles{opp_i}.vecLength;
                w_opp   = obj.highway.Vehicles{opp_i}.vecWidth;

                % case longitudinal distance is too far
                if abs(x_opp - x_car) >= distance_max
                    continue
                end
                
                % too far to be considered as environment vehicle
                % This should be removed for HighD merging validation since
                % there are unobservable vehicles being added
                % if x_opp < obj.highway.LaneParams.leftBound || x_opp > obj.highway.LaneParams.rightBound
                %     continue
                % end
                
                % CORA toolbox is used here
                is_intersecting_x   = isIntersecting(interval(x_opp-l_opp/2, x_opp+l_opp/2),...
                    interval(x_car-l_car/2, x_car+l_car/2));
                is_intersecting_y   = isIntersecting(interval(y_opp-w_opp/2, y_opp+w_opp/2),...
                    interval(y_car-w_car/2, y_car+w_car/2)); 
                
                i_idx   = 2 + sign(y_car-y_opp) * (~is_intersecting_y);
                j_idx   = 2 + sign(x_opp-x_car) * (~is_intersecting_x);
                
                if isempty(idxCell{i_idx, j_idx}) % if empty                    
                    idxCell{i_idx, j_idx}   = opp_i;
                else % not empty 
                    if ismember(j_idx, [1,3]) % is far away from the car_i
                        x_occupied  = obj.highway.Vehicles{idxCell{i_idx, j_idx}}.state(1);
                        if abs(x_opp-x_car) < abs(x_occupied-x_car)
                            idxCell{i_idx, j_idx}   = opp_i;
                        end
                    else % is near the car_i
                        idxCell{i_idx, j_idx}   = [idxCell{i_idx, j_idx}, opp_i];
                    end
                end
            end            
            idxs            = [idxCell{:}];
        end
    end
end