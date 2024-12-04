classdef highwayParams < matlab.mixin.Copyable
    properties
        x0_vehicles;
        latent_weights;
        vehiclesDims;
        
        EgoParams;
        LaneParams;
        simParams;
        mpcParams;
        actionParams;
        rewardParams;
    end
    methods
        function obj = highwayParams()
            % ========================================================
            % state = [x, y, v, 'SVO', svo, w1, w2, w3, w4]; or
            % x = longitudinal coordinates; y = lateral coordinates
            % v = velocity; svo = social value orientation
            % w1  = timeHeadway; w2 = travelingTime;
            % w3  = controlEffort; w4  = collision (can be omitted);
            % ========================================================
            x0_ramp     = { [-55, -2.5, 30], 'SVO', [0, [0, 1, 0]./1]}; 
            x0_highway  = { [-65, 2.5, 30], 'SVO', [90, [1, 0, 0]./1];
                            [45, 7.5, 28], 'SVO', [0, [0, 1, 0]./1];
                            [55, 7.5, 28], 'SVO', [0, [0, 0, 1]./1]}; 
            % -------------------------------------------------------------
            % Vehicle states
            % -------------------------------------------------------------                          
            obj.x0_vehicles     = cat(1, x0_ramp, x0_highway);
            obj.latent_weights  = true; % must set true, if there are IDM vehicles
            obj.vehiclesDims    = repmat([5;2], 1, size(obj.x0_vehicles, 1));
            %obj.vehiclesDims    = [5;2].*(ones(2, size(obj.x0_vehicles, 1)) + [2;0.5].*rand(2, size(obj.x0_vehicles, 1)));
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
            % -------------------------------------------------------------
            % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            % !!!!!! Open ramp: exiting and merging at the same time !!!!!! 
            % !!!!!!     is left for next step's development         !!!!!!
            % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            % -------------------------------------------------------------
            % -------------------------------------------------------------
            obj.LaneParams.lane_width   = 5;
            obj.LaneParams.leftBound    = -80;
            obj.LaneParams.rightBound   = 130;
            obj.LaneParams.lane_length  = obj.LaneParams.rightBound - obj.LaneParams.leftBound;
            obj.LaneParams.lane_ids     = 0:2; % 0 for ramp, 1~ for highway lanes
                                        
            if ismember(0, obj.LaneParams.lane_ids)            
                obj.LaneParams.rampClosed   = true; % case closed end ramp
                obj.LaneParams.rampStartX   = -50; 
                obj.LaneParams.rampMidX     = 95; 
                obj.LaneParams.rampEndX     = 115; 
                obj.LaneParams.RampBound    = [ obj.LaneParams.leftBound, -obj.LaneParams.lane_width;...
                                                obj.LaneParams.rampMidX, -obj.LaneParams.lane_width;...
                                                obj.LaneParams.rampEndX, 0;...
                                                obj.LaneParams.rightBound, 0];
                                            
                num_highway     = length(obj.LaneParams.lane_ids) - 1;
                obj.LaneParams.RoadBound    = [ obj.LaneParams.leftBound, num_highway*obj.LaneParams.lane_width;...
                                                obj.LaneParams.rightBound, num_highway*obj.LaneParams.lane_width;...
                                                obj.LaneParams.RampBound];
            else
                num_highway     = length(obj.LaneParams.lane_ids);
                obj.LaneParams.RoadBound    = [ obj.LaneParams.leftBound, num_highway*obj.LaneParams.lane_width;...
                                                obj.LaneParams.rightBound, num_highway*obj.LaneParams.lane_width;...
                                                obj.LaneParams.leftBound, 0;...
                                                obj.LaneParams.rightBound, 0];
            end
            
            % -------------------------------------------------------------
            % -------------------------------------------------------------                   
            obj.simParams.dt        = 0.01; % minimum time interval
            obj.simParams.dtSim     = 25*obj.simParams.dt; % time interval for update simulation trajectory
            obj.simParams.dtCtrl    = 4*obj.simParams.dtSim; % time interval for update control signal
            obj.simParams.dtLane    = 2*4*obj.simParams.dtSim; % time interval for a complete lane change (must be a even multiplier of obj.simParams.dtSim)

            obj.mpcParams.HorizonLane   = 2; % number of a MPC prediction horizon for lane change
            obj.mpcParams.nHorizon      = obj.mpcParams.HorizonLane + 1; % number of a MPC prediction horizon
            obj.mpcParams.dtHorizon     = obj.simParams.dtLane / obj.mpcParams.HorizonLane; % time interval for a MPC prediction horizon

            if mod(obj.mpcParams.dtHorizon, obj.simParams.dtCtrl) ~= 0
                error('mpcParams.dtHorizon must be an integer multiple of simParams.dtCtrl for generating control sequence');
            end
            
            obj.actionParams.a_lim  = [-6, 6]; % limits on vehicle acceleration       
            obj.actionParams.v_lim  = [18, 30]; % limits on vehicle velocity
            obj.actionParams.epsilon_acc    = 2; % minimum acc for IDM being counted as acc. or decel.
            
            % -------------------------------------------------------------
            % Parameters for Ego Decision
            % -------------------------------------------------------------
            obj.EgoParams.laneChangeAug     = true; % consider acc and dec in lane change
            obj.EgoParams.laneChangeAbort   = false; % consider lane change abortion
            obj.EgoParams.minCollisionTrimProb  = 0.5; % minimum probability of collision to trim a search direction 
            obj.EgoParams.prediction_threshold.params_p     = 1/22 - 1e-4; % cut the search tree using probability (parameter cmbns)
            obj.EgoParams.prediction_threshold.actions_p    = 1/43 - 1e-4; % cut the search tree using probability (action seq lists)
            obj.EgoParams.prediction_threshold.actions_n    = 5; % cut the search tree with fixed remaining branch (action seq lists)
            % -------------------------------------------------------------       
            % the simParams for ego and for the env vehicle shall be the 
            % same if you need to used the behavior model with Attention
            % -------------------------------------------------------------  
            obj.EgoParams.simParams         = obj.simParams;
            obj.EgoParams.simParams.dtCtrl  = obj.simParams.dtCtrl; % faster ego decision
            obj.EgoParams.simParams.dtBayes = obj.simParams.dtCtrl; % faster ego decision
            % -------------------------------------------------------------
            
            obj.rewardParams.enforceCollision   = true;
            obj.rewardParams.maxReward          = 20;
            obj.rewardParams.gamma              = 0.99; % discount factor
            obj.rewardParams.RampGoal           = [obj.LaneParams.rightBound; obj.LaneParams.lane_width/2]; % reaching the end of the lane
            obj.rewardParams.collisionMargin    = [3, 0.5]; % extending size of collision bounding box of vehicles         
            % -------------------------------------------------------------
            % -------------------------------------------------------------
        end
    end    
end