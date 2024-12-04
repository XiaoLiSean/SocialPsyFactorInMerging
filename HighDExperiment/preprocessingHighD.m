function varargout = preprocessingHighD(dataset_num, side, addyVelocity)
    %======================================================================    
    addpath('./3rdPartyHighDRepo/data');  
    addpath('./decisionMaking');
    recordingMetaFilename   = sprintf('%s_recordingMeta.csv', dataset_num);
    tracksMetaFilename      = sprintf('%s_tracksMeta.csv', dataset_num);
    tracksFilename          = sprintf('%s_tracks.csv', dataset_num);

    recordingMeta   = readtable(recordingMetaFilename, 'Delimiter', ',');
    tracksMeta      = readtable(tracksMetaFilename, 'Delimiter', ',');
    tracks          = readtable(tracksFilename, 'Delimiter', ',');
    varargout       = {};
    %======================================================================  
    %% Prepare parameters (Upper Lane: X axis direction is opposite)
    if strcmp(side, 'upper')
        pUpper  = highwayParamsHighD();    
        markingsUpper   = regexp(recordingMeta.upperLaneMarkings{:}, ';', 'split');
        markingsUpper   = cellfun(@str2num, markingsUpper);
        vehicleIDsUpper = tracksMeta.id(tracksMeta.drivingDirection == 1);
        tracksUpper     = tracks(ismember(tracks.id, vehicleIDsUpper), :);
        % -----------------------------------------------------------------
        % Lane parameters: Geometry
        % -----------------------------------------------------------------
        pUpper.LaneParams.highDMinVy    = 0.4; % minimum y velocity to be consider in lane change
        pUpper.LaneParams.lane_width    = (markingsUpper(end)-markingsUpper(1))/(length(markingsUpper)-1);
        pUpper.LaneParams.leftBound     = -max(tracksUpper.x);
        pUpper.LaneParams.rightBound    = -min(tracksUpper.x);
        pUpper.LaneParams.lane_length   = pUpper.LaneParams.rightBound - pUpper.LaneParams.leftBound;
        pUpper.LaneParams.epsilon_lc    = 0.04*pUpper.LaneParams.lane_width;

        if ismember(dataset_num, {'58', '59', '60'}) 
            pUpper.LaneParams.lane_ids     = 0:(length(markingsUpper)-2); 
        else
            pUpper.LaneParams.lane_ids     = 1:(length(markingsUpper)-1); 
        end
        % ---------------------------------------------------------------------
        % Lane parameters: Bounding vertix for lane visualizarion
        % ---------------------------------------------------------------------
        if ismember(0, pUpper.LaneParams.lane_ids)    
            y_offset_upper  = -markingsUpper(2);
            pUpper.LaneParams.rampClosed 	= true; % case closed end ramp
            pUpper.LaneParams.rampStartX    = pUpper.LaneParams.leftBound + 35; 
            pUpper.LaneParams.rampMidX      = pUpper.LaneParams.leftBound + 180; 
            pUpper.LaneParams.rampEndX      = pUpper.LaneParams.leftBound + 225; 
            pUpper.LaneParams.RampBound     = [ pUpper.LaneParams.leftBound, -pUpper.LaneParams.lane_width;...
                                                pUpper.LaneParams.rampMidX, -pUpper.LaneParams.lane_width;...
                                                pUpper.LaneParams.rampEndX, 0;...
                                                pUpper.LaneParams.rightBound, 0];

            num_highway     = length(pUpper.LaneParams.lane_ids) - 1;
            pUpper.LaneParams.RoadBound  	= [ pUpper.LaneParams.leftBound, num_highway*pUpper.LaneParams.lane_width;...
                                                pUpper.LaneParams.rightBound, num_highway*pUpper.LaneParams.lane_width;...
                                                pUpper.LaneParams.RampBound];
        else   
            y_offset_upper  = -markingsUpper(1);
            num_highway     = length(pUpper.LaneParams.lane_ids);
            pUpper.LaneParams.RoadBound   	= [ pUpper.LaneParams.leftBound, num_highway*pUpper.LaneParams.lane_width;...
                                                pUpper.LaneParams.rightBound, num_highway*pUpper.LaneParams.lane_width;...
                                                pUpper.LaneParams.leftBound, 0;...
                                                pUpper.LaneParams.rightBound, 0];
        end
        % -----------------------------------------------------------------
        % Simulation parameters
        % -----------------------------------------------------------------
        pUpper.latent_weights   = true; % must set true, if there are IDM vehicles
        pUpper.simParams.fr     = recordingMeta.frameRate; % framerate
        
        pUpper.simParams.dt     = 1/recordingMeta.frameRate; % minimum time interval
        pUpper.simParams.dtSim  = 13*pUpper.simParams.dt; % simulation time per step [sec]
        pUpper.simParams.dtCtrl = pUpper.simParams.dtSim; % time interval for update control signal
        pUpper.simParams.dtLane = 8*pUpper.simParams.dtSim; % time interval for a complete lane change (must be a even multiplier of obj.simParams.dtSim)
    
        pUpper.mpcParams.HorizonLane    = 2; % number of a MPC prediction horizon for lane change
        pUpper.mpcParams.nHorizon       = pUpper.mpcParams.HorizonLane + 1; % number of a MPC prediction horizon
        pUpper.mpcParams.dtHorizon      = pUpper.simParams.dtLane / pUpper.mpcParams.HorizonLane; % time interval for a MPC prediction horizon
    
        if mod(pUpper.mpcParams.dtHorizon, pUpper.simParams.dtCtrl) ~= 0
            error('mpcParams.dtHorizon must be an integer multiple of simParams.dtCtrl for generating control sequence');
        end

        pUpper.actionParams.a_lim  = [-max(tracksUpper.xAcceleration), -min(tracksUpper.xAcceleration)]; % limits on vehicle acceleration       
        pUpper.actionParams.v_lim  = [-max(tracksUpper.xVelocity), -min(tracksUpper.xVelocity)]; % limits on vehicle velocity

        % -------------------------------------------------------------
        % Parameters for Ego Decision
        % -------------------------------------------------------------
        pUpper.EgoParams.laneChangeAug      = true; % consider acc and dec in lane change
        pUpper.EgoParams.laneChangeAbort    = false; % consider lane change abortion
        pUpper.EgoParams.minCollisionTrimProb  = 0.5; % minimum probability of collision to trim a search direction 
        pUpper.EgoParams.prediction_threshold.params_p  = 1/22 - 1e-4; % cut the search tree using probability (parameter cmbns)
        pUpper.EgoParams.prediction_threshold.actions_p = 1/93 - 1e-4; % cut the search tree using probability (action seq lists)
        pUpper.EgoParams.prediction_threshold.actions_n = 5; % cut the search tree with fixed remaining branch (action seq lists)
        % -------------------------------------------------------------       
        % the simParams for ego and for the env vehicle shall be the 
        % same if you need to used the behavior model with Attention
        % -------------------------------------------------------------             
        pUpper.EgoParams.simParams          = pUpper.simParams;
        pUpper.EgoParams.simParams.dtCtrl   = pUpper.simParams.dtCtrl; % faster ego decision
        pUpper.EgoParams.simParams.dtBayes  = pUpper.simParams.dtCtrl; % faster ego decision
        % -------------------------------------------------------------

        pUpper.rewardParams.enforceCollision   = true;
        pUpper.rewardParams.gamma              = 0.99; % discount factor
        pUpper.rewardParams.RampGoal           = [pUpper.LaneParams.rightBound; pUpper.LaneParams.lane_width/2]; % reaching the end of the lane
        pUpper.rewardParams.collisionMargin    = [3, 0.5]; % extending size of collision bounding box of vehicles         
        % -----------------------------------------------------------------
        % Vehicle Dimension and States
        % -----------------------------------------------------------------
        pUpper.vehiclesDims = [tracksMeta.width(vehicleIDsUpper)'; tracksMeta.height(vehicleIDsUpper)'];
        upperStateHistory   = cell(max(tracksUpper.frame), length(vehicleIDsUpper));
        for row_i = 1:size(tracksUpper, 1)
            frame_i     = tracksUpper.frame(row_i);
            vehicle_i   = find(vehicleIDsUpper == tracksUpper.id(row_i));
            x_raw_i     = tracksUpper.x(row_i);
            y_raw_i     = tracksUpper.y(row_i);

            l_i = tracksUpper.width(row_i);
            w_i = tracksUpper.height(row_i);
            x_i = -(x_raw_i + l_i/2); % shifted by bounding box, and reflex 
            y_i = (y_raw_i + w_i/2 + y_offset_upper); % shifted by bounding box, and shift bottom lane to zero, then reflex 
            v_i = -tracksUpper.xVelocity(row_i);
            if nargin == 3 && addyVelocity
                v_y     = tracksUpper.yVelocity(row_i);
                upperStateHistory{frame_i, vehicle_i}   = [x_i, y_i, v_i, v_y];
            else
                upperStateHistory{frame_i, vehicle_i}   = [x_i, y_i, v_i];
            end
        end
        varargout   = {varargout{:}, pUpper, upperStateHistory};
    end

    %% Prepare parameters (Lower Lane: Y axis direction is opposite)
    if strcmp(side, 'lower')
        pLower  = highwayParamsHighD();    
        markingsLower   = regexp(recordingMeta.lowerLaneMarkings{:}, ';', 'split');
        markingsLower   = cellfun(@str2num, markingsLower);
        vehicleIDsLower = tracksMeta.id(tracksMeta.drivingDirection == 2);
        tracksLower     = tracks(ismember(tracks.id, vehicleIDsLower), :);
        % -----------------------------------------------------------------
        % Lane parameters: Geometry
        % -----------------------------------------------------------------
        pLower.LaneParams.highDMinVy    = 0.4; % minimum y velocity to be consider in lane change
        pLower.LaneParams.lane_width    = (markingsLower(end)-markingsLower(1))/(length(markingsLower)-1);
        pLower.LaneParams.leftBound     = min(tracksLower.x);
        pLower.LaneParams.rightBound    = max(tracksLower.x);
        pLower.LaneParams.lane_length   = pLower.LaneParams.rightBound - pLower.LaneParams.leftBound;
        pLower.LaneParams.lane_ids      = 1:(length(markingsLower)-1); 
        pLower.LaneParams.epsilon_lc    = 0.04*pLower.LaneParams.lane_width;
        
        % -----------------------------------------------------------------
        % Lane parameters: Bounding vertix for lane visualizarion
        % -----------------------------------------------------------------
        num_highway     = length(pLower.LaneParams.lane_ids);
        pLower.LaneParams.RoadBound   	= [ pLower.LaneParams.leftBound, num_highway*pLower.LaneParams.lane_width;...
                                            pLower.LaneParams.rightBound, num_highway*pLower.LaneParams.lane_width;...
                                            pLower.LaneParams.leftBound, 0;...
                                            pLower.LaneParams.rightBound, 0];
        % -----------------------------------------------------------------
        % Simulation parameters
        % -----------------------------------------------------------------
        pLower.latent_weights   = true; % must set true, if there are IDM vehicles
        pLower.simParams.fr     = recordingMeta.frameRate; % framerate
        
        pLower.simParams.dt     = 1/recordingMeta.frameRate; % minimum time interval
        pLower.simParams.dtSim  = 13*pLower.simParams.dt; % simulation time per step [sec]
        pLower.simParams.dtCtrl = pLower.simParams.dtSim; % time interval for update control signal
        pLower.simParams.dtLane = 8*pLower.simParams.dtSim; % time interval for a complete lane change (must be a even multiplier of obj.simParams.dtSim)
    
        pLower.mpcParams.HorizonLane    = 2; % number of a MPC prediction horizon for lane change
        pLower.mpcParams.nHorizon       = pLower.mpcParams.HorizonLane + 1; % number of a MPC prediction horizon
        pLower.mpcParams.dtHorizon      = pLower.simParams.dtLane / pLower.mpcParams.HorizonLane; % time interval for a MPC prediction horizon
    
        if mod(pLower.mpcParams.dtHorizon, pLower.simParams.dtCtrl) ~= 0
            error('mpcParams.dtHorizon must be an integer multiple of simParams.dtCtrl for generating control sequence');
        end

        pLower.actionParams.a_lim  = [min(tracksLower.xAcceleration), max(tracksLower.xAcceleration)]; % limits on vehicle acceleration       
        pLower.actionParams.v_lim  = [min(tracksLower.xVelocity), max(tracksLower.xVelocity)]; % limits on vehicle velocity
        
        % -------------------------------------------------------------
        % Parameters for Ego Decision
        % -------------------------------------------------------------
        pLower.EgoParams.laneChangeAug      = true; % consider acc and dec in lane change
        pLower.EgoParams.laneChangeAbort    = false; % consider lane change abortion
        pLower.EgoParams.minCollisionTrimProb  = 0.5; % minimum probability of collision to trim a search direction 
        pLower.EgoParams.prediction_threshold.params_p  = 1/22 - 1e-4; % cut the search tree using probability (parameter cmbns)
        pLower.EgoParams.prediction_threshold.actions_p = 1/93 - 1e-4; % cut the search tree using probability (action seq lists)
        pLower.EgoParams.prediction_threshold.actions_n = 5; % cut the search tree with fixed remaining branch (action seq lists)
        % -------------------------------------------------------------       
        % the simParams for ego and for the env vehicle shall be the 
        % same if you need to used the behavior model with Attention
        % -------------------------------------------------------------              
        pLower.EgoParams.simParams          = pLower.simParams;
        pLower.EgoParams.simParams.dtCtrl   = pLower.simParams.dtCtrl; % faster ego decision
        pLower.EgoParams.simParams.dtBayes  = pLower.simParams.dtCtrl; % faster ego decision
        % -------------------------------------------------------------

        pLower.rewardParams.enforceCollision   = true;
        pLower.rewardParams.gamma              = 0.99; % discount factor
        pLower.rewardParams.RampGoal           = [pLower.LaneParams.rightBound; pLower.LaneParams.lane_width/2]; % reaching the end of the lane
        pLower.rewardParams.collisionMargin    = [3, 0.5]; % extending size of collision bounding box of vehicles         
        % -----------------------------------------------------------------
        % Vehicle Dimension and States
        % -----------------------------------------------------------------
        pLower.vehiclesDims = [tracksMeta.width(vehicleIDsLower)'; tracksMeta.height(vehicleIDsLower)'];
        lowerStateHistory   = cell(max(tracksLower.frame), length(vehicleIDsLower));
        for row_i = 1:size(tracksLower, 1)
            frame_i     = tracksLower.frame(row_i);
            vehicle_i   = find(vehicleIDsLower == tracksLower.id(row_i));
            x_raw_i     = tracksLower.x(row_i);
            y_raw_i     = tracksLower.y(row_i);

            l_i = tracksLower.width(row_i);
            w_i = tracksLower.height(row_i);
            x_i = (x_raw_i + l_i/2); % shifted by bounding box, and reflex 
            y_i = -(y_raw_i + w_i/2) + markingsLower(end); % shifted by bounding box, reflex, and shift bottom lane to zero
            v_i = tracksLower.xVelocity(row_i);
            if nargin == 3 && addyVelocity
                v_y     = -tracksLower.yVelocity(row_i);
                lowerStateHistory{frame_i, vehicle_i}   = [x_i, y_i, v_i, v_y];
            else
                lowerStateHistory{frame_i, vehicle_i}   = [x_i, y_i, v_i];
            end
        end
        varargout   = {varargout{:}, pLower, lowerStateHistory};
    end
end