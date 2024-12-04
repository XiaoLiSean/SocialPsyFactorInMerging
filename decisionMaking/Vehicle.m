classdef Vehicle < matlab.mixin.Copyable
    properties (SetAccess = immutable)
        vecLength;
        vecWidth;
    end
    properties        
        state;

        SVO; % For SVO vehicles
        IDM; % For IDM vehicles
        
        on_ramp;
        lane_id;    

        % -----------------------------------------------------------------
        % Vehicle's action:
        % 1: slow down
        % 2: maintain lane and keep speed
        % 3: speed up 
        % 4: change lane left (obj.simParams.dkLane step)
        % 5: change lane right (obj.simParams.dkLane step)
        % -----------------------------------------------------------------        
        actionTypeLists;        
        prev_actions;   
        acceleration; % only used in IDM vehicles with different accelerations     
    end
    methods
        function obj = Vehicle(x0, vecDim, on_ramp, lane_id, actionTypeLists, paramType, params)
            obj.state       = x0;
            obj.vecLength   = vecDim(1);
            obj.vecWidth    = vecDim(2);
            obj.on_ramp     = on_ramp;
            obj.lane_id     = lane_id;
            obj.actionTypeLists     = actionTypeLists;
            obj.prev_actions        = [];
            obj.acceleration        = [];
            obj.SVO     = [];
            obj.IDM     = [];
            
            if strcmp(paramType, 'SVO')
                svo     = params(1);
                obj.SVO.svo     = svo;
                obj.SVO.weights = params(2:end);
                obj.updateThetas();
            elseif strcmp(paramType, 'IDM')
                obj.IDM.T       = params(1);
                obj.IDM.d_min   = params(2);
                obj.IDM.a_max   = params(3);
                obj.IDM.b       = params(4);
                obj.IDM.v_des   = params(5);                
            elseif ~strcmp(paramType, 'EGO') && ~strcmp(paramType, 'EGOLF') &&...
                    ~strcmp(paramType, 'EGOAtt') && ~strcmp(paramType, 'EGOAttSearch')
                error('Incorrect vehicle parameter type');
            end
        end

        function updateThetas(obj)
            switch obj.SVO.svo
                case 90
                    [obj.SVO.theta_1, obj.SVO.theta_2]  = deal(0, 1);
                case 45
                    [obj.SVO.theta_1, obj.SVO.theta_2]  = deal(0.5, 0.5);
                case 0
                    [obj.SVO.theta_1, obj.SVO.theta_2]  = deal(1, 0);
                case -45
                    [obj.SVO.theta_1, obj.SVO.theta_2]  = deal(0.5, -0.5);
                otherwise
                    error('Incorrect SVO')
            end
        end
    end
end