function [state_sim, theta_sim] = LaneChangeTraj(x, y, v_x, actionType, prevActions, v_lim, a_lim, lane_width, dkLane, dtSim, dt)

% Input variables:
% x: ego x position
% y: ego y position
% v_x: ego x velocity; lane_width: lane width
% actionType: lane change direction 4/6/8 (left: const/dec/acc), 5/7/9 (right: const/dec/acc)
% lane_width: lane width
% epsilon_lc: to determine which step in dkLane
% dkLane: numbers of simulation time steps needed for a lane change
% dtSim: time duration of each simulation step
% dt: smallest simulation time_step

% Return coefficents of 5th order polynomial fit of lane change behaviours
% a_0 = a(1); a_1 = a(2) ....
% b_0 = b(1); b_1 = b(2) ....
% t_sim, x_sim for a duration of one simulation step 

% ------------------------------------------------------------------ 
% Prepare fitting parameter for lane change from 0 to dtLane
% ------------------------------------------------------------------ 
if isempty(prevActions)
    prevActions = 0;
end

laneChangeActions   = 4:9;
if ~ismember(actionType, laneChangeActions)
    error('must be lane change actions');
end
prevActions(~ismember(prevActions, laneChangeActions))  = NaN;

changeDir   = mod(actionType, 2);
dtLane      = dkLane*dtSim; % time duration of lane change

switch actionType - changeDir
    case 4
        a_x = 0;
    case 6
        a_u = min((v_lim(1) - v_x)/dtSim, 0); % a_x upper bound ensure it's deceleration
        a_x = max(a_lim(1),  a_u); % speed cannot exceed the limits
    case 8
        a_l = max((v_lim(2) - v_x)/dtSim, 0); % a_x lower bound ensure it's acceleration
        a_x = min(a_lim(2), a_l); % speed cannot exceed the limits
end

% ------------------------------------------------------------------ 
% Case which steps (k_cur) are the lane change in within the dkLane period
% ------------------------------------------------------------------ 
neededMoreSteps = 0;
if ismember(prevActions(end), laneChangeActions) && mod(prevActions(end), 2) == changeDir % this means we do have same lane change previously
    prevChangeDir   = mod(prevActions(end), 2);
    % this means the lane change is not ended
    consecutiveLaneChanges  = 0;
    for k = length(prevActions):-1:1
        if mod(prevActions(k), 2) == prevChangeDir
            consecutiveLaneChanges  = consecutiveLaneChanges + 1;
        else
            break;
        end
    end

    if mod(consecutiveLaneChanges, dkLane) == 0
        neededMoreSteps = 0;
    else                
        neededMoreSteps = dkLane - mod(consecutiveLaneChanges, dkLane);
    end
end 

if neededMoreSteps == 0 % case previous actions contains a completed lane changes
    k_cur   = 0;
else
    k_cur   = dkLane - neededMoreSteps;
end

t_cur       = k_cur*dtSim;
t_sim       = t_cur + (0:dt:dtSim);

% ------------------------------------------------------------------ 
% Parameter fitting 
% ------------------------------------------------------------------ 
v_0_virtual     = v_x - a_x*t_cur; % suppose actionType for entire lane change

[x_0, xdot_0, xddot_0, y_0, ydot_0, yddot_0]    = deal(0, v_0_virtual, a_x, 0, 0, 0);
[x_f, xdot_f, xddot_f, y_f, ydot_f, yddot_f]    = deal(v_0_virtual*dtLane + 0.5*a_x*dtLane^2, v_0_virtual + a_x*dtLane, 0, lane_width, 0, 0);

A   = [ 1, 0, 0, 0, 0, 0;
        1, dtLane, dtLane^2, dtLane^3, dtLane^4, dtLane^5;
        0, 1, 0, 0, 0, 0;
        0, 1, 2*dtLane, 3*dtLane^2, 4*dtLane^3, 5*dtLane^4;
        0, 0, 2, 0, 0, 0;
        0, 0, 2, 6*dtLane, 12*dtLane^2, 20*dtLane^3];
B   = A;

a   = inv(A) * [y_0; y_f; ydot_0; ydot_f; yddot_0; yddot_f];
a_0 = a(1); a_1 = a(2); a_2 = a(3); a_3 = a(4); a_4 = a(5); a_5 = a(6);

b   = inv(B) * [x_0; x_f; xdot_0; xdot_f; xddot_0; xddot_f];
b_0 = b(1); b_1 = b(2); b_2 = b(3); b_3 = b(4); b_4 = b(5); b_5 = b(6);

% ------------------------------------------------------------------ 
% Current Time steps offset
% ------------------------------------------------------------------ 
x_offset    = b_5*t_cur^5 + b_4*t_cur^4 + b_3*t_cur^3 + b_2*t_cur^2 + b_1*t_cur + b_0; % x distance that already traveled
y_offset    = a_5*t_cur^5 + a_4*t_cur^4 + a_3*t_cur^3 + a_2*t_cur^2 + a_1*t_cur + a_0; % y distance that already traveled

% ------------------------------------------------------------------ 
% Lane Change trajectory fitting
% ------------------------------------------------------------------ 
y_sim       = a_5*t_sim.^5 + a_4*t_sim.^4 + a_3*t_sim.^3 + a_2*t_sim.^2 + a_1*t_sim + a_0;
ydot_sim    = 5*a_5*t_sim.^4 + 4*a_4*t_sim.^3 + 3*a_3*t_sim.^2 + 2*a_2*t_sim + a_1;
x_sim       = b_5*t_sim.^5 + b_4*t_sim.^4 + b_3*t_sim.^3 + b_2*t_sim.^2 + b_1*t_sim + b_0;
xdot_sim    = 5*b_5*t_sim.^4 + 4*b_4*t_sim.^3 + 3*b_3*t_sim.^2 + 2*b_2*t_sim + b_1;

state_sim   = [x + (x_sim - x_offset); y + (-1)^changeDir.*(y_sim - y_offset); v_x + (t_sim-t_cur).*a_x];
theta_sim   = (-1)^changeDir*atan(ydot_sim./xdot_sim);

if abs(norm([xdot_sim(end), ydot_sim(end)], 2) - state_sim(3,end)) > 2
    error('Something wrong with the longitudinal velocity during lane change');
end

end