function [public_vars] = plan_motion(read_only_vars, public_vars)
%%PURE PURSUIT

% Tuning parameters – vary these to observe their effect (see report)
LOOKAHEAD_DIST = 0.5;   % (m)   look-ahead distance L
                                
LINEAR_VEL     = 0.55;  % (m/s) constant forward speed
                                                                 
GOAL_RADIUS    = 0.25;  % (m)   stop when within this distance of path end
% -----------------------------------------------------------------------

d       = read_only_vars.agent_drive.interwheel_dist;   % 0.2 m
max_vel = read_only_vars.agent_drive.max_vel;           % 1.0 m/s

% Retrieve MoCap pose 
pose = public_vars.estimated_pose;   % [x, y, theta] 
path = public_vars.path;            % N×2 waypoint matrix

% no pose or no path -> stand still
if isempty(pose) || any(isnan(pose)) || isempty(path) || size(path,1) < 2
    public_vars.motion_vector = [0, 0];
    return; 
end

x     = pose(1);
y     = pose(2);
theta = pose(3);

% Stop condition- robot close to the last waypoint
dist_to_end = sqrt((path(end,1) - x)^2 + (path(end,2) - y)^2);
if dist_to_end < GOAL_RADIUS
    public_vars.motion_vector = [0, 0];
    return;
end

%% HOT Pursuit ---------------------------------------------------------
target = get_target(pose, path, LOOKAHEAD_DIST);    % look-ahead point [x, y]

% Heading angle from robot to target (robot-fixed frame)
alpha = atan2(target(2) - y, target(1) - x) - theta;
alpha = atan2(sin(alpha), cos(alpha));% normalise to [-pi, pi]

% Arc curvature
kappa = 2 * sin(alpha) / LOOKAHEAD_DIST;

% Differential-drive conversion
omega = LINEAR_VEL * kappa;
v_R   = LINEAR_VEL + omega * d / 2;
v_L   = LINEAR_VEL - omega * d / 2;

% Clamp to hardware limits
v_R = max(-max_vel, min(max_vel, v_R));
v_L = max(-max_vel, min(max_vel, v_L));

public_vars.motion_vector = [v_R, v_L];

end
