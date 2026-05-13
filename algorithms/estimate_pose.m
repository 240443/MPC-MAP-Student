function [estimated_pose] = estimate_pose(public_vars)
% ESTIMATE_POSE  Recover a single pose from the particle cloud.
%
% Position is the arithmetic mean of particle x,y. Angle uses the circular
% mean -- atan2(<sin>, <cos>) -- because averaging raw angles wraps wrong
% when particles straddle the +/-pi boundary.

if isfield(public_vars, 'particles') && ~isempty(public_vars.particles)
    p = public_vars.particles;
    x     = mean(p(:, 1));
    y     = mean(p(:, 2));
    theta = atan2(mean(sin(p(:, 3))), mean(cos(p(:, 3))));
    estimated_pose = [x, y, theta];
elseif isfield(public_vars, 'mu')
    estimated_pose = public_vars.mu';
else
    estimated_pose = [nan, nan, nan];
end
end
