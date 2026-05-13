function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
% INIT_KALMAN_FILTER  Initialise EKF state from the current pose estimate.
%
% Seeded from whatever localizer is running (PF mean here). No hardcoded
% heading -- works for any start orientation defined in setup.m.

public_vars.kf.C = [eye(2), zeros(2,1)];
public_vars.kf.R = diag([0.001, 0.001, 0.0005]);

if isfield(public_vars, 'uncertainties') && isfield(public_vars.uncertainties, 'gnss_cov')
    public_vars.kf.Q = public_vars.uncertainties.gnss_cov;
else
    public_vars.kf.Q = diag([0.01, 0.01]);
end

est = estimate_pose(public_vars);
public_vars.mu    = est(:);
public_vars.sigma = diag([0.1, 0.1, 0.1]);
end
