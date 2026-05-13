function [public_vars] = student_workspace(read_only_vars, public_vars)
% STUDENT_WORKSPACE  Top-level orchestration.
%
% Phases:
%   counter == 1                init PF
%   counter 2..CALIB            PF update each step (robot stationary,
%                               particles tighten under LIDAR/GNSS)
%   counter == CALIB            plan A* path from PF estimate, smooth it
%   counter > CALIB             PF update + Pure Pursuit using PF estimate

addpath algorithms/uncertainties/;
addpath algorithms/particle_filter/;
addpath algorithms/kalman_filter/;
addpath algorithms/path_planning/;
addpath algorithms/motion_control/;

CALIB_STEPS  = 75;
plot_enable  = 0;
SPIN_VEL    = 0.1;   % (m/s) each wheel, opposite directions — pure rotation

% --- counter == 1: initialise PF, hold position --------------------------
if read_only_vars.counter == 1
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars.motion_vector = [0, 0];
    public_vars.path          = [];
    public_vars.estimated_pose = estimate_pose(public_vars);
    clear functions;
    return;
end

% --- Sensor noise statistics (used by EKF init if it runs) ---------------
public_vars.uncertainties = calc_unc(read_only_vars, plot_enable);

% --- PF update every step ------------------------------------------------
public_vars.particles      = update_particle_filter(read_only_vars, public_vars);
public_vars.estimated_pose = estimate_pose(public_vars);

% --- End of calibration: plan the path -----------------------------------
if read_only_vars.counter < CALIB_STEPS
    public_vars.motion_vector = [SPIN_VEL, -SPIN_VEL];
    return;
end

% --- End of calibration: stop and plan -----------------------------------
if read_only_vars.counter == CALIB_STEPS
    public_vars.motion_vector = [0, 0];
    public_vars.path = plan_path(read_only_vars, public_vars);
    return;
end

% --- After calibration: drive --------------------------------------------
if read_only_vars.counter > CALIB_STEPS
    public_vars = plan_motion(read_only_vars, public_vars);
end
end
