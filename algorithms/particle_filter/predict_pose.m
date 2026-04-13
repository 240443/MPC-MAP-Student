function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here

x = old_pose(1);
y = old_pose(2);
theta = old_pose(3);

v_L = motion_vector(1);
v_R = motion_vector(2);

d  = read_only_vars.agent_drive.interwheel_dist;
dt = read_only_vars.sampling_period;

v = (v_R + v_L) / 2;
omega = (v_L - v_R) / d; %check kinematics

% Probabilistic noise dispersion coefs --------
alpha_v = 0.1;
alpha_omega = 0.1;
base_noise = 0.1;

v_noisy = v + randn() * (alpha_v* abs(v) + base_noise);
omega_noisy = omega + randn() * (alpha_omega * abs(omega) + base_noise);

dx = v_noisy*cos(theta);
dy = v_noisy*sin(theta);

new_x = x + dx * dt;
new_y = y + dy * dt;
new_theta = theta + omega_noisy * dt;

% Wrap angle to [-pi, pi]
new_theta = atan2(sin(new_theta), cos(new_theta));
new_pose = [new_x, new_y, new_theta];

end

