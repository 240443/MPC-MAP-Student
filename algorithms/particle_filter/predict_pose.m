function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
% PREDICT_POSE  Differential-drive motion model with probabilistic noise.
%
% Sign convention matches ekf_predict: omega = (v_L - v_R)/d (Y-down arena).
% Noise has a base floor so particles still diffuse when the robot is
% stationary -- otherwise alpha-coefficient noise vanishes and PF freezes.

v_L = motion_vector(1);
v_R = motion_vector(2);

dt = read_only_vars.sampling_period;
d  = read_only_vars.agent_drive.interwheel_dist;

v     = (v_R + v_L) / 2;
omega = (v_L - v_R) / d;

theta = old_pose(3);

base_noise = 0.05;
alpha_v    = 0.10;
alpha_w    = 0.10;

sigma_xy = base_noise + alpha_v * abs(v);
sigma_th = base_noise + alpha_w * abs(omega);

new_x     = old_pose(1) + v * cos(theta) * dt + sigma_xy * randn() * dt;
new_y     = old_pose(2) + v * sin(theta) * dt + sigma_xy * randn() * dt;
new_theta = theta + omega * dt + sigma_th * randn() * dt;
new_theta = atan2(sin(new_theta), cos(new_theta));

new_pose = [new_x, new_y, new_theta];
end
