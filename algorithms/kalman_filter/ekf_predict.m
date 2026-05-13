function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, dt)
% EKF_PREDICT  Differential-drive prediction step.
%
% Sign convention: omega = (v_L - v_R)/d to match the Y-down simulator.
% Angle wrap uses atan2(sin,cos) -- no toolbox dependency.

x     = mu(1);
y     = mu(2);
theta = mu(3);

v_L = u(1);
v_R = u(2);

v     = (v_R + v_L) / 2;
omega = (v_L - v_R) / 0.2;

new_x     = x + v * cos(theta) * dt;
new_y     = y + v * sin(theta) * dt;
new_theta = theta + omega * dt;
new_theta = atan2(sin(new_theta), cos(new_theta));
new_mu    = [new_x; new_y; new_theta];

G      = eye(3);
G(1,3) = -v * sin(theta) * dt;
G(2,3) =  v * cos(theta) * dt;

new_sigma = G * sigma * G' + kf.R;
end
