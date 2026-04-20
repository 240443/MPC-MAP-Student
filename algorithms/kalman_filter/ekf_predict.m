function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, dt)


x     = mu(1); % x
y     = mu(2); % y
theta = mu(3); % theta

v_L = u(1);
v_R = u(2);

v     = (v_R + v_L) / 2;
omega = (v_L - v_R) / 0.2;

new_x     = x + v * cos(theta) * dt;
new_y     = y + v * sin(theta) * dt;
new_theta = theta + omega * dt;
new_theta = wrapToPi(new_theta);
new_mu    = [new_x; new_y; new_theta];


% Build Jacobian G [3×3] — derivative of g w.r.t state
G      = eye(3);
G(1,3) = -v*sin(theta)*dt;        %∂x'/∂θ  = -v·sin(θ)·dt
G(2,3) = v*cos(theta)*dt;         %∂y'/∂θ  =  v·cos(θ)·dt

% Propagate covariance
new_sigma = G * sigma * G' + kf.R;

end