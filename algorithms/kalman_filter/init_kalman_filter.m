function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here
public_vars.kf.C = [eye(2), zeros(2,1)];    % [2×3]
public_vars.kf.R = eye(3) * 0.001;           % [3×3] process noise
public_vars.kf.R(3,3) = 0.0005;
public_vars.kf.Q = public_vars.uncertainties.gnss_cov;  % [2×2] measurement noise

public_vars.mu = [mean(read_only_vars.gnss_history)'; pi/2];  % [3×1]
public_vars.sigma = zeros(3);   

fprintf('initiliazed position: ');
disp(size(public_vars.mu)) % [3×3]
end

