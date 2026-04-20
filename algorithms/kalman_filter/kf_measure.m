function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)

z = z(:);
%Compute Kalman Gain
% Size: [3×2] state is 3, measurement is 2
K = sigma*kf.C'/(kf.C*sigma*kf.C' + kf.Q);

%Compute innovation
innovation = z - kf.C*mu;

%Update mean
new_mu = mu + K*innovation;

%Update covariance
new_sigma = (eye(3) - K*kf.C) * sigma;
end