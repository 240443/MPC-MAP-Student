function [weights] = weight_particles(particle_measurements, lidar_distances)
% WEIGHT_PARTICLES  Gaussian observation model on LIDAR distance error.
%
% For each particle, sums Gaussian log-likelihoods across valid rays
% (rays that didn't hit anything return inf and are excluded). Uses the
% log-sum-exp trick before exponentiating for numerical stability.

SIGMA = 0.3;

N = size(particle_measurements, 1);
z = lidar_distances(:)';

log_w = zeros(N, 1);
for i = 1 : N
    diff  = particle_measurements(i, :) - z;
    valid = isfinite(diff);
    if any(valid)
        log_w(i) = -0.5 * sum(diff(valid).^2) / SIGMA^2;
    else
        log_w(i) = -inf;
    end
end

log_w   = log_w - max(log_w);
weights = exp(log_w);

s = sum(weights);
if s > 0
    weights = weights / s;
else
    weights = ones(N, 1) / N;
end
end
