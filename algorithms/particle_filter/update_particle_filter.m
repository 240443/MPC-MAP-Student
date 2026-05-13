function [particles] = update_particle_filter(read_only_vars, public_vars)
% UPDATE_PARTICLE_FILTER  One full PF iteration: predict + correct + resample.
%
% Correction fuses LIDAR (always available) with GNSS (when the reading is
% finite, i.e. the robot is outside any GNSS-denied zone). Final weight
% sharpening amplifies inter-particle differences before resampling to speed
% up convergence without changing relative ordering.

particles = public_vars.particles;
N         = size(particles, 1);

% I. Prediction
for i = 1 : N
    particles(i, :) = predict_pose(particles(i, :), ...
                                   public_vars.motion_vector, ...
                                   read_only_vars);
end

% II. LIDAR correction
n_rays = length(read_only_vars.lidar_config);
measurements = zeros(N, n_rays);
for i = 1 : N
    measurements(i, :) = compute_lidar_measurement(read_only_vars.map, ...
                                                   particles(i, :), ...
                                                   read_only_vars.lidar_config);
end
weights = weight_particles(measurements, read_only_vars.lidar_distances);

% III. GNSS correction (only when not in a denied zone)
g = read_only_vars.gnss_position;
if ~isempty(g) && all(isfinite(g))
    sigma_gnss = 0.5;
    dx = particles(:, 1) - g(1);
    dy = particles(:, 2) - g(2);
    log_gnss = -0.5 * (dx.^2 + dy.^2) / sigma_gnss^2;
    log_gnss = log_gnss - max(log_gnss);
    weights  = weights .* exp(log_gnss);
    s = sum(weights);
    if s > 0
        weights = weights / s;
    else
        weights = ones(N, 1) / N;
    end
end

% IV. Sharpen and resample
weights = weights .^ 3;
weights = weights / sum(weights);
particles = resample_particles(particles, weights);
end
