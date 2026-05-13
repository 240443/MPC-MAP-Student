function [particles] = update_particle_filter(read_only_vars, public_vars)
% UPDATE_PARTICLE_FILTER  Predict + correct + conditional resample.
%
% False-convergence defences:
%   1. Likelihood monitor  — if mean raw weight is below LOST_THRESH,
%                            inject RESCUE_FRAC random particles
%   2. GNSS re-seed        — if GNSS is valid but disagrees with PF
%                            estimate by > GNSS_JUMP_M, inject near GNSS
%   3. Sharpening ramp     — exponent rises from 1 to 3 over RAMP_STEPS
%                            so early steps stay diverse

SKIP_STEPS              = 2;
ESS_THRESH              = 0.40;
MAX_PARTICLES_CONVERGED = 200;
CONV_RADIUS             = 0.8;

LOST_THRESH  = 1e-6;   % mean raw weight below this → assume lost
RESCUE_FRAC  = 0.25;   % fraction of cloud to replace with random particles
GNSS_JUMP_M  = 2.0;    % (m) GNSS vs PF disagreement that triggers re-seed
RAMP_STEPS   = 40;     % steps over which sharpening exponent rises 1 → 3

particles = public_vars.particles;
N         = size(particles, 1);
counter   = read_only_vars.counter;

lim  = read_only_vars.map.limits;
step = read_only_vars.map.discretization_step;
occ  = read_only_vars.discrete_map.map > 0;
[nR, nC] = size(occ);

% --- Shrink cloud once converged -----------------------------------------
if N > MAX_PARTICLES_CONVERGED
    spread = sqrt(var(particles(:,1)) + var(particles(:,2)));
    if spread < CONV_RADIUS
        particles = particles(1:MAX_PARTICLES_CONVERGED, :);
        N         = MAX_PARTICLES_CONVERGED;
    end
end

% --- Prediction (every step) ---------------------------------------------
for i = 1 : N
    particles(i, :) = predict_pose(particles(i, :), ...
                                   public_vars.motion_vector, ...
                                   read_only_vars);
end

% --- Correction (every SKIP_STEPS) ---------------------------------------
if mod(counter, SKIP_STEPS) ~= 0
    return;
end

n_rays = length(read_only_vars.lidar_config);
measurements = zeros(N, n_rays);
for i = 1 : N
    measurements(i, :) = compute_lidar_measurement(read_only_vars.map, ...
                                                   particles(i, :), ...
                                                   read_only_vars.lidar_config);
end
weights = weight_particles(measurements, read_only_vars.lidar_distances);

% --- Defence 1: likelihood monitor ---------------------------------------
mean_raw = mean(weights);   % weights are already normalised inside weight_particles
                            % so mean_raw ≈ 1/N when all equal; lower means
                            % some particles dominate — but truly lost means
                            % the un-normalised values were all tiny.
%
% Re-run raw (un-normalised) scoring to get absolute likelihood level.
log_w = zeros(N, 1);
z     = read_only_vars.lidar_distances(:)';
SIGMA = 0.3;
for i = 1 : N
    diff  = measurements(i, :) - z;
    valid = isfinite(diff);
    if any(valid)
        log_w(i) = -0.5 * sum(diff(valid).^2) / SIGMA^2;
    else
        log_w(i) = -inf;
    end
end
best_raw = exp(max(log_w));   % best single-particle likelihood (unnormalised)

if best_raw < LOST_THRESH
    n_rescue  = round(RESCUE_FRAC * N);
    particles = inject_random(particles, n_rescue, lim, occ, nR, nC, step);
    % Recompute weights after injection
    measurements = zeros(N, n_rays);
    for i = 1 : N
        measurements(i, :) = compute_lidar_measurement(read_only_vars.map, ...
                                                       particles(i, :), ...
                                                       read_only_vars.lidar_config);
    end
    weights = weight_particles(measurements, read_only_vars.lidar_distances);
end

% --- GNSS correction + Defence 2: re-seed on large jump ------------------
g = read_only_vars.gnss_position;
if ~isempty(g) && all(isfinite(g))
    % Check disagreement with current PF estimate
    if isfield(public_vars, 'estimated_pose') && ~isempty(public_vars.estimated_pose)
        pf_xy = public_vars.estimated_pose(1:2);
        gnss_dist = sqrt((pf_xy(1)-g(1))^2 + (pf_xy(2)-g(2))^2);
        if gnss_dist > GNSS_JUMP_M
            % PF is far from GNSS — inject half the cloud near GNSS
            n_seed    = round(0.5 * N);
            particles = inject_near_gnss(particles, n_seed, g, occ, nR, nC, lim, step);
            measurements = zeros(N, n_rays);
            for i = 1 : N
                measurements(i, :) = compute_lidar_measurement(read_only_vars.map, ...
                                                               particles(i, :), ...
                                                               read_only_vars.lidar_config);
            end
            weights = weight_particles(measurements, read_only_vars.lidar_distances);
        end
    end

    sigma_gnss = 0.5;
    dx = particles(:,1) - g(1);
    dy = particles(:,2) - g(2);
    log_gnss = -0.5*(dx.^2 + dy.^2) / sigma_gnss^2;
    log_gnss = log_gnss - max(log_gnss);
    weights  = weights .* exp(log_gnss);
    s = sum(weights);
    if s > 0, weights = weights / s; else, weights = ones(N,1)/N; end
end

% --- Defence 3: ramp sharpening exponent ---------------------------------
exponent = 1 + 2 * min(counter / RAMP_STEPS, 1);   % 1 → 3 over RAMP_STEPS
weights  = weights .^ exponent;
s = sum(weights);
if s > 0, weights = weights / s; else, weights = ones(N,1)/N; end

% --- Conditional resampling ----------------------------------------------
ESS = 1 / sum(weights.^2);
if ESS < ESS_THRESH * N
    particles = resample_particles(particles, weights);
end

end


% -------------------------------------------------------------------------
function particles = inject_random(particles, n, lim, occ, nR, nC, step)
% Replace the n worst particles with uniformly random free-space samples.
N = size(particles, 1);
count    = 0;
attempts = 0;
while count < n && attempts < 50 * n
    x   = lim(1) + (lim(3)-lim(1)) * rand();
    y   = lim(2) + (lim(4)-lim(2)) * rand();
    col = round((x-lim(1))/step) + 1;
    row = round((y-lim(2))/step) + 1;
    if row>=1 && row<=nR && col>=1 && col<=nC && ~occ(row,col)
        idx = N - n + count + 1;
        particles(idx, :) = [x, y, -pi + 2*pi*rand()];
        count = count + 1;
    end
    attempts = attempts + 1;
end
end


% -------------------------------------------------------------------------
function particles = inject_near_gnss(particles, n, g, occ, nR, nC, lim, step)
% Replace n particles with samples drawn around the GNSS fix.
N     = size(particles, 1);
count = 0;
attempts = 0;
while count < n && attempts < 50 * n
    x   = g(1) + 0.8 * randn();
    y   = g(2) + 0.8 * randn();
    col = round((x-lim(1))/step) + 1;
    row = round((y-lim(2))/step) + 1;
    if row>=1 && row<=nR && col>=1 && col<=nC && ~occ(row,col)
        idx = N - n + count + 1;
        particles(idx, :) = [x, y, -pi + 2*pi*rand()];
        count = count + 1;
    end
    attempts = attempts + 1;
end
end
