function [public_vars] = init_particle_filter(read_only_vars, public_vars)
% INIT_PARTICLE_FILTER  Spawns N particles across the arena.
%
% If a valid GNSS reading is available, particles are seeded as a Gaussian
% cloud around it (fast convergence outdoor). Otherwise they are uniform
% across the map limits (works indoor too, just takes longer to collapse).
% Particles that land inside walls are rejected.

N = 300;

lim  = read_only_vars.map.limits;                 % [xmin ymin xmax ymax]
step = read_only_vars.map.discretization_step;
occ  = read_only_vars.discrete_map.map > 0;
[nR, nC] = size(occ);

g = read_only_vars.gnss_position;
seed_gnss = ~isempty(g) && all(isfinite(g));

particles = zeros(N, 3);
count     = 0;
attempts  = 0;

while count < N && attempts < 50 * N
    if seed_gnss
        x = g(1) + 1.0 * randn();
        y = g(2) + 1.0 * randn();
    else
        x = lim(1) + (lim(3) - lim(1)) * rand();
        y = lim(2) + (lim(4) - lim(2)) * rand();
    end

    col = round((x - lim(1)) / step) + 1;
    row = round((y - lim(2)) / step) + 1;

    if row >= 1 && row <= nR && col >= 1 && col <= nC && ~occ(row, col)
        count = count + 1;
        particles(count, :) = [x, y, -pi + 2*pi*rand()];
    end
    attempts = attempts + 1;
end

if count < N
    particles = particles(1:count, :);
end

public_vars.particles = particles;
end
