function [new_particles] = resample_particles(particles, weights)
% RESAMPLE_PARTICLES  Systematic resampling.
%
% Places N evenly spaced pointers along the cumulative weight distribution,
% offset by a single uniform random shift. Particles with higher weights
% cover more of the [0,1] line and so get selected more often.

N    = size(particles, 1);
pos  = ((0:N-1)' + rand()) / N;
cumw = cumsum(weights(:));
cumw(end) = 1;        % guard against floating-point drift

new_particles = zeros(size(particles));
j = 1;
for i = 1 : N
    while pos(i) > cumw(j) && j < N
        j = j + 1;
    end
    new_particles(i, :) = particles(j, :);
end
end
