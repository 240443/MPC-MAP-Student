function [new_particles] = resample_particles(particles, weights)

    N = size(particles, 1);
    cumsum_w = cumsum(weights);

    % One random start, then N evenly spaced pointers
    start = rand() / N;
    pointers = start + (0:N-1)' / N;   % Nx1

    new_particles = zeros(N, 3);
    j = 1;
    for i = 1:N
        while cumsum_w(j) < pointers(i)
            j = j + 1;
        end
        new_particles(i, :) = particles(j, :);
    end
end