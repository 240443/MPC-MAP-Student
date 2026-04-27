function new_path = smooth_path(old_path, read_only_vars)
% SMOOTH_PATH  Iterative gradient-descent path smoother (lecture algorithm).
%
% Each interior waypoint is updated by:
%   p_i += alpha * (p_i_orig - p_i)  +  beta * (p_{i-1} + p_{i+1} - 2*p_i)
%
% Parameter effects:
%   alpha  - data term: pulls waypoints back toward the original A* path.
%            Larger alpha => path stays close to A* (less smooth, safer).
%   beta   - smoothness term: pulls each point toward its neighbours' midpoint,
%            reducing curvature. Larger beta => smoother arcs but waypoints
%            may drift farther from the original route.
%
% Obstacle-aware: candidate moves that would enter the inflated obstacle space
% are rejected, so the 0.25 m clearance guaranteed by A* is preserved.

new_path = old_path;

if size(old_path, 1) < 3
    return;
end

map  = read_only_vars.discrete_map.map;
step = read_only_vars.map.discretization_step;
lim  = read_only_vars.map.limits;

clearance_cells = ceil(0.25 / step);
occ  = imdilate(map > 0, strel('disk', clearance_cells));
[nR, nC] = size(occ);

p2g  = @(p) [round((p(2) - lim(2)) / step) + 1, ...
              round((p(1) - lim(1)) / step) + 1];

is_free = @(p) cell_free(p2g(p), nR, nC, occ);

% Tuning parameters
alpha = 0.15;    % data fidelity  (0.05 - 0.3)
beta  = 0.35;    % smoothness     (0.2  - 0.6)
tol   = 1e-5;    % convergence threshold on total displacement

for iter = 1:3000
    delta = 0;
    for i = 2 : size(new_path, 1) - 1
        correction = alpha * (old_path(i, :) - new_path(i, :)) + ...
                     beta  * (new_path(i-1, :) + new_path(i+1, :) - 2 * new_path(i, :));
        candidate = new_path(i, :) + correction;
        if is_free(candidate)
            new_path(i, :) = candidate;
            delta = delta + norm(correction);
        end
    end
    if delta < tol, break; end
end
end


function ok = cell_free(gi, nR, nC, occ)
r  = gi(1);
c  = gi(2);
ok = r >= 1 && r <= nR && c >= 1 && c <= nC && ~occ(r, c);
end
