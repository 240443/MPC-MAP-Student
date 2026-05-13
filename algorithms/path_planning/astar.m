function path = astar(read_only_vars, public_vars)
% ASTAR  Plans a collision-free path on the occupancy grid using A*.
%
% Grid convention (matches generate_discrete_map):
%   col = round((world_x - x_min) / step) + 1   (x -> column)
%   row = round((world_y - y_min) / step) + 1   (y -> row)
%
% The obstacle map is pre-inflated by 0.25 m before search, so the raw path
% already satisfies the clearance requirement.

path = [];

map  = read_only_vars.discrete_map.map;
step = read_only_vars.map.discretization_step;
lim  = read_only_vars.map.limits;

w2g = @(p) [round((p(2) - lim(2)) / step) + 1, ...
             round((p(1) - lim(1)) / step) + 1];
g2w = @(r, c) [(c - 1) * step + lim(1), ...
                (r - 1) * step + lim(2)];

% Start from the current pose estimate (PF-driven), goal from the map.
if isfield(public_vars, 'estimated_pose') && ...
   ~isempty(public_vars.estimated_pose) && ...
   all(isfinite(public_vars.estimated_pose(1:2)))
    start_xy = public_vars.estimated_pose(1:2);
elseif isfield(public_vars, 'mu')
    start_xy = public_vars.mu(1:2);
else
    warning('A*: no pose estimate available.');
    return;
end

s = w2g(start_xy);
g = w2g(read_only_vars.map.goal(1:2));

clearance_cells = ceil(0.1 / step);
occ = inflate_obstacles(map, clearance_cells);
[nR, nC] = size(occ);

s = max(1, min([nR, nC], s));
g = max(1, min([nR, nC], g));

if occ(s(1), s(2)) || occ(g(1), g(2))
    warning('A*: start or goal is inside an inflated obstacle.');
    return;
end

dirs  = [-1,-1; -1,0; -1,1; 0,-1; 0,1; 1,-1; 1,0; 1,1];
costs = step * [sqrt(2); 1; sqrt(2); 1; 1; sqrt(2); 1; sqrt(2)];
heur  = @(r, c) step * sqrt((r - g(1))^2 + (c - g(2))^2);

gcost  = inf(nR, nC);
gcost(s(1), s(2)) = 0;
parent = zeros(nR, nC, 2);
closed = false(nR, nC);
open   = [heur(s(1), s(2)), s(1), s(2)];

while ~isempty(open)
    [~, idx] = min(open(:, 1));
    r = open(idx, 2);
    c = open(idx, 3);
    open(idx, :) = [];

    if closed(r, c), continue; end
    closed(r, c) = true;

    if r == g(1) && c == g(2)
        path = reconstruct_path(parent, g, s, g2w);
        return;
    end

    for d = 1 : 8
        nr = r + dirs(d, 1);
        nc = c + dirs(d, 2);
        if nr < 1 || nr > nR || nc < 1 || nc > nC, continue; end
        if occ(nr, nc) || closed(nr, nc),           continue; end

        ng = gcost(r, c) + costs(d);
        if ng < gcost(nr, nc)
            gcost(nr, nc)     = ng;
            parent(nr, nc, :) = [r, c];
            open(end + 1, :)  = [ng + heur(nr, nc), nr, nc]; %#ok<AGROW>
        end
    end
end

warning('A*: no path found between start and goal.');
end


function path = reconstruct_path(parent, g, s, g2w)
path = g2w(g(1), g(2));
cur  = g;
while ~isequal(cur, s)
    cur  = squeeze(parent(cur(1), cur(2), :))';
    path = [g2w(cur(1), cur(2)); path]; %#ok<AGROW>
end
end
