function path = astar(read_only_vars, public_vars)
% ASTAR  Plans a collision-free path on the occupancy grid using A*.
%
% Grid convention (matches generate_discrete_map):
%   col = round((world_x - x_min) / step) + 1   (x -> column)
%   row = round((world_y - y_min) / step) + 1   (y -> row)
%
% The obstacle map is pre-inflated (Task 2) before search, so the raw
% path already satisfies the 0.25 m clearance requirement.

path = [];

map  = read_only_vars.discrete_map.map;
step = read_only_vars.map.discretization_step;   % 0.2 m
lim  = read_only_vars.map.limits;                % [x_min y_min x_max y_max]

% --- Coordinate conversion helpers ------------------------------------
w2g = @(p) [round((p(2) - lim(2)) / step) + 1, ...   % row (y-axis)
             round((p(1) - lim(1)) / step) + 1];       % col (x-axis)
g2w = @(r, c) [(c - 1) * step + lim(1), ...            % world x
                (r - 1) * step + lim(2)];               % world y

% --- Start and goal ---------------------------------------------------
s = w2g(public_vars.mu(1:2));
g = w2g(read_only_vars.map.goal(1:2));

% --- Task 2: inflate obstacles by 0.25 m clearance -------------------
clearance_cells = ceil(0.25 / step);
occ = inflate_obstacles(map, clearance_cells);
[nR, nC] = size(occ);

% Clamp to grid bounds
s = max(1, min([nR, nC], s));
g = max(1, min([nR, nC], g));

if occ(s(1), s(2)) || occ(g(1), g(2))
    warning('A*: start or goal is inside an inflated obstacle.');
    return;
end

% --- A* search --------------------------------------------------------
dirs  = [-1,-1; -1,0; -1,1; 0,-1; 0,1; 1,-1; 1,0; 1,1];
costs = step * [sqrt(2); 1; sqrt(2); 1; 1; sqrt(2); 1; sqrt(2)];
heur  = @(r, c) step * sqrt((r - g(1))^2 + (c - g(2))^2);

gcost  = inf(nR, nC);
gcost(s(1), s(2)) = 0;
parent = zeros(nR, nC, 2);
closed = false(nR, nC);
open   = [heur(s(1), s(2)), s(1), s(2)];   % [f, row, col]

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

    for d = 1:8
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


function occ = inflate_obstacles(map, r)
% Replaces imdilate(map>0, strel('disk',r)) without any toolbox.
occ = map > 0;
[rows, cols] = find(occ);
[nR, nC] = size(occ);
for k = 1:numel(rows)
    rr = rows(k); cc = cols(k);
    for dr = -r:r
        for dc = -r:r
            if dr^2 + dc^2 <= r^2
                nr = rr+dr; nc = cc+dc;
                if nr>=1 && nr<=nR && nc>=1 && nc<=nC
                    occ(nr, nc) = true;
                end
            end
        end
    end
end
end