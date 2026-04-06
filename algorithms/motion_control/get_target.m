function [target] = get_target(pose, path, lookahead_dist)


x = pose(1);
y = pose(2);
n = size(path, 1);

% Find the closest point on the path
dists = sqrt((path(:,1) - x).^2 + (path(:,2) - y).^2);
[~, closest_idx] = min(dists);

% --- 2. Walk forward until a point is beyond the lookahead circle ------
target = path(end, :);          % default: stop at path end

for i = closest_idx : n
    d = sqrt((path(i,1) - x)^2 + (path(i,2) - y)^2);
    if d >= lookahead_dist
        target = path(i, :);
        break;
    end
end

end
