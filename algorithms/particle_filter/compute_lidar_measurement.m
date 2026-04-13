function [measurement] = compute_lidar_measurement(map, pose, lidar_config)

    x     = pose(1);
    y     = pose(2);
    theta = pose(3);

    ray_origin  = [x, y];
    measurement = zeros(1, length(lidar_config));

    for i = 1:length(lidar_config)
        direction     = theta + lidar_config(i);  % local angle → global
        intersections = ray_cast(ray_origin, map.walls, direction);

        if isempty(intersections)
            measurement(i) = inf;
        else
            % distance from ray origin to each intersection point
            diffs = intersections - ray_origin;
            dists = sqrt(sum(diffs.^2, 2));
            measurement(i) = min(dists); % pnly care
        end  
    end
end