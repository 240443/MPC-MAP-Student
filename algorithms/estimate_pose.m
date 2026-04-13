function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here

particles = public_vars.particles;

    if isempty(particles)
        estimated_pose = nan(1, 3);
        return;
    end

    estimated_pose(1) = mean(particles(:, 1));  % mean x
    estimated_pose(2) = mean(particles(:, 2));  % mean y

    % Circular mean for angle .. can't just average angles directly
    estimated_pose(3) = atan2(mean(sin(particles(:, 3))), ...
                              mean(cos(particles(:, 3))));

end

