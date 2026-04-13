function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here
x_min = read_only_vars.map.limits(1);
y_min = read_only_vars.map.limits(2);
x_max = read_only_vars.map.limits(3);
y_max = read_only_vars.map.limits(4);

N = read_only_vars.max_particles;

particles = zeros(N, 3);
particles(:, 1) = x_min + rand(N, 1) * (x_max - x_min);  % x
particles(:, 2) = y_min + rand(N, 1) * (y_max - y_min);  % y
particles(:, 3) = rand(N, 1) * 2 * pi - pi;   
public_vars.particles = particles;
end

