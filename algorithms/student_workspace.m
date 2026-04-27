function [public_vars] = student_workspace(read_only_vars, public_vars)

%Path selection is controlled by the PATH_SELECT variable in setup.m:
%    1 = straight line
%    2 = circular arc (upper semicircle)
%    3 = sine wave
addpath algorithms/uncertainties/;
plot_enable = 1;
if read_only_vars.counter == 1

    public_vars = init_particle_filter(read_only_vars, public_vars);
        % set to 1 to see histogram plots alongside the arena
    clear functions;
    
    public_vars.move_en = 0;


end % counter == 1



%% PURE PURSUIT OR HOT PURSUIT?

public_vars.uncertainties = calc_unc(read_only_vars, plot_enable);

%% Kalman filter
if read_only_vars.counter == 200
public_vars = init_kalman_filter(read_only_vars, public_vars);
%% PATH SELECTION:
public_vars.path = plan_path(read_only_vars, public_vars);
end

if read_only_vars.counter > 200
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
public_vars = plan_motion(read_only_vars, public_vars);
public_vars.estimated_pose = estimate_pose(public_vars);
end

% public_vars.particles     = update_particle_filter(read_only_vars, public_vars);



end
