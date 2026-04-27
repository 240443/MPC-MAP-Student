function path = plan_path(read_only_vars, public_vars)
% PLAN_PATH  Entry point for path planning.
%
% Set planning_required = 1 to replan from the current position to the goal.
% Set planning_required = 0 to keep the previously computed path unchanged.

planning_required = 1;

if planning_required
    path = astar(read_only_vars, public_vars);
    path = smooth_path(path, read_only_vars);   % <-- read_only_vars passed here
else
    path = public_vars.path;
end
end