function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

fw = [0.25 0.25];
r = [0.15 0.25]; %roughly 90 degrees per 60 steps
l = [0.25 0.15];

if read_only_vars.counter > 0 && read_only_vars.counter <= 100
    public_vars.motion_vector = [0, 0];
elseif read_only_vars.counter > 100 && read_only_vars.counter <= 300
    public_vars.motion_vector = [0.3, 0.302];
elseif read_only_vars.counter > 300 && read_only_vars.counter <= 500
    public_vars.motion_vector = [0.3, 0.332];
elseif read_only_vars.counter > 500 && read_only_vars.counter <= 600
    public_vars.motion_vector = [0.3, 0.3];
elseif read_only_vars.counter > 600 && read_only_vars.counter <= 780
    public_vars.motion_vector = [0.334, 0.3];
elseif read_only_vars.counter > 780 && read_only_vars.counter <= 1000
    public_vars.motion_vector = [0.3, 0.3];
else
    public_vars.motion_vector = [0, 0];
end


end