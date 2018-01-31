function [r, p, y] = part1( target, link_lengths, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles )
%% Function that uses optimization to do inverse kinematics for a snake robot

%%Outputs
  % [r, p, y] = roll, pitch, yaw vectors of the N joint angles
  %            (N link coordinate frames)
%%Inputs:
    % target: [x, y, z, q0, q1, q2, q3]' position and orientation of the end
    %    effector
    % link_length : Nx1 vectors of the lengths of the links
    % min_xxx, max_xxx are the vectors of the
    %    limits on the roll, pitch, yaw of each link.
    % limits for a joint could be something like [-pi, pi]
    % obstacles: A Mx4 matrix where each row is [ x y z radius ] of a sphere
    %    obstacle. M obstacles.

weights = struct;
weights.POS = 1;
weights.ATT = 0;
weights.LIMIT = 0.1;
weights.COLLISION = 0;

N = numel(link_lengths);

lb = [ min_yaw; min_pitch; min_roll ];
ub = [ max_yaw; max_pitch; max_roll ];

angles = 2 * pi * rand(3, N) - pi;

obj_f = @(angles) objective(target, link_lengths, lb, ub, obstacles, angles, weights);
options = optimset('MaxFunEvals', 1e6, ...
                   'MaxIter', 1e6, ...
                   'Algorithm', 'sqp');
[best_angles, cost] = fmincon(obj_f, angles, ...
                              [], [], [], [], ...
                              lb, ub, @nlcon, options)

r = best_angles(3, :);
p = best_angles(2, :);
y = best_angles(1, :);

end
