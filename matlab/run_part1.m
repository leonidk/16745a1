target_pos = [ 2 5 9 ];
%target_pos = [ -18 0 0 ];
target_q = [ 0.707 0 0.707 0 ];

link_lengths = [ 3 3 3 3 3 3 ];

min_roll = [ -pi/10 -pi -pi -pi -pi -pi ];
max_roll = [ pi/10 pi pi pi pi pi ];

min_pitch = [ -pi -pi -pi -pi -pi -pi ];
max_pitch = [ pi pi pi pi pi pi ];

min_yaw = [ -pi/4 -pi -pi -pi -pi -pi ];
max_yaw = [ pi/4 pi pi pi pi pi ];

% x y z radius
obstacles = [
 2 7 2 7
 2 2 4 0.4
];

target = [ target_pos target_q ];

[r, p, y] = part1(target, link_lengths, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles);
[points, final_rot] = fk(link_lengths, [y; p; r])
[pos_err, ang_err] = errors(points, final_rot, target);

distance_to_goal = norm(pos_err)
angle_error = ang_err

vis(points, obstacles, target);
