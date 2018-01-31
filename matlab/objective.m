function cost = objective( target, link_lengths, lb, ub, obstacles, angles, weights)
  [points, final_rot] = fk(link_lengths, angles);
  [end_pos_err, angle] = errors(points, final_rot, target);

  center_angles = (lb + ub) / 2;

  cost = weights.POS * sum(end_pos_err .^ 2) + ...
         weights.ATT * angle ^ 2 + ...
         weights.LIMIT * sum(sum((angles - center_angles) .^ 2)) + ...
         weights.COLLISION * sphere_collisions(points, obstacles);
end
