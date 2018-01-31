function cost = sphere_collisions(points, obstacles)
  RADIUS_ADDITION = 0.1;
  cost = 0;
  for s_i = 1 : size(obstacles, 1)
    rad_eff = obstacles(s_i, 4) + RADIUS_ADDITION;

    for p_i = 2 : size(points, 2)
      dist_sq = sum(obstacles(s_i, 1:3)' - points(:, p_i)) .^ 2;
      if dist_sq < rad_eff ^ 2
        cost = cost + dist_sq;
      end

      bminusa =  points(:, p_i) - points(:, p_i - 1);
      pminusa =  obstacles(s_i, 1:3)' - points(:, p_i - 1);
      a = sum(bminusa .^ 2);
      b = sum(-2 .* pminusa .* bminusa);
      c = sum(pminusa .^ 2) - rad_eff ^ 2;

      r = roots([a b c]);
      for root_i = 1 : 2
        if any(isreal(r(root_i)) && r(root_i) >= 0 && r(root_i) <= 1)
          cost = cost + r(root_i) - r(root_i) ^ 2;
          return;
        end
      end
    end
  end
end
