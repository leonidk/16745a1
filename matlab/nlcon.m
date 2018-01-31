function [c, ceq] = nlcon(angles, link_lengths, obstacles)
  %R[points, final_rot] = fk(link_lengths, angles);

  %RRADIUS_ADDITION = 0.1;
  %Rcost = 0;
  %Rfor s_i = 1 : size(obstacles, 1)
  %R  for p_i = 2 : size(points, 2)
  %R    bminusa =  points(:, p_i) - points(:, p_i - 1);
  %R    pminusa =  obstacles(s_i, 1:3)' - points(:, p_i - 1);
  %R    a = sum(bminusa .^ 2);
  %R    b = sum(-2 .* pminusa .* bminusa);
  %R    c = sum(pminusa .^ 2) - (obstacles(s_i, 4) + RADIUS_ADDITION) ^ 2;

  %R    r = roots([a b c]);
  %R    for root_i = 1 : 2
  %R      if any(isreal(r(root_i)) && r(root_i) >= 0 && r(root_i) <= 1)
  %R        cost = cost + r(root_i) - r(root_i) ^ 2;
  %R        return;
  %R      end
  %R    end
  %R  end
  %Rend

  c = [];
  ceq = [];
end
