function vis(points, obstacles, target)
  figure

  plot3(points(1, :), points(2, :), points(3, :))
  hold on

  for i = 1 : size(points, 2)
    plot3(points(1, i), points(2, i), points(3, i), 'ro')
    hold on
  end

  plot3(target(1), target(2), target(3), 'gx', 'MarkerSize', 20, 'LineWidth', 4)

  for i = 1 : size(obstacles, 1)
    [x, y, z] = sphere;
    surf(obstacles(i, 4) * x + obstacles(i, 1), ...
         obstacles(i, 4) * y + obstacles(i, 2), ...
         obstacles(i, 4) * z + obstacles(i, 3));
  end

  axis equal
  grid on
end
