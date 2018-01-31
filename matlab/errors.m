function [pos_err, ang_err] = errors(points, final_rot, target)
  pos_err = points(:, end) - target(1:3)';

  delta_quat = product(inverse(quaternion(target(4:end))), quaternion.rotationmatrix(final_rot));
  [ang_err, axis] = AngleAxis(delta_quat);
  if ang_err > pi
    ang_err = ang_err - 2 * pi;
  end
end
