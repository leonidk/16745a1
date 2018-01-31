function [points, final_rot] = fk( link_lengths, angles )

points = zeros(3, numel(link_lengths) + 1);
final_rot = eye(3);

for i = numel(link_lengths) : -1 : 1
  rot = eul2rotm(angles(:, i)', 'ZYX');
  points(:, i + 1:end) = rot * (points(:, i + 1:end) + [ link_lengths(i) 0 0 ]');
  final_rot = rot * final_rot;
end

end
