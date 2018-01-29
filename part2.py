#!/usr/bin/env python3
import numpy as np
import scipy.optimize as opt


def part2( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles):
    """Function that uses analytic gradients to do optimization for inverse kinematics in a snake robot

    Args:
        target: [x, y, z, q0, q1, q2, q3]' position and orientation of the end
           effector
        link_length : Nx1 vectors of the lengths of the links
        min_xxx, max_xxx are the vectors of the
           limits on the roll, pitch, yaw of each link.
        limits for a joint could be something like [-pi, pi]
        obstacles: A Mx4 matrix where each row is [ x y z radius ] of a sphere
           obstacle. M obstacles.
    """
    return

if __name__ == '__main__':
    print('jakobs day')