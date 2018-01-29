#!/usr/bin/env python3
import numpy as np
import scipy.optimize as opt
import transforms3d

def fwd(start,length,r,p,y):
    return    

def part1(target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles):
    """Function that uses optimization to do inverse kinematics for a snake robot

    Args:
        target:  [x, y, z, q0, q1, q2, q3]' position and orientation of the end effector
        link_length:  Nx1 vectors of the lengths of the links
        min_xxx, max_xxx are the vectors of the limits on the roll, pitch, yaw of each link.
        obstacles: A Mx4 matrix where each row is [ x y z radius ] of a sphere obstacle.
        M obstacles.

    Returns:
        r: N vector of roll
        p: N vector of pitch
        y: N vector of yaw

    """
    N = len(link_length)
    def fwd(ll):
        return [1,2,3,4,5,6]
    def func(x0):
        pose = fwd(x0)
        
        C = 1 # meters and radians. Close enough
        return np.sqrt((pose[:3]-target[:3])**2) + C*(1.0-(pose[3:] * pose[3:])**2)

    bounds =          [(x,y) for x,y in zip(min_roll, max_roll)]
    bounds = bounds + [(x,y) for x,y in zip(min_pitch,max_pitch)]
    bounds = bounds + [(x,y) for x,y in zip(min_yaw,  max_yaw)]
    
    midpoint = lambda mn,mx: mn+0.5*(mx-mn)
    x0 = [midpoint(min_roll,max_roll) for _ in range(N)] + [midpoint(min_pitch,max_pitch) for _ in range(N)] + [midpoint(min_yaw,  max_yaw) for _ in range(N)] 

    if False:     # quat should be norm 1 ?
        eps = 1e-3
        constraints =               [{'type:': 'eq', 'fun': lambda x: (x[3]**2 + x[4]**2 + x[5]**2 + x[6]**2) > 1.0-eps }]
        constraints = constraints + [{'type:': 'eq', 'fun': lambda x: (x[3]**2 + x[4]**2 + x[5]**2 + x[6]**2) < 1.0+eps }]
    else:
        contraints = []

    for ob in obstacles:
        constraints += {'type': 'ineq', 'fun': lambda x: (x[:3]-ob[:3])**2 > ob[3]**2 }

    # I think only method='SLSQP' is good?
    res = opt.minimize(func,x0=x0,bounds=bounds,constraints=constraints)
    return res.x[:N], res.x[N:2*N], res.x[2*N:]

if __name__ == '__main__':
    N = 3
    pi = 3.14159
    link_lengths = [1 for _ in range(N)]
    min_roll     = [-pi for _ in range(N)]
    max_roll     = [+pi for _ in range(N)]
    min_yaw      = [-pi/2.0 for _ in range(N)]
    max_yaw      = [+pi/2.0 for _ in range(N)]
    min_pitch    = [-pi for _ in range(N)]
    max_pitch    = [+pi for _ in range(N)]
    obstacles    = [[2,2,2, 1]]

    target = [1,1,1, 1,0,0,0]


    import matplotlib as mpl
    mpl.use('Qt5Agg')
    mpl.rcParams['legend.fontsize'] = 10

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # plot lines
    start = [0,0,0]
    for ll 
    theta = np.linspace(-4 * np.pi, 4 * np.pi, 10)
    z = np.linspace(-2, 2, 10)
    r = z**2 + 1
    x = r * np.sin(theta)
    y = r * np.cos(theta)
    ax.plot(x, y, z, label='parametric curve')

    # plot spheres
    for o in obstacles:
        u = np.linspace(0, 2 * np.pi, 10)
        v = np.linspace(0, np.pi, 10)
        x = o[3] * np.outer(np.cos(u), np.sin(v)) + o[0]
        y = o[3] * np.outer(np.sin(u), np.sin(v)) + o[1]
        z = o[3] * np.outer(np.ones(np.size(u)), np.cos(v)) + o[2]
        ax.plot_surface(x, y, z, color='b')

    # plot goal
    ax.scatter(target[0], target[1], target[2], c='r')

    ax.legend()
    ax.set_xlim(-3,3)
    ax.set_ylim(-3,3)
    ax.set_zlim(-3,3)
    plt.show()
