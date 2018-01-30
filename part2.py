#!/usr/bin/env python3
import autograd.numpy as np
from autograd import grad, jacobian  
from autograd import elementwise_grad as egrad 

import scipy.optimize as opt
import transforms3d_grad as transforms3d

def sphere_line_intersection(l1, l2, sp, r):

    def square(f):
        return f * f
    from autograd.numpy import sqrt

    # l1[0],l1[1],l1[2]  P1 coordinates (point of line)
    # l2[0],l2[1],l2[2]  P2 coordinates (point of line)
    # sp[0],sp[1],sp[2], r  P3 coordinates and radius (sphere)
    # x,y,z   intersection coordinates
    #
    # This function returns a pointer array which first index indicates
    # the number of intersection point, followed by coordinate pairs.

    p1 = p2 = None

    a = square(l2[0] - l1[0]) + square(l2[1] - l1[1]) + square(l2[2] - l1[2])
    b = 2.0 * ((l2[0] - l1[0]) * (l1[0] - sp[0]) +
               (l2[1] - l1[1]) * (l1[1] - sp[1]) +
               (l2[2] - l1[2]) * (l1[2] - sp[2]))

    c = (square(sp[0]) + square(sp[1]) + square(sp[2]) + square(l1[0]) +
            square(l1[1]) + square(l1[2]) -
            2.0 * (sp[0] * l1[0] + sp[1] * l1[1] + sp[2] * l1[2]) - square(r))

    i = b * b - 4.0 * a * c

    if i < 0.0:
        pass  # no intersections
    elif i == 0.0:
        # one intersection
        p[0] = 1.0

        mu = -b / (2.0 * a)
        p1 = (l1[0] + mu * (l2[0] - l1[0]),
              l1[1] + mu * (l2[1] - l1[1]),
              l1[2] + mu * (l2[2] - l1[2]),
              )

    elif i > 0.0:
        # first intersection
        mu = (-b + sqrt(i)) / (2.0 * a)
        p1 = (l1[0] + mu * (l2[0] - l1[0]),
              l1[1] + mu * (l2[1] - l1[1]),
              l1[2] + mu * (l2[2] - l1[2]),
              )

        # second intersection
        mu = (-b - sqrt(i)) / (2.0 * a)
        p2 = (l1[0] + mu * (l2[0] - l1[0]),
              l1[1] + mu * (l2[1] - l1[1]),
              l1[2] + mu * (l2[2] - l1[2]),
              )

    return p1, p2


def fwd(start,length,r,p,y,qM):
    M = transforms3d.euler.euler2mat(r,p,y,'sxyz')
    return start+np.array([length,0,0]).dot(M), qM.dot(M)

def part2(target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles):
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
    def func(x0):
        pos = np.array([0,0,0])
        qM = np.eye(3)
        rs = x0[:N]
        ps = x0[N:2*N]
        ys = x0[2*N:]
        ll = link_length
        for r,p,y,l in zip(rs,ps,ys,ll):
            pos,qM = fwd(pos,l,r,p,y,qM)
        t = np.array(target)
        C = 1 # meters and radians. Close enough
        extra = 0.0
        for ob in obstacles:
            p0 = np.array([0,0,0])
            q = np.eye(3)
            for r,p,y,l in zip(rs,ps,ys,ll):
                p1,q = fwd(p0,l,r,p,y,q)
                i1,i2 = sphere_line_intersection(p0,p1,ob[:3],ob[3])
                p0 = p1
                if i1 is not None:
                    i1 = np.array(i1)
                    extra += (((ob[:3]-i1)**2-ob[3])**2).sum()
                if i2 is not None:
                    i2 = np.array(i2)
                    extra += (((ob[:3]-i2)**2-ob[3])**2).sum()
        quat = transforms3d.quaternions.mat2quat(qM)
        rot_error = 1.0 - ((quat*np.array([t[3],-t[4],-t[5],-t[6]]))**2 ).sum()
        return ((pos[:3]-t[:3])**2).sum() + C*rot_error + extra

    bounds =          [(x,y) for x,y in zip(min_roll, max_roll)]
    bounds = bounds + [(x,y) for x,y in zip(min_pitch,max_pitch)]
    bounds = bounds + [(x,y) for x,y in zip(min_yaw,  max_yaw)]
    
    midpoint = lambda mn,mx: mn+0.5*(mx-mn)
    x0 = [midpoint(min_roll[i],max_roll[i]) for i in range(N)] + [midpoint(min_pitch[i],max_pitch[i]) for i in range(N)] + [midpoint(min_yaw[i],  max_yaw[i]) for i in range(N)] 
    jac = grad(func)
    print(jac(x0))
    if False:     # quat should be norm 1 ?
        eps = 1e-3
        constraints =               [{'type:': 'eq', 'fun': lambda x: (x[3]**2 + x[4]**2 + x[5]**2 + x[6]**2) > 1.0-eps }]
        constraints = constraints + [{'type:': 'eq', 'fun': lambda x: (x[3]**2 + x[4]**2 + x[5]**2 + x[6]**2) < 1.0+eps }]
    else:
        constraints = []

    for ob in obstacles:
        pass #soft for now?

    # I think only method='SLSQP' is good?
    res = opt.minimize(func,x0=x0,bounds=bounds,constraints=constraints,method='SLSQP')
    print(res)
    return res.x[:N], res.x[N:2*N], res.x[2*N:]

if __name__ == '__main__':
    N = 6
    pi = 3.14159
    link_lengths = [2 for _ in range(N)]
    min_roll     = [-pi for _ in range(N)]
    max_roll     = [+pi for _ in range(N)]
    min_yaw      = [-pi/2.0 for _ in range(N)]
    max_yaw      = [+pi/2.0 for _ in range(N)]
    min_pitch    = [-pi for _ in range(N)]
    max_pitch    = [+pi for _ in range(N)]
    obstacles    = [ ] #[1,1,1, 0.75]

    target = [3,3,3, 1,0,0,0]
    res = part2(target,link_lengths,min_roll,max_roll,min_pitch,max_pitch,min_yaw,max_yaw,obstacles)

    import matplotlib as mpl
    mpl.use('Qt5Agg')
    mpl.rcParams['legend.fontsize'] = 10

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    x0 = np.hstack(res)#[0,0,0, pi/4,pi/4,pi/4, 0,0,0]
    # plot lines
    pos = np.array([0,0,0])
    qM = np.eye(3)
    rs = x0[:N]
    ps = x0[N:2*N]
    ys = x0[2*N:]
    ll = link_lengths
    for r,p,y,l in zip(rs,ps,ys,ll):
        pos0 = pos
        pos,qM = fwd(pos,l,r,p,y,qM)
        ax.plot([pos0[0],pos[0]], [pos0[1],pos[1]],[pos0[2],pos[2]])
        print(pos,qM)

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
    ax.set_xlim(0,5)
    ax.set_ylim(0,5)
    ax.set_zlim(0,5)
    plt.show()
