import sys
import os

if True:
    import autograd.numpy as np
    import transforms3d_grad as transforms3d
    from autograd import grad, jacobian, hessian, jacobian, hessian_vector_product
    from autograd import elementwise_grad as egrad
else: #not using grad path
    import transforms3d
    import numpy as np
import scipy.optimize as opt

# from http://paulbourke.net/geometry/circlesphere/sphere_line_intersection.py
def sphere_line_intersection(l1, l2, sp, r):

    def square(f):
        return f * f
    #from math import sqrt

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
        #p[0] = 1.0

        mu = -b / (2.0 * a)
        p1 = (l1[0] + mu * (l2[0] - l1[0]),
              l1[1] + mu * (l2[1] - l1[1]),
              l1[2] + mu * (l2[2] - l1[2]),
              )

    elif i > 0.0:
        # first intersection
        mu = (-b + np.sqrt(i)) / (2.0 * a)
        p1 = (l1[0] + mu * (l2[0] - l1[0]),
              l1[1] + mu * (l2[1] - l1[1]),
              l1[2] + mu * (l2[2] - l1[2]),
              )

        # second intersection
        mu = (-b - np.sqrt(i)) / (2.0 * a)
        p2 = (l1[0] + mu * (l2[0] - l1[0]),
              l1[1] + mu * (l2[1] - l1[1]),
              l1[2] + mu * (l2[2] - l1[2]),
              )

    return p1, p2


def fwd(start,length,r,p,y,qM):
    M = transforms3d.euler.euler2mat(r,p,y,'sxyz')
    return start+np.array([length,0,0]).dot(M), qM.dot(M)

def solve(target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles):
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
    #print('BOUNDS',bounds)
    #midpoint = lambda mn,mx: mn+0.5*(mx-mn)
    #x0 = [midpoint(min_roll[i],max_roll[i]) for i in range(N)] + [midpoint(min_pitch[i],max_pitch[i]) for i in range(N)] + [midpoint(min_yaw[i],  max_yaw[i]) for i in range(N)] 
    
    x0 = np.array(initial_x) + 1e-6
    constraints = []
    
    if use_grad or use_hess:
        jac = grad(func)
        def jac_reg(x):
            j = jac(x)
            if np.isfinite(j).all():
                return j
            else:
                return opt.approx_fprime(x,func,1e-6)
        jacob = jac_reg
    else:
        jacob = None

    if use_hess:
        hess = jacobian(grad(func))
        def hess_reg(x):
            h = hess(x)
            if np.isfinite(h).all():
                return h
            else:
                return opt.approx_fprime(x,jac_reg,1e-6)
        hessi = hess_reg
    else:
        hessi = None

    if method == 'CMA':
        import cma
        es = cma.CMAEvolutionStrategy(x0, 3.141592/2.0, {'bounds':list(zip(*bounds)),'verb_log':0})
        es.optimize(func)
        #print(es.result_pretty())
        resx = es.result.xbest
    elif method == 'diffev':
        def callback(xk, convergence):
            #print(convergence)
            if convergence < 0.01:
                return True
            return False
        res = opt.differential_evolution(func,bounds,callback=callback)
        print(res)
        resx = res.x
    else:
        res = opt.minimize(func,x0=x0,bounds=bounds,constraints=constraints,method=method,jac=jacob,hess=hessi)
        print(res)
        resx = res.x
    return resx[:N], resx[N:2*N], resx[2*N:]