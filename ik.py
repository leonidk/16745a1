#!/usr/bin/env python3
import argparse
import iksolver as ik

def chunks(l, n):
    """Yield successive n-sized chunks from l."""
    for i in range(0, len(l), n):
        yield l[i:i + n]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Solve ik for N joint arm',formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--links', nargs='+', help="link lengths",default=[2,2,2])
    parser.add_argument('--target', nargs='+', help="target position and quat",default=[3,3,3, 1,0,0,0])
    parser.add_argument('--method', help="solver method", choices=['Powell', 'SLSQP', 'COBYLA', 'CMA','L-BFGS-B','diffev'],default='SLSQP')
    parser.add_argument('--grad', dest='grad', action='store_true',help='use the analytical jacobian')
    parser.add_argument('--hess', dest='hess', action='store_true',help='use the analytical hessian')
    parser.add_argument('--obs', nargs='*', help="obstacles 4xM values",default=[1,1,1, 0.5])
    parser.add_argument('--random', dest='random', action='store_true',help='use random init')
    parser.add_argument('--img', dest='img', action='store_true',help='save image')

    parser.set_defaults(grad=False)
    parser.set_defaults(hess=False)
    parser.set_defaults(random=False)
    parser.set_defaults(img=False)

    args = parser.parse_args()
    N = len(args.links)
    pi = 3.14159
    link_lengths = [int(x) for x in args.links] #[2 for _ in range(N)]
    min_roll     = [-pi for _ in range(N)]
    max_roll     = [+pi for _ in range(N)]
    min_pitch    = [-pi for _ in range(N)]
    max_pitch    = [+pi for _ in range(N)]
    min_yaw      = [-pi/2.0 for _ in range(N)]
    max_yaw      = [+pi/2.0 for _ in range(N)]
    obstacles    = list(chunks([float(x) for x in args.obs],4))

    target = [float(x) for x in args.target]
    
    ik.method = args.method
    ik.use_grad = args.grad
    ik.use_hess = args.hess
    ik.initial_x = [0.0 for _ in range(N*3)]
    if args.random:
        import random
        ik.initial_x = [random.uniform(-pi,pi) for _ in range(N)] + [random.uniform(-pi,pi) for _ in range(N)] + [random.uniform(-pi/2.0,pi/2.0) for _ in range(N)] 
 
    res = ik.solve(target,link_lengths,min_roll,max_roll,min_pitch,max_pitch,min_yaw,max_yaw,obstacles)

    import matplotlib as mpl
    mpl.use('Qt5Agg')
    mpl.rcParams['legend.fontsize'] = 10

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import numpy as np

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
        pos,qM = ik.fwd(pos,l,r,p,y,qM)
        ax.plot([pos0[0],pos[0]], [pos0[1],pos[1]],[pos0[2],pos[2]])
    err = np.sqrt(((np.array(pos) - np.array(target[:3]))**2).sum())
    # plot spheres
    for o in obstacles:
        u = np.linspace(0, 2 * np.pi, 10)
        v = np.linspace(0, np.pi, 10)
        x = o[3] * np.outer(np.cos(u), np.sin(v)) + o[0]
        y = o[3] * np.outer(np.sin(u), np.sin(v)) + o[1]
        z = o[3] * np.outer(np.ones(np.size(u)), np.cos(v)) + o[2]
        ax.plot_surface(x, y, z, color='b')

    # plot goal
    ax.scatter(target[0], target[1], target[2], c='r',label='target')

    ax.set_title('{0} of {1} links w/ sq. error of {2:.2f}'.format(args.method,N,err))
    ax.legend(loc=4)
    ax.set_xlim(0,5)
    ax.set_ylim(0,5)
    ax.set_zlim(0,5)
    if args.img:
        import time
        seconds = time.time()
        plt.savefig('img{}.png'.format(int(seconds%(3600*24))), bbox_inches='tight')
    plt.show()
