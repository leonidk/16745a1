# Inverse Kinematics through Optimization
[CMU RI 16-745: Dynamic Optimization: Assignment 1](http://www.cs.cmu.edu/~cga/dynopt/ass1/), from Leonid Keselman and Alex Spitzer

## Part 1
We implemented the solve function following the requested template (see **solve()** in **iksolver.py**). This uses  **scipy.optimize.minimize** to set an **N** link arm of given lengths to a given target position. Optimization is performed using [Euler Angles](https://en.wikipedia.org/wiki/Euler_angles), which means solver is susceptible to gimbal lock. Roll, pitch and yaw have a hard inequality constraint for limits of [-pi,pi],[-pi,pi],[-pi/2,pi/2] respectively. while obstacle avoidance is done with a soft penalty for every time a link segment intersects any given obstacle. We minimize squared distance from the target point, along with a [0,1] penalty for the arcosine of the rotation error. 

Some results can be seen below for different arms of different lengths, with different target positions, different obstacles, etc. 

## Part 2
We implemented computation of a Jacobian via the Python [autograd](https://github.com/HIPS/autograd) package, which implements [reverse mode differentiation](https://en.wikipedia.org/wiki/Automatic_differentiation#Reverse_accumulation). This can be seen enabled in our script by using the `--grad` option. We include the ability to compute the hessian as well with the `--hess` option but in most of our test cases, the Hessian leads to unstable steps. 

Some results from the improved solver can be seen below

## Part 3
We included the Python CMA package, which uses CMA-ES. We set sigma to pi/2.0 for all dimensions. The Python optimization package, much like the MATLAB one, includes many potential solvers. Some do not support bounds, some do not support hard constraints (hence the use of the soft constrained penalties for object intersection). The fixed solvers that we ended up including (due to fairly good convergence and wide parameter support) were 'Powell', 'SLSQP', 'COBYLA', 'CMA', 'L-BFGS-B'. We additionally added Python’s differential evolution optimizer, which like CMA, is derivative-free and worked fairly well. 

Some of our reports results are below

## Part 4
We can find multiple local minima by initializing different solvers with different random initial conditions. We can then report multiple converged results, sorted by our fitness function. Some examples of multiple solutions returned and sorted can be seen below. 

## Usage
We're using Python 3.6, and the python packages *numpy*, *matplotlib*, *cma*, *autograd* and *transforms3d*. The first four can be installed with `pip install` while the last one already is included locally. 

See `python ik.py --help` for information about how to run the optimizer (set target position, select method, set obstacles, etc.) 
```
usage: ik.py [-h] [--links LINKS [LINKS ...]] [--target TARGET [TARGET ...]]
             [--method {Powell,SLSQP,COBYLA,CMA,L-BFGS-B,diffev}] [--grad]
             [--hess] [--obs [OBS [OBS ...]]] [--random]

Solve ik for N joint arm

optional arguments:
  -h, --help            show this help message and exit
  --links LINKS [LINKS ...]
                        link lengths (default: [2, 2, 2])
  --target TARGET [TARGET ...]
                        target position and quat (default: [3, 3, 3, 1, 0, 0,
                        0])
  --method {Powell,SLSQP,COBYLA,CMA,L-BFGS-B,diffev}
                        solver method (default: SLSQP)
  --grad                use the analytical jacobian (default: False)
  --hess                use the analytical hessian (default: False)
  --obs [OBS [OBS ...]]
                        obstacles 4xM values (default: [1, 1, 1, 0.5])
  --random              use random init (default: False)
  ```

## Repository Information
**ik.py** is a command line program demonstrating the use of an optimizer for inverse kinematics. **iksolver.py** contains the solve method, the objective function and bounds setting. **part1.py** and **part2.py** provide the same functionality but without command line arguments, following the more classical structure of the assignment. 

Additionally, we have a **matlab/** folder which contains an alternative implementation for part of the assignment, although we mostly recommend and report results against our Python version documented here. 
