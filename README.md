# Inverse Kinematics through Optimization
[CMU RI 16-745: Dynamic Optimization: Assignment 1](http://www.cs.cmu.edu/~cga/dynopt/ass1/)

Leonid Keselman and Alex Spitzer

## Part 1
We implemented the solve function following the requested template (see **solve()** in **iksolver.py**). This uses  **scipy.optimize.minimize** to set an **N** link arm of given lengths to a given target position. Optimization is performed using [Euler Angles](https://en.wikipedia.org/wiki/Euler_angles), which means solver is susceptible to gimbal lock. Roll, pitch and yaw have a hard inequality constraint for limits of [-pi,pi],[-pi,pi],[-pi/2,pi/2] respectively. while obstacle avoidance is done with a soft penalty for every time a link segment intersects any given obstacle. We minimize squared distance from the target point, along with a [0,1] penalty for the arccosine of the rotation error.

Some results can be seen below for different arms of different lengths, with different target positions, different obstacles, etc.

<img src="img/img20844.png?raw=true" width="300"> <img src="img/img20949.png?raw=true" width="300"> 
<img src="img/img21015.png?raw=true" width="300"> <img src="img/img20862.png?raw=true" width="300"> 


## Part 2
We implemented computation of a Jacobian via the Python [autograd](https://github.com/HIPS/autograd) package, which implements [reverse mode differentiation](https://en.wikipedia.org/wiki/Automatic_differentiation#Reverse_accumulation). This can be seen enabled in our script by using the `--grad` option. We include the ability to compute the hessian as well with the `--hess` option but in most of our test cases, the Hessian leads to unstable steps. 

Some results from the improved solver can be seen below, from the results & situations shown above. All results converged identically well.

| Simulation  | # of Func Eval  (numerical) | # of Func Eval (analytical) |
| ------------- | ------------- |------------- |
| 3 link  | 871  | 112 | 
| 6 link#1  | 480  | 66  | 
| 6 link#2  | 398  | 49  | 
| 7 link  | 629  | 79  | 


## Part 3
We included the Python CMA package, which uses CMA-ES. We set sigma to pi/2.0 for all dimensions. The Python optimization package, much like the MATLAB one, includes many potential solvers. Some do not support bounds, some do not support hard constraints (hence the use of the soft constrained penalties for object intersection). The fixed solvers that we ended up including (due to fairly good convergence and wide parameter support) were 'Powell', 'SLSQP', 'COBYLA', 'CMA', 'L-BFGS-B'. We additionally added Python’s differential evolution optimizer, which like CMA, is derivative-free and worked fairly well. 

Some of our results are below for challenging simulations which our initial solver (SLSQP, first row) struggles with. Some solvers give results with different issues when they don't completely converge. For example, in the second simulation shown here, the [Powell](https://en.wikipedia.org/wiki/Powell%27s_method) solver generates a solution through the sphere due to the use of soft constraints, put places the end effector accurately. On the other hand SLSQP avoids the sphere, but fails to place the end effector as accurately. 

<img src="img/f1.png?raw=true" width="300"> <img src="img/s2.png?raw=true" width="300">
<img src="img/c1.png?raw=true" width="300"> <img src="img/p1.png?raw=true" width="300">
<img src="img/p2.png?raw=true" width="300"> <img src="img/c2.png?raw=true" width="300">

## Part 4
We can find multiple local minima by initializing different solvers with different random initial conditions. We can then report multiple converged results, sorted by our fitness function. Some examples of multiple solutions returned and sorted can be seen below. Sim uses `--links 2 3 1 3 2 --target 5 3 2  1 0 0 0 --obs 2 2 2 0.75 2 4 2 0.5 4 2 2 0.5 2 2 4 0.5`

<img src="img/img23229.png?raw=true" width="300"> <img src="img/img23258.png?raw=true" width="300">
<img src="img/img23239.png?raw=true" width="300"> <img src="img/img23246.png?raw=true" width="300"> 
<img src="img/img23192.png?raw=true" width="300"> <img src="img/img23264.png?raw=true" width="300">

## Additional Analysis
We observed that many of these solvers are sensitive to initial conditions. For example, first order methods (with the exception of Powell) all struggled to produce a 180 flip of the arm. However, the derivative-free methods, such as CMA-ES and differential evolution, were able to correctly discover adequate solutions. This suggests that many optimization scenarios would benefit from multiple initial seeds to help discover an adequate solution. Following the paper [Random Search for Hyper-Parameter Optimization](http://www.jmlr.org/papers/volume13/bergstra12a/bergstra12a.pdf), we prefer random initialization and optimization over grid search as it (often) can be more sample efficient. In the cited paper, it even can outperform sequential optimization methods that do meta-learning in certain conditions. Thus, as the derivative free methods sample their functional evaluations in a stochastic (and eventually structured) way, they seem to avoid getting stuck in inadequate local minima. 

Our experiments with inverse kinematics falling into local minima that are inadequate suggest that future work should explore alternative parameterizations of angles (for example, by using gimbal-lock free forms such as exponential maps). Additionally the Hessian generated by auto-differentiation system was almost always unusable, and never lead to improved performance. 

Overall, we were able to test many strategies (random restart & initializations, or using a method like CMA) which lead to viable solutions of inverse kinematics problems, with a range of target positions and obstacles, by use of optimization methods. To enable a wider variety of methods, we tended to prefer using a soft constraint for the obstacles, as it allowed larger families of solvers (namely derivative free methods) to operate correctly. Satisfying hard constraints would have required gradients relative to these constraints (see: [KKT conditions](https://en.wikipedia.org/wiki/Karush%E2%80%93Kuhn%E2%80%93Tucker_conditions)), and rendered a very useful class of solvers unsuitable for this problem.  

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
### Examples
`python ik.py --links 3 3 3 3 3 3 --target 2 5 9 0.707 0 0.707 0 --obs 2 7 2 7 --method CMA`

`python ik.py --links 2 2 2 --target 3 3 3 1 0 0 0 --obs 1 1 1 0.5`

`python ik.py --links 2 2  --target -4 0 0 1 0 0 0 --obs --method CMA`

`python ik.py --links 1 2 3 5 3 2 1  --target 4 5 4 1 0 0 0 --obs 2 2 2 2`

## Repository Information
**ik.py** is a command line program demonstrating the use of an optimizer for inverse kinematics. **iksolver.py** contains the solve method, the objective function and bounds setting. **part1.py** and **part2.py** provide the same functionality but without command line arguments, following the more classical structure of the assignment.

Additionally, we have a **matlab/** folder which contains an alternative implementation for part of the assignment, although we mostly recommend and report results against our Python version documented here. 
