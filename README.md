# Model Predictive Control for Path Tracking

In this project MPC controller has been implemented to manuever the vehicle around the lake race track. Unlike the PID control project where cross track error (CTE) is given by simulator, it has to be calculated by ourselves using a kinematic model and vehicle co-ordinates. Additionally, there's a latency of 100 millisecond between actuation commands on top of connection latency.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Kinematic Model

Model predictive control uses an optimizer to determine the control inputs and minimiza the cost function. The MPC setup requires definition of a kinematic model of the system, constraints, cost function and length of prediction horizon. As we are dealing with a vehicle system that tracks a desired path, we use simple bicycle kinematic model assuming that the car is symmetric about longitudinal axis. The model equations are:

```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt

y[t+1] = y[t] + v[t] * sin(psi[t]) * dt

psi[t+1] = psi[t] + v[t]/Lf * delta[t] * dt

v[t+1] = v[t] + a[t] * dt

cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt

epsi[t+1] = psi[t] - psi_des + v[t]/Lf * delta[t] * dt
```

The model has 6 states which include:

* x position (x)

* y position (y)

* yaw angle (psi)

* velocity (v)

* cross track error (cte)

* yaw angle error (epsi)


The model also has two actuator inputs which include:

* steering angle (delta)

* acceleration (a)

## Choosing N and dt

T is the prediction horizon over which future predictions are made to determine the optimal control inputs. It is the product of two variables, N and dt. N is the number of timesteps in the horizon and dt is how much time elapses between actuations. Larger T makes the control inputs more stable, however having smaller dt at the same time makes the control inputs more accurate but leads to slower controller performance and sometimes make the vehicle go off the track. Also, T must not be choosen too large as the future might not be same as we expect it and change due to disturbances from surroundings. Several combinations of N (20, 15, 10) and dt (0.1, 0.05, 0.07) have been tried out to find the values that minize the cost. The best that worked for me is N = 10 and dt = 0.1 sec. It must be noted that tuning of N and dt was done simultaneously with the tuning of cost function weights.

## Polynomial Fitting and MPC Preprocessing

The way points provided by the simulator are in global frame and have to transformed to local frame. This transformation is done by using vehicle's position co-ordinates and orientation. These way points are fitted to a 3rd degree polynomial as the track includes several turnings, whose curvature can be best defined by a higher degree polynomial. The coefficients of the polynomial obtained from curve fitting are then used to calculate the cross track error (cte) and yaw angle error (epsi) based on the vehicle's position. 

## Model Predictive Control with Latency

Latency of 100sec is included between actuation commands which is usually the case in a real car. This can be explicitly taken into account by an MPC algorithm. This is dealt by including a kinematic vehicle model that simulates from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC. 

## Result

The vehicle drives around the track safely after tuning the weights associated with the cost function and parameters of the  prediction horizon. The maximum speed reached by the vehicle during simulation is 40 mph.




