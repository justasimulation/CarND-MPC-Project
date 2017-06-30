# CarND-Controls-MPC
---

## Model

The kinematic model is used.

The state consists of the following:
```
* x - coordinate
* y - coordinate
* v - velocity
* psi - orientation
```

Actuators are:

```
* steering angle - its value is can be read and set directly
* throttle - althogh its value can be read and set it is not clear what it means in this
context and how it is related to acceleration, so some approximation of the relation is needed
```

Transition equations:

```
* x = x + v * cos(psi) * dt
* y = y + v * sin(psi) * dt
* v = v + acceleration * dt
* psi = psi + v * steering_angle / Lf * dt
* orientation
```

Where 
* Lf - is distance between the car from and its center of gravity
* acceleration - real acceleration of the car which is not perceived directly by any device in this set
* dt - time step length


## Units and conversions

* Velocity is provided as miles per hour and converted into meters per second
* Steering angle is limited to [-25 degree; 25 degree] and is perceived in radians,
but when setting the actuator's value it is needed to convert it into [-1; 1] interval.
* Throttle value is limited to [-1; 1]. I change speed and acceleration using throttle,
 but it is not clear how they are related. I use the following approximation: it is assumed
 that the maximum throttle 1 gives us 4 meters per second per second of acceleration, so
  we limit acceleration value by [-4; 4] and map to to throttle interval. This approximation
  is almost always false, but probably because the problem is resolved several times per second
  it still works.
  
  
## Timestep length and elapsed duration (N and dt)

* N - is chosen so the predicted trajectory would be approximately the same size as the target line,
in case of linear motion with constant speed between waypoints. I observed that in case of large N sometimes
predicted trajectory becomes much longer than the target line and the result of optimization looks realy bad.
So the equations is

```
N = waypoints_length / (v * dt)
```
Where v is current speed

* dt - is chosen 0.05 seconds. This part is not clear for me. My idea was to choose dt the same as it is in the simulator,
in other words dt would be a time between setting actuators values. 
On my laptop this time is 0.14-0.2 seconds (including latency emulation), but for some reasons it works
considerably worse than 0.05. Either there is an error or my laptop is too slow for this.


## Polynomial fitting, preprocessing and latency

* First latency is simulated by applying motion equations to the initial state given by telemetry. Also the average execution
time of the method is added to the latency time, so I try try to predict the state at the time right after the method is executed.

```
dt = latency_time + method_execution_time
```

 
* After latency simulation all coordinates are converted into car's coordinate system and 3rd order polynomial is fitted.

## Results

The submitted result works on my laptop at approximately 57 mph. I was able to drive much faster but that looked
a bit unstable and as I use a slow laptop it is always a mystery whether there is an error or my hardware is just not good enough.
So I decided to use a slow more stable version.

I noticed when driving more than 60 mph it is required to calculate curvature and break when curvature to speed ratio is to large, 
using that it is possible to drive more than 100 mph. I guess that dynamic model can automatically handle this curvature situation.


## Reflections

* It is much better than PID for sure.
* Throttle acceleration connection is not clear (even for guys that did QA on this project)
* Sometimes the optimizer shows really weird results (may be it returns some error code about that, need to check)
* Need to control speed curvature ratio (probably dynamic models could do this automatically)



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
