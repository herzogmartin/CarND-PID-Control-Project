# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

# Reflection of implementing PID-control
After PID was working in principle manual initial controler weights were chosen. 
Afterwards twiddle was used to optimize the weights. The simulator is automatically
reseted if the vehicle leaves the track. The weights are only used if the driven 
distance is long enough and the error of the summed absolute cross track error (cte)
is getting smaller.


## Initial P-Part
Kp is always steering towards the center. A high Kp is reaching faster the center of 
the track but results in overshooting the middle. A start value of 0.1 was chosen.

## Initial D-Part
Kd is damping the oscillation of the controler. This result in a smoother approach
of the middle position and less overshooting. A start value of 1.0 was chosen.

## Initial I-Part
Ki is responsible to eliminate the static control error. But the I-part tends to
make the controler unstable.

For steering PID a start value of 0 was chosen. The value was chosen by twiddle.

For speed PID-control a value of 0.005 was tested and sufficent to hold the speed. 
The speed control was not optimized any more but this could also be done with twiddle.

## Optimization with twiddle
If the `#define TWIDDLE_ON` statement is set to `1` twiddle optimization is done for steering PID.
In the final release it is set to `0`.

Twiddle was performed with 60 mph. For the finial release the target
speed is set to 50 mph. This can be changed with `#define TARGET_SPEED`.

## Final controler weights 
The following final controler weights are chosen with twiddle:

* Kp = 0.116
* Kd = 1.86086
* Ki = 0.00252475

## Outlook
It would be possible to change the target speed depending on the steering angle.
From the steering angle, the speed and the wheel base the lateral acceleration of the vehicle can
be calculated. The the target speed could be limited so that the lateral acceleration
is not too high.

If this would be done the speed PID needs to do more and its controler weights
should be optimized (e.g. with twiddle).
 