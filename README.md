# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


## Implementation

The implementation considers two PID controllers: one for the steering and one for the throttle. Both controllers share the same code but differ on the gains. They are based on the standard representation for PID controllers, as seen in the lessons.

The output of the controllers is limited to values between [-1,1] to avoid sending dangerous control actions.

An anti-windup technique deals with the integral component accumulating too large values. It won't contribute to the output if the control action exceeds the limits,  but it will keep collecting error. That is particularly useful when the controller setpoint is still distant.

The target speed decreases whenever the cross-track-error exceeds a set limit, easing driving on curved parts of the road. The controller will always attempt to go at 30 mph, but the speed may reduce up to 20 mph.

## Reflection

This section describes the effect of each PID component and how the gains were selected.

### Components

**P - Proportional Component**: This action is proportional to the error, and is responsible for setting the plant near the setpoint quickly. It is directly related to the control speed. By using a big gain, the plant may oscillate, while too small values may not even have any effect.

**I - Integral Component**: This action is proportional to the accumulated error over time. Value increases slowly. It is responsible for setting the steady-state error to zero.

**D - Derivative Component**: This action is proportional to the rate of change of the error. It is responsible for adjusting the control action when the error changes quickly, allowing a faster response.

### Tuning of PID gains

Both controllers are tuned using the same procedure.  I started by the throttle controller because having a constant speed makes tuning the steering controller easier.

1. Deactivate the control action limits to avoid hiding the effect of the gains.
2. Increase the P gain until the system starts oscillating and is still stable.
3. Increase the D gain (small values) and verify the oscillations decrease.
4. Increase the I gain to compensate for the steady error.
5. Test on different setpoints: 20mph-50mph, straight and curved road.
6. Introduce perturbations to the plant by manually adding error on the simulator (increase/decrease speed, force steering to the sides), and make sure the controllers can deal with that.
