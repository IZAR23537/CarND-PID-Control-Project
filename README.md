# CarND-PID-Control-Project
Self-Driving Car Engineer Nanodegree Program

## Overwiew

In this project we'll revisit the lake race track from the Behavioral Cloning Project. 
This time, however, we'll implement a PID controller in C++ to maneuver the vehicle around the track!
The simulator will provide us the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Project Rubic points

### Your code should compile.

The code compiled without errors. No modification were made on the initial setup.

### The PID procedure follows what was taught in the lessons.

The implementation of the PID is done in the CCP.ccp file, where in the PID::UpdateError method calculates the proportional, integral and derivative errors and the PID::TotalError calculates the total error with the coefficients.

### Describe the effect each of the P, I, D components had in your implementation.

P - The proportional factor deals with how far are we from the center of the road.

I - The integral factor deals with the systematic bias as the wheels are not alligned. 

D - The derivative factor deals with oscillation around the center line. 

### Describe how the final hyperparameters were chosen.

I chose the final hyperparameters of the P.I.D controller by tuning it manually.
First I tried to tune the Kp value alone around -1.0. In the straight road it controlled the car around the center, but as the vehicle moved to a turn, It wiggled out of the road.
I tuned the Kp value to -0.1, changed the Kd value to -1.0 and I set the Ki value to 0.0 and left unchanged during the tuning process, because I expected and found out that the wheels are alligned.
As I tuned the Kd value further the car started to move much more smoothly and the car finally drove around the track.
In the and I chose the following value for Kp -0.15 and for Kd -2.7.

### The vehicle must successfully drive a lap around the track.

After tuning the Kp, Ki, Kd values, the car successfully drove around the track.

