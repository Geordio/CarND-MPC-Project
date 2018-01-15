# CarND-Controls-MPC
This document forms the report of my submission for the Model Predictive Control Project of the Udacity Self-Driving Car Engineer Nanodegree Program

## Compilation

The solution can be compiled by executing 'cmake .. && make' in the build directory.

## The Model

The vehicle is represented by the following states: X position, Y position, orientation  (angle), and velocity

The model includes the following actuators: Steering, and Throttle (actually throtlle and brake combined)

In addition to these, we also conside the errors: Cross Track Error (The distance from the required path, and Psi Error (The error from the required orientation)

The update equations are shown below:

```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt

Where:
x = x poistion
y = y position
psi = vehicle orientation
v = velocity
dt = time step
Lf = distance between the front of the vehicle and the centre of gravity
```

## Timestep Length and Elapsed Duration

I initially based my Timestep Length and Elapsed Duration on the parameters used in the lesson Quiz, i.e N=25, dt = 0.05, resulting in a 1.25 second prediction ahead.
I found that this performed acceptably at lower speeds, but at higher speeds I was concerned with the computing overhead, as it occasionally behaved erratically, so I changed this to N=12.5, and dt =0.1. This gives the same overal prediction time, but requires less computing resource. However, when I raised the speed further to 70, further issues occured, as shown by the screenshot below, resulting in a crash.

<img src="https://github.com/Geordio/CarND-MPC-Project/blob/master/images/crash.png" alt="Crash" width="400" height="400"/>

I limited my trials of dt to 0.05 and 0.1, as the latency of the vehicle actuations is 0.1s, and my solution to the latency problem requires that the dt is a factor of the latency time.

## Polynomial fitting

I fitted a polynomial to the waypoints by calling the polyfit method, passing the vector represenations of the x and y coordinates of the waypoints, and the order of polynomial to fit, in this instance a third order polynomial is used as it can represent waypoints of a vehicle well.

## Preprocessing

Prior to fitting the polynomial, I converted the way points from the map corodinate system, to the vehicle coordinates. I.e the position and orientation of the waypoints being relative to the vehicle location.
This is done with the following code, based on trigonomtery


```cpp
for (int i = 0; i < no_waypoints; i++) {
  // calcualte the x and y positions of the waypoint relative to the vehicle position
  double relx = ptsx[i] - px;
  double rely = ptsy[i] - py;

  waypointsx_veh[i] = relx * cos(-psi) - rely * sin(-psi);
  waypointsy_veh[i] = relx * sin(-psi) + rely * cos(-psi);
}
```

## Latency

The MPC hanfles the 100 ms latency successfully. At low speeds this latency makes no significant difference.

My solution to handle the latency is for the MPC to return the predicted actuations for the step 100ms in the future. I.e the model calculates the steps as normal, but does not return the initial step, but the approproate future one. This has some flaws, i.e the future step is based on the assumption that the initial step took place successfully. Hence if the vehicle did not perform the first step, then the predicted future steps and not likely to be as accurate.

## Tuning

Tuning the MPC took considerable time and effort. This was mainly based on experimentation and trial and error (there must be a better way to do this activity). I created a weight factor for the cte, psi error, velocity error, steering, speed, and rate of change of both the steering and speed. It was easy to establish values that meant that the vehicle could complete the majority of the track at 40mph, but above this it took a lot of iterations to get to useable values.


## Simulation

Below is a gif showing the model successfully negotiating a section of the track.

I was able to set the speed to 70mph (UK speed limit) and sucessfully complete multiple laps.

<img src="https://github.com/Geordio/CarND-MPC-Project/blob/master/images/working.gif" alt="Crash" width="341" height="250"/>

# Other

Below is the original readme for completeness, including how to set up the project.

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
