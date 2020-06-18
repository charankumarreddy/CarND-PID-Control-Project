# CarND-Controls-PID Project

The goal for PID controller project is car to drive around a track in a simulator while staying within the lane boundaries. This is accomplished by calculating the steering angle that is proportional to the Cross Track Error (CTE): the lateral distance between the car and the reference trajectory.

## Background

A critical module in the working of any robotic system is the control module. Control module defines the action, which the robotic system performs in order to achieve a task. These actions can vary on type of the system and type of the task. For e.g.: A simple mixer grinder's control module only controls the speed of rotating motor. A little more complex system such as a Remote Controlled (RC) car needs a control module to move forward, backward and turn. Highly complex systems such as prosthetic arms, self driving cars, product manufacturing factory units require control modules for multiple tasks at the same time.

One of the basic implementation of a control system is a Proportional (P), Differential (D), Integral (I), together, a PID controller. PID controller is the most popular controller and is used in applications across domains.

## Working With PID Controller

The basic principle of working of a PID controller is to satisfy a boundary value problem. Most common examples of such problems are either minimization of the total error in the system or maximization of the total gain in the system. These error or gain problems are represented mathematically and are used to govern the effect to P, I and D components of a PID controller. These components are described below:

1. **Proportional (P) component**: Mathematically, the P component establishes linear relationship with the problem. The effect of P component is in proportional to the value of problem at the current time step. 

      The proportional term, when used by itself to calculate the steering angle, sets a steering angle that is proportional to the CTE. However, the end result is a steering angle which oscillates around the reference trajectory. The proportional coefficient (Kp) determines how fast the car oscillates(or overshoots) around the reference trajectory.Plese check the video [here](https://youtu.be/sWJtCcznwfI)

    The value of P component is given by the formula:
    αp = -Kp * error

        where, Kp is a tuning parameter known as the proportional gain. The negative sign in the beginning signifies that P component is used to cancel the effect or error, or in other words, reduce it.


2. **Differential (D) component**: Mathematically, the D component establishes linear relationship with the rate of change of problem. The effect of D component is in proportional to the rate of change problem from last time step to current time step. 

    The derivative component uses a rate of change of error to reduce the overshoot caused by the proportional component. This derivative coefficient (Kd) term is used to optimize how far the car overshoots (also known as oscillation amplitude) from the reference trajectory.Check the car simulator video [here](https://youtu.be/K2X3iTDGzYM)

    The value of D component is given by the formula:
    αd = -Kd * d(error)/dt

        where, Kd is a tuning parameter known as the differential gain. The negative sign in the beginning signifies that D component is used to cancel the effect or rate of change of error, or in other words, reduce it. The D component is used to minimize sudden changes in the system.

3. **Integral (D) component**: Mathematically, the I component establishes linear relationship between the average value of problem over time. The effect of I component is in proportional to the average value of problem from the beginning of time to the current time step. 

    Over time, the steering angle accrues errors due to systematic bias which could drive the car out of the track eventually, but not immediately. The integral component fixes this problem. As this component impacts the error over a period of time, the integral coefficient (Ki) should be carefully optimized in small steps as it has a large impact on the overall performance.Check the car simulator [here](hhttps://youtu.be/B0tgEnbmaDU)

    I component is given by the formula:
    αi = -Ki * ∑ error

        where, Ki is a tuning parameter known as the integral gain. The negative sign in the beginning signifies that I component is used to cancel the effect or average error, or in other words, reduce it. The I component is used to correct systemic bias.

4. **PD Controller**:- In this step, a PD controller was used. The I component was still switched off by setting the value of Ki to zero. 

As seen, the car was able to stay on the track and drive successfully for most of the portion. However, it was observed that the car wouldn't stay in the center of the lane and often drift to the edges. This resulted in very sharp turns which is certainly not desirable in case a human was sitting inside the car. 

Check the video from [here](https://youtu.be/eBpEHj1A7JY)

5. **PID Controller**

***PID formula***

![PID formula](/images/pid_formula.png)

or

![PID Udacity formula](/images/PID_Udacity_formula.png)

where Kp(Jp),Ki(Ji) and Kd(Jd) all non-negative, denote the coefficients for the proportional, integral, and derivative terms respectively (sometimes denoted P, I, and D).

Plese check PID video [here](https://youtu.be/sWJtCcznwfI)


### How to tune the parameters

The parameters are tuned manually with the order of: p, d, i. The d and i are first setted to be zeros, and 0.15 is used for the p value. I adjust the p value up and down till it could drive around the first corner and hard to imporve more. Then I keep the p value as it is, and increase the d value. Use the same approach for d value and i value.

In order to automatically fine tune the parameters, an optimization algorithm twiddle can be used

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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

