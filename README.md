

# CarND-Controls-PID


In this project, we'll implement a PID controller in C++ to maneuver the vehicle around the track!

The simulator [here](https://github.com/udacity/self-driving-car-sim/releases) will provide the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

> The speed limit has been increased from 30 mph to 100 mph. Try to drive SAFELY as fast as possible!

---



## Reflection

I used  960x720   screen solution, with a graphic quality of fastest in macOS Sierra Version 0.12.3  Macbook Pro.

Here is the effect of PID controller influenced by 3 parameters(P, I, D)

**P (proportional control)**

* I found that the parameter Kp determines how fast my car will turn. So the higher the p-error coefficient is, the sharper will be returned. I found my best Kp value is 0.76.


**D (derivative control)** 

* The D controller is used to dampen the situation when my car is getting overshoot or oscillating through the whole track. Without changing my p coefficient (the best value I tuned when having not set D and I control yet), I gradually increased my Kd parameter and found that when the Kd was at 36, the combination of P and D control could result in a relatively smooth round. 

**I (integral control)**

* the parameter Ki is used to fix a bias and in this case, it is much more likely to compensate for the drift of car steering. With the value of 0.00094 I found my car could reach out to a smoother ride through the whole track. Also, I found that the Ki should be extremely small relative to Kp, otherwise the car will go out of the track. 


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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


```python

```
