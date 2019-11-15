# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## The Project

In this project a PID controller is to be implemented to maneuver the vehicle around the track in the simulator.

The simulator provides the cross track error (CTE) and the velocity (mph) and current steering angle. This data is used to compute the appropriate steering angle and throttle.

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Project Rubric

### Cross Track Error
A cross track error is a distance of the vehicle from trajectory. In theory it’s best suited to control the car by steering in proportion to Cross Track Error(CTE).

#### P component
It sets the steering angle in proportion to CTE with a proportional factor tau. This causes car to steer in proportional to the error in the opposite direction of the error.
```
-tau * cte
```

#### I Component:
It’s the integral or sum of error to deal with systematic biases.
```
int_cte += cte
tau_i * int_cte
```

#### D component:
It’s the differential component of the controller which helps to take temporal derivative of error. This means when the car turned enough to reduce the error, it will help not to overshoot through the x axis.
```
diff_cte = cte - prev_cte
prev_cte = cte
- tau_d * diff_cte
```

Total Error is calculated as below formula. Basically summing up all the errors.
```
(-tau_p * cte) - (tau_i * int_cte) - (tau_d * diff_cte)
```

Kp, Ki & Kd for PID class are initialized in PID::Init() along with resetting the individual errors.

```
void PID::Init(double Kp_, double Ki_, double Kd_) {
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	p_error = 0;
	i_error = 0;
	d_error = 0;
}
```

Error are updated at each step as per below code:

```
void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}
```

```
double PID::TotalError() {
  return (-Kp * p_error) - (Ki * i_error) - (Kd * d_error);
}
```

Initially, the hyperparameters were chosen randomly with trial and error method to see if car can drive with less oscillation.
Initial hyperparameters: (Kp,Ki,Kd) = (0.135, 0.0002, 3.0)

[![Watch the video](https://img.youtube.com/vi/0C3fenE10Zo/hqdefault.jpg)](https://youtu.be/0C3fenE10Zo)

After this I ran the twiddle algorithm to fine tune the gains. After around 400 interations, final hyperparameters were chosen.
Final hyperparameters: (Kp,Ki,Kd) = (0.152937, 7.89867e-05, 3.15709)

[![Watch the video](https://img.youtube.com/vi/7CV27ZNB9-o/hqdefault.jpg)](https://youtu.be/7CV27ZNB9-o)


