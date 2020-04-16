# **PID Control**


In this project the goal is to safely navigate around a virtual track using a PID control to steer the wheels of the car. The main points are:
* The PID procedure follows what was taught in the lessons.
* The vehicle must successfully drive a lap around the track.

---
### Files Submitted & Usage

##### Files

My project includes the following source files:
* **src/json.hpp** helpful to work with json data
* **src/PID.h** and **src/PID.cpp** contains a library to work with splines
* **src/main.cpp** contains the execution flow

And some extra files:
* **CMakeLists.txt** to build the project using cmake
* **README.md** the readme of the original project from Udacity
* **writeup.md** this file with the results

##### Usage

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.


---
### Strategy


##### Parameters definition

###### P - Proportional
It is the gain applied directly to the error. It brings the car back to the middle of the road. The effect in differents values is:
* Low value: it will take long time for the car to reach the centre of the road. It will overshot (if d=0) but it will get oscillations smaller every iteration.
* Middle value: the car recover from the perturbation and produce stable oscillations forever.
* High value: the car will get unstable with oscillations getting bigger every time.

###### D - Derivative
It is the gain applied to the derivate of the error. It controls the oscillation of the car subtracting control power when the tendency of the error is taking it to zero. The values can be:
* Low: the car will still oscillate.
* Middle: the car is critically damped, i.e, just the value when the car stops oscillating.
* High: it will stop the car from reaching the 0 error fast enough.

###### I - Integral
It is the gain applied to the integral of the error. It adjust the control to overcome the bias in the plant. It can produce problems if we integrate over long time and the suddenly the car recovers so I applied a technique that resets this integration when the error cross the zero. Usually it gets a small value related to the other constants because it is applied to an integral.



##### Tune the parameters
I decided to tune the parameters manually following the steps found in this forum(https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops):
1. Set all gains to zero.
2. Increase the P gain until the response to a disturbance is steady oscillation.
3. Increase the D gain until the the oscillations go away (i.e. it's critically damped).
4. Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
5. Set P and D to the last stable values.
6. Increase the I gain until it brings you to the setpoint with the number of oscillations desired (normally zero but a quicker response can be had if you don't mind a couple oscillations of overshoot)


The final parameters for my car have been Kp=0.12, Ki=0.001, Kd=1.5.


---
### Implementation
The implementation of the PID is straightforward.
```c++
class PID {
 public:
   PID::PID() {}
   PID::~PID() {}

   void PID::Init(double Kp_, double Ki_, double Kd_)
   {
      Kp = Kp_;
      Ki = Ki_;
      Kd = Kd_;
      p_error = 0;
      i_error = 0;
      d_error = 0;
   }

   void PID::UpdateError(double cte) {
      d_error = cte - p_error;
      if (cte*p_error < 0)
        i_error = 0;     // zero crossing reset
      i_error += cte;
      p_error = cte;
   }

   double PID::TotalError() {
     return -Kp*p_error-Ki*i_error-Kd*d_error;  // TODO: Add your total error calc here!
   }

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */
  double Kp;
  double Ki;
  double Kd;
};
```



---
##### Video

<a href="http://www.youtube.com/watch?feature=player_embedded&v=s9SKrT7sGwI
"><img src="http://img.youtube.com/vi/s9SKrT7sGwI/0.jpg"
alt="Path Planning in highway" border="10" /></a>


---
### Reflection

PID is a very powerful and simple control algorithm, that makes it widely used almost in every applications. It has some drawback like parameter tuning.
