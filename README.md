# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

This repository utilizes PID controls to allow a vehicle to drive around a test track. The inputs to the vehicle are steering angle and throttle. They are implemented towards Udacity's Self-Driving Car's Term 2 Simulator.

[//]: # (Image References)

[image1]: ./images/params_4.gif "Kp_3e-2"
[image2]: ./images/params_6.gif "Kp_9e-2"
[image3]: ./images/params_10.gif "Kd_0"
[image4]: ./images/params_9.gif "Kd_5"

## PID Controls

The PID controls used here takes a target input and calculates a response such that the system tries to reach/maintain the target input over future periods.

### 1 Steering Angle

Input - Cross-track-error, Output - Steering angle

```
void PID::UpdateError(double cte) {
  d_error_  = cte - p_error_;
  p_error_  = cte;
  i_error_ += cte;
}
```


### 2 Throttle Control

Input - Difference between current speed and target speed, Output - Throttle

```
double ThrottleControl::GetResponse() const {
  return -Kp_ * p_error_ -Kd_ * d_error_;
}
```

## Parameter Tuning

As the simulator has no systematic bias, we used a Ki = 0 for both the Cross-Track-Error (CTE) controller, and the Throttle controller (TC). The table below shows the parameters and results of the tuning. The last column "AccumError" is the total accumulated CTE as of time step 800. The table below can be read in order, each subsequent row has an incremental change from the previous best model, if the change results in a worse `AccumError`, we try the change in the other direction. When it's sufficiently stable, we move on to the next parameter. The first row's parameters were chosen arbitrarily.

| Id   |  CTE-Kp  | CTE-Kd | TC-Kp | TC-Kd  |  AccumError |
|:----:|:--------:|:------:|:-----:|:------:|:-----------:|
|  1   |  0.05    |    5   |  0.2  |   1    | 1103.96     |
|  2   |  0.05    |    1   |  0.2  |   1    | 1029.12     |
|  3   |  0.05    |    2   |  0.2  |   1    |  984.94     |
|  4   |  0.03    |    2   |  0.2  |   1    | 1441.34     |
|  5   |  0.07    |    2   |  0.2  |   1    |  719.71     |
|  6   |  0.09    |    2   |  0.2  |   1    |  581.99     |
|  7   |  0.12    |    2   |  0.2  |   1    |  433.38     |
|  8   |  0.15    |    2   |  0.2  |   1    |  361.95     |
|  9   |  0.15    |    5   |  0.2  |   1    |  485.26     |
| 10   |  0.15    |    0   |  0.2  |   1    |  N/A*       |
| 11   |  0.15    |    2   |  0    |   1    |  N/A*       |
| 12   |  0.15    |    2   |  0.5  |   1    |  359.97     |
| 13   |  0.15    |    2   |  0.1  |   1    |  375.87     |
| 14   |  0.15    |    2   |  0.2  |   0    |  358.89     |

*N/A indicates failure to complete the track


## Reflection

For both PID systems, we decided to use a manual tuning process. Incrementally changing one parameter in either direction at a time. We used total accumulated CTE as our error measure in comparing the parameter's effectiveness in steering our vehicle.

The key reason we decided in applying a manual tuning parameter is that it wasn't straightforward for us to apply automated parameter optimization (such as twiddle) since running the simulator iteratively was unwiedly. Besides, manual tuning provided us insight on how the vehicle steering smoothness change.

### PID Parameter Effect

#### CTE Model

For the CTE model, our intuition was that the P control would move the car towards the middle of the lane. A higher Kp would move the car towards centre faster. This indeed agrees with our observation, as can be seen in the comparison between (4) and (6).

Kp = 0.03

![alt text][image1] 

Kp = 0.09

![alt text][image2]

Where the car with lower Kp tends to be closer to the right side of the lane during a left curve, indicating that there's insufficient correction steering.

Next, we investigate the effect of Kd, given that `d_error` in our model is defined as

```
d_error = difference in CTE between t and t-1
``` 
This can be interpreted as a term that corrects for overshooting from the impact of Kp. Comparing between (9) and (10), it's clear that Kd has the effect of stabilizing the car once it reaches the middle of the lane due to Kp correction.

Kd = 0

![alt text][image3]

Kd = 5

![alt text][image4]

#### Throttle Control Model

Whereas for the throttle control (TC), we set the target speed to be 40. As such, Kp error for the throttle control is the difference from current car speed to 40. One important thing to note is that the throttle response was floored at 0 before being fed to the simulator, this allows us to avoid excessive braking on the vehicle.

This throttle control is crucial because if we apply a constant throttle, then the vehicle will simply accelerate all the time, and as can be seen from (11), it causes the car to not complete the track. On the other hand, we notice that while it's important to have a TC-Kp, `AccumError` itself was not particularly sensitive to its value, as seen between (8), (12), and (13). Finally, from (14), we learned that TC-Kd is not material either. 

The final parameter used is model (14),

| CTE  |   Value  |
|:----:|:--------:|
| Kp   |    0.15  |
| Ki   | Not used |
| Kd   |    2     |

| TC   |  Value   |
|:----:|:--------:|
|   Kp |    0.2   |
|   Ki | Not used |
|   Kd | Not used |


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

