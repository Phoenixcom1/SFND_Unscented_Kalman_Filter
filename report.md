#Unscented Kalman Filter Highway Project

##Rubic
###Compiling and Testing

>Criteria:
>
>The submission must compile.

>Meets specifications
>
>The project code must compile without errors using cmake and make.

###Code Efficiency

>Criteria:
>
>The methods in the code should avoid unnecessary calculations.

>Meets specifications
>
>Your code does not need to sacrifice comprehension, stability, or robustness for speed. However, you should maintain good and efficient coding practices when writing your functions.
>
> Here are some things to avoid. This is not a complete list, but there are a few examples of inefficiencies.
>
> * Running the exact same calculation repeatedly when you can run it once, store the value and then reuse the value later.
> * Loops that run too many times.
> * Creating unnecessarily complex data structures when simpler structures work equivalently.
> * Unnecessary control flow checks.

###Accuracy

>Criteria:
>
>px, py, vx, vy output coordinates must have an RMSE <= [0.30, 0.16, 0.95, 0.70] after running for longer than 1 second.

>Meets specifications
>
>The simulation collects the position and velocity values that your algorithm outputs and they are compare to the ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [0.30, 0.16, 0.95, 0.70] after the simulator has ran for longer than 1 second. The simulator will also display if RMSE values surpass the threshold.

###Follows the Correct Algorithm

>Criteria:
>
>Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

>Meets specifications
>
>While you may be creative with your implementation, there is a well-defined set of steps that must take place in order to successfully build a Kalman Filter. As such, your project should follow the algorithm as described in the preceding lesson.

##Project Description

###Introduction

####CTRV Model

Within this project, cars shall be tracked with a **C**onstant **T**urn **R**ate and **V**elocity magnitude model (CTRV model), using a kalman filter. Since non-linear functions are involved, an unscented kalman filter will be used, but details on this will follow.

<img src="media/report/ctrv_model.png" width="943" height="831" />

**Figure 1** - CTRV model - Source: Udacity

The general state vector of the CTRV model consists of five elements:

![x = \begin{bmatrix} p_x\\ p_y\\ v\\ \psi\\ \dot{\psi} \end{bmatrix}](https://render.githubusercontent.com/render/math?math=x%20%3D%20%5Cbegin%7Bbmatrix%7D%20p_x%5C%5C%20p_y%5C%5C%20v%5C%5C%20%5Cpsi%5C%5C%20%5Cdot%7B%5Cpsi%7D%20%5Cend%7Bbmatrix%7D)

Where ![p_x](https://render.githubusercontent.com/render/math?math=p_x) and ![p_y](https://render.githubusercontent.com/render/math?math=p_y) are the cartesian coordinates in 2D space, ![v](https://render.githubusercontent.com/render/math?math=v) is the velocity (from the perspective of the tracking object), ![\psi](https://render.githubusercontent.com/render/math?math=%5Cpsi) is the yaw angle and ![\dot{\psi}](https://render.githubusercontent.com/render/math?math=%5Cdot%7B%5Cpsi%7D) ist the yaw rate.

![\dot{p_x} = cos(\psi) \cdot v](https://render.githubusercontent.com/render/math?math=%5Cdot%7Bp_x%7D%20%3D%20cos(%5Cpsi)%20%5Ccdot%20v)

![\dot{p_y} = sin(\psi) \cdot v](https://render.githubusercontent.com/render/math?math=%5Cdot%7Bp_y%7D%20%3D%20sin(%5Cpsi)%20%5Ccdot%20v)

![\dot{v} = 0](https://render.githubusercontent.com/render/math?math=%5Cdot%7Bv%7D%20%3D%200)

![\dot{\Psi} = \dot{\Psi}](https://render.githubusercontent.com/render/math?math=%5Cdot%7B%5CPsi%7D%20%3D%20%5Cdot%7B%5CPsi%7D)

![\dot\dot{\Psi} = 0](https://render.githubusercontent.com/render/math?math=%5Cdot%5Cdot%7B%5CPsi%7D%20%3D%200)


![x_{k+1} = x_k + \begin{bmatrix} \frac{v_k}{\Psi_k} (sin(\Psi_k + \dot{\Psi_k}\Delta t) - sin(\dot{\Psi_k})) \\ \frac{v_k}{\Psi_k} (-cos(\Psi_k + \dot{\Psi_k}\Delta t) + cos(\dot{\Psi_k})) \\ 0 \\ \dot{\Psi_k} \Delta t \\ 0 \end{bmatrix}](https://render.githubusercontent.com/render/math?math=x_%7Bk%2B1%7D%20%3D%20x_k%20%2B%20%5Cbegin%7Bbmatrix%7D%20%5Cfrac%7Bv_k%7D%7B%5CPsi_k%7D%20(sin(%5CPsi_k%20%2B%20%5Cdot%7B%5CPsi_k%7D%5CDelta%20t)%20-%20sin(%5Cdot%7B%5CPsi_k%7D))%20%5C%5C%20%5Cfrac%7Bv_k%7D%7B%5CPsi_k%7D%20(-cos(%5CPsi_k%20%2B%20%5Cdot%7B%5CPsi_k%7D%5CDelta%20t)%20%2B%20cos(%5Cdot%7B%5CPsi_k%7D))%20%5C%5C%200%20%5C%5C%20%5Cdot%7B%5CPsi_k%7D%20%5CDelta%20t%20%5C%5C%200%20%5Cend%7Bbmatrix%7D)




####Kalman filter

For this project two sensors will be used as input for the kalman filter, to estimate the position of the object to be tracked. The estimation process is split into a state prediction step and and a measurement update step.

<img src="media/report/project_map.png" width="1828" height="1017" />

**Figure 2** - project map - Source: Udacity

Both steps work on the state of an object. Earlier in the CTRV model section the state was been described by the general state vector. Since the kalman filter is performing an estimation on this state, the values obviously won't be absolutely the ground truth. So in terms of the kalman filter we are talking about a so called "mean state vector" **x**, because the state elements (e.g. x & y coordinates) are been represented by an gaussian distribution, where the "mean" is been given by the "mean state vector".
The gaussian distribution, or more practically the uncertainty of the state is been described by the covariance matrix **P**.

Both, the state **x** and the covariance matrix **P** need to be initialized with the first measurements before the iteration through the estimation process can start.

Let's assume we start with an initialized state at point in time k (![x_k](https://render.githubusercontent.com/render/math?math=x_k) ), the filter will iterate through the following steps:

* receiving a measurement after ![\Delta t](https://render.githubusercontent.com/render/math?math=%5CDelta%20t) or ![k+1](https://render.githubusercontent.com/render/math?math=k%2B1)
* predict the new state (![x_{k+1 | k}](https://render.githubusercontent.com/render/math?math=x_%7Bk%2B1%20%7C%20k%7D) ) after ![\Delta t](https://render.githubusercontent.com/render/math?math=%5CDelta%20t) not considering the new measurement yet
* update to the new state (![x_{k+1}](https://render.githubusercontent.com/render/math?math=x_%7Bk%2B1%7D) ) based on the prediction **and** the measurement. The prediction and the measurement are weighted based on their uncertainty.

This basically means that the new state will be a bit more biased on either the measurement or the prediction, depending on the trustworthiness, expressed through the uncertainty given by the covariance matrix.
This will be repeated for each measurement and both sensors. E.g. if there is a lidar (L) measurement coming in followed by a radar (R) measurement, the filter will perform the same steps sequentially for both measurements.

<img src="media/report/kalman_flow.png" width="2690" height="1398" />

**Figure 3** - flow of kalman filter - Source: Udacity

If the measurements would come in at the same time, the filter would just start with one of the measurements and proceed with the other, the order of the sensors won't matter in this case, but the second prediction step could actually been skipped since it would produce the same results as the first time.

#####state prediction

To predict the state in the next time step a state transition function is needed. The general form looks like the following:

![x_{k+1 | k} = Fx_{k} (+ Bu) + \nu](https://render.githubusercontent.com/render/math?math=x_%7Bk%2B1%20%7C%20k%7D%20%3D%20Fx_%7Bk%7D%20(%2B%20Bu)%20%2B%20%5Cnu)

where ![F](https://render.githubusercontent.com/render/math?math=F) is the state transition matrix, ![B](https://render.githubusercontent.com/render/math?math=B) is the so called control input matrix, ![u](https://render.githubusercontent.com/render/math?math=u) is the corresponding control vector and ![\nu](https://render.githubusercontent.com/render/math?math=%5Cnu) representing process noise.
The control input matrix and vector would normally provide information about the objects internal behaviour, like acceleration at a given point in time, but for this project they will be ignored (![Bu = 0](https://render.githubusercontent.com/render/math?math=Bu%20%3D%200) ) since the assumption is, that this information is just not available to our tracking. Instead this lag of information about e.g. the precise acceleration will be considered as process noise ![\nu](https://render.githubusercontent.com/render/math?math=%5Cnu).

The process noise is been defined as ![\nu = N(0,Q)](https://render.githubusercontent.com/render/math?math=%5Cnu%20%3D%20N(0%2CQ)) , a gaussian distributed noise, having a mean of zero and a covariance of ![Q](https://render.githubusercontent.com/render/math?math=Q).

This uncertainty is been considered in the update of the covariance matrix:

![P_{k+1|k} = FP_kF^T+Q](https://render.githubusercontent.com/render/math?math=P_%7Bk%2B1%7Ck%7D%20%3D%20FP_kF%5ET%2BQ)

This gives us a prediction of the state expressed as gaussian distribution.

#####measurement update

Receiving a sensors measurement ![z](https://render.githubusercontent.com/render/math?math=z) provides data which can be compared to our prediction ![x_{k+1 | k}](https://render.githubusercontent.com/render/math?math=x_%7Bk%2B1%20%7C%20k%7D) via the measurement function ![H](https://render.githubusercontent.com/render/math?math=H).

![y = z-Hx_{k+1 | k}](https://render.githubusercontent.com/render/math?math=y%20%3D%20z-Hx_%7Bk%2B1%20%7C%20k%7D)

Like the prediction, the measurement has also an uncertainty which is been describe by the matrix ![R](https://render.githubusercontent.com/render/math?math=R) . This can also be expressed as a gaussian distributed measurement noise ![\omega ~ N(0,R)](https://render.githubusercontent.com/render/math?math=%5Comega%20~%20N(0%2CR)) , with a mean of zero and a covariance ![R](https://render.githubusercontent.com/render/math?math=R) .

The so called kalman gain is combining the uncertainty of the prediction (![P_{k+1|k}](https://render.githubusercontent.com/render/math?math=P_%7Bk%2B1%7Ck%7D) ) and the measurement (![R](https://render.githubusercontent.com/render/math?math=R)) to give the more certain value a higher weight in the overall result.

![K = PH^TS^{-1}](https://render.githubusercontent.com/render/math?math=K%20%3D%20PH%5ETS%5E%7B-1%7D) where ![S = HPH^T+R](https://render.githubusercontent.com/render/math?math=S%20%3D%20HPH%5ET%2BR)

The state after the measurement update calculates as following:

![x_{k+1} = x_k + Ky](https://render.githubusercontent.com/render/math?math=x_%7Bk%2B1%7D%20%3D%20x_k%20%2B%20Ky)

The updated covariance matrix calculates according to the following formula:

![P_{k+1} = (I-KH)P_{k+1|k}](https://render.githubusercontent.com/render/math?math=P_%7Bk%2B1%7D%20%3D%20(I-KH)P_%7Bk%2B1%7Ck%7D)

###Implementation

####Initialization

```C++
//for lidar the initial state vector will be initialized directly with the raw cartesian coordinates from the raw measurement
        if (meas_package.LASER == meas_package.sensor_type_) {
            auto px = meas_package.raw_measurements_(0);
            auto py = meas_package.raw_measurements_(1);
            x_ << px, py, 0, 0, 0;

            //for radar the raw measurement needs to be translated to cartesian coordinates first before initializing the state vector
        } else if (meas_package.RADAR == meas_package.sensor_type_) {
            auto roh = meas_package.raw_measurements_(0);
            auto theta = meas_package.raw_measurements_(1);
            auto roh_dot = meas_package.raw_measurements_(2);
            auto px = roh * cos(theta);
            auto py = roh * sin(theta);
            x_ << px, py, roh_dot, theta, 0;
```

```C++
/**
   * the covariance matrix gets initialized with
   * process noise standard deviation longitudinal acceleration std_a_ and
   * process noise standard deviation yaw acceleration std_yawdd_
  **/
  P_ = MatrixXd::Identity(n_x_, n_x_);
  P_(2, 2) = std_a_ * std_a_;
  P_(3, 3) = std_yawdd_ * std_yawdd_;

```

####prediction

Since the transfer functions of the CTRV model are non-linear, the unscented version of the kalman filter has been used.

TODO: extend UKF description

```C++
void UKF::Prediction(double delta_t) {
    /**
     * Modify the state vector, x_. Predict sigma points, the state,
     * and the state covariance matrix.
     */
    GenerateSigmaPoints();
    PredictSigmaPoints(delta_t);
    PredictMeanAndCovariance();
}
```

####update

```C++
if (is_initialized_) {

    Prediction(delta_t);
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        UpdateLidar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        UpdateRadar(meas_package);
    } else {
        return;
    }
```

```C++
// update state mean and covariance matrix
x_ = x_ + K_ * (z_ - z_pred_);
P_ = P_ - K_ * S_ * K_.transpose();
```
