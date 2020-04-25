#include <iostream>
#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    is_initialized_ = false;
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;
    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // State dimension, x,y,acc.,yaw , yawd
    n_x_ = 5;
    // Augmented state dimension
    n_aug_ = 7;
    // Sigma point spreading parameter
    lambda_ = 3 - n_aug_;

    //number of sigma points
    n_sig_ = 2 * n_aug_ + 1;


    // Weights of sigma points
    weights_ = VectorXd(n_sig_);
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < 2 * n_aug_ + 1; i++) { //2n+1 weights
        weights_(i) = 0.5 / (n_aug_ + lambda_);
    }

    // initial state vector
    x_ = VectorXd(n_x_);
    x_.fill(0);
    // initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);
    P_.fill(0);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 2;
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.5;

    time_us_ = 0;

    /**
     * DO NOT MODIFY measurement noise values below.
     * These are provided by the sensor manufacturer.
     */

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;
    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;
    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;
    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;
    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

    /**
     * End DO NOT MODIFY section for measurement noise values
     */



    /**
     * TODO: Complete the initialization. See ukf.h for other member properties.
     * Hint: one or more values initialized above might be wildly off...
     */
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

    double delta_t = (double) ((meas_package.timestamp_ - time_us_) / 1e6);
    time_us_ = meas_package.timestamp_;
    if (is_initialized_) {

        Prediction(delta_t);
        if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
            UpdateLidar(meas_package);
        } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
            UpdateRadar(meas_package);
        } else {
            return;
        }


    } else {
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

        } else {
            std::cout << "Error, sensor type not set for measurement package" << std::endl;
            return;
        }

        /**
         * the covariance matrix gets initialized with
         * process noise standard deviation longitudinal acceleration std_a_ and
         * process noise standard deviation yaw acceleration std_yawdd_
        **/
        P_ = MatrixXd::Identity(n_x_, n_x_);
        P_(2, 2) = std_a_ * std_a_;
        P_(3, 3) = std_yawdd_ * std_yawdd_;

        is_initialized_ = true;
    }

}

void UKF::Prediction(double delta_t) {
    /**
     * Modify the state vector, x_. Predict sigma points, the state,
     * and the state covariance matrix.
     */
    GenerateSigmaPoints();
    PredictSigmaPoints(delta_t);
    PredictMeanAndCovariance();
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
     * about the object's position. Modify the state vector, x_, and
     * covariance, P_.
     * You can also calculate the lidar NIS, if desired.
     */
    // set measurement dimension, lidar can measure x and y
    int n_z_ = 2;
    // create matrix for sigma points in measurement space
    MatrixXd Zsig_ = MatrixXd(n_z_, n_sig_);
    // mean predicted measurement
    VectorXd z_pred_ = VectorXd(n_z_);
    z_pred_.fill(0);
    // measurement covariance matrix S
    MatrixXd S_ = MatrixXd(n_z_, n_z_);
    S_.fill(0);

    for (int i = 0; i < Xsig_pred_.cols(); i++) {
        double px = Xsig_pred_(0, i), py = Xsig_pred_(1, i);
        Zsig_.col(i) << px, py;
        // calculate mean predicted measurement
        z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
    }

    // calculate innovation covariance matrix S
    for (int i = 0; i < Xsig_pred_.cols(); i++) {
        VectorXd deltaX = Zsig_.col(i) - z_pred_;

        S_ = S_ + (weights_(i) * deltaX) * deltaX.transpose();
    }

    MatrixXd R = MatrixXd(n_z_, n_z_);
    R << std_laspx_ * std_laspx_, 0,
            0, std_laspy_ * std_laspy_;


    S_ = S_ + R;

    UpdateState(Zsig_, z_pred_, S_, meas_package.raw_measurements_, n_z_);
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
     * TODO: Complete this function! Use radar data to update the belief
     * about the object's position. Modify the state vector, x_, and
     * covariance, P_.
     * You can also calculate the radar NIS, if desired.
     */

    // set measurement dimension, radar can measure r, phi, and r_dot
    int n_z_ = 3;
    // create matrix for sigma points in measurement space
    MatrixXd Zsig_ = MatrixXd(n_z_, n_sig_);
    // mean predicted measurement
    VectorXd z_pred_ = VectorXd(n_z_);
    z_pred_.fill(0);
    // measurement covariance matrix S
    MatrixXd S_ = MatrixXd(n_z_, n_z_);
    S_.fill(0);


    // transform sigma points into measurement space
    for (int i = 0; i < n_sig_; i++) {
        double px = Xsig_pred_(0, i);
        double py = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double psi = Xsig_pred_(3, i);

        //roh
        double roh = sqrt(pow(px, 2) + pow(py, 2));
        //phi
        double phi = atan2(py, px);
        //rohd
        double rohd = (px * cos(psi) * v + py * sin(psi) * v) / roh;
        Zsig_.col(i) << roh, phi, rohd;

        // calculate mean predicted measurement
        z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
    }


    // calculate innovation covariance matrix S
    VectorXd deltaX = VectorXd(n_x_);
    for (int i = 0; i < Xsig_pred_.cols(); i++) {
        deltaX = Zsig_.col(i) - z_pred_;
        AngleNormalization(deltaX(1));

        S_ = S_ + (weights_(i) * deltaX) * deltaX.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z_, n_z_);
    R << std_radr_ * std_radr_, 0, 0,
            0, std_radphi_ * std_radphi_, 0,
            0, 0, std_radrd_ * std_radrd_;

    S_ = S_ + R;

    UpdateState(Zsig_, z_pred_, S_, meas_package.raw_measurements_, n_z_);

}

void UKF::GenerateSigmaPoints() {

    // create augmented sigma points
    // create augmented mean vector and state covariance to include process noise into sigma points
    Eigen::VectorXd x_aug_ = VectorXd(n_aug_);
    Eigen::MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);
    P_aug_.fill(0);

    //sigma points
    Xsig_ = MatrixXd(n_aug_, n_sig_);
    Xsig_.fill(0);

    // create augmented mean state and covariance matrix
    //mean of process noise is 0
    x_aug_.block(0, 0, n_x_, 1) = x_;
    x_aug_.block(n_x_, 0, n_aug_ - n_x_, 1) << 0, 0;


    P_aug_.block(0, 0, n_x_, n_x_) = P_;
    P_aug_.block(n_x_, n_x_, n_aug_ - n_x_, n_aug_ - n_x_) << std_a_ * std_a_, 0,
            0, std_yawdd_ * std_yawdd_;

    // create square root matrix
    MatrixXd A = P_aug_.llt().matrixL();

    // set first column of sigma point matrix
    Xsig_.col(0) = x_aug_;
    // set remaining sigma points
    for (int i = 0; i < n_aug_; ++i) {
        Xsig_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * A.col(i);
        Xsig_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * A.col(i);
    }

}

void UKF::PredictSigmaPoints(double delta_t) {

    // predicted sigma points matrix
    Xsig_pred_ = MatrixXd(n_x_, n_sig_);

    Eigen::VectorXd x_k1, predict, noise = VectorXd(n_x_);

    // predict sigma points
    for (int i = 0; i < Xsig_pred_.cols(); i++) {
        VectorXd x_k = Xsig_.col(i);

        // extract values for better readability
        double p_x = Xsig_(0, i);
        double p_y = Xsig_(1, i);
        double v = Xsig_(2, i);
        double yaw = Xsig_(3, i);
        double yawd = Xsig_(4, i);
        double nu_a = Xsig_(5, i);
        double nu_yawdd = Xsig_(6, i);

        // predicted state values
        double px_p, py_p;
        // avoid division by zero
        if (fabs(yawd) < 1e-6) {
            px_p = p_x + v * cos(yaw) * delta_t;
            py_p = p_y + v * sin(yaw) * delta_t;
        } else {
            px_p = p_x + (v / yawd) * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_p = p_y + (v / yawd) * (-cos(yaw + yawd * delta_t) + cos(yaw));
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        // add noise
        px_p = px_p + .5 * pow(delta_t, 2) * cos(yaw) * nu_a,
        py_p = py_p + .5 * pow(delta_t, 2) * sin(yaw) * nu_a,
        v_p = v_p + delta_t * nu_a,
        yaw_p = yaw_p + .5 * pow(delta_t, 2) * nu_yawdd,
        yawd_p = yawd_p + delta_t * nu_yawdd;


        // write predicted sigma point into right column
        Xsig_pred_.col(i) << px_p, py_p, v_p, yaw_p, yawd_p;
    }

}

void UKF::PredictMeanAndCovariance() {

    // predict state mean
    x_.fill(0.0);

    for (int i = 0; i < n_sig_; i++) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    // predict state covariance matrix
    P_.fill(0);
    VectorXd deltaX = VectorXd(n_x_);

    for (int i = 0; i < n_sig_; i++) {
        deltaX = Xsig_pred_.col(i) - x_;

        // // angle normalization
        // while (deltaX(3)> M_PI) deltaX(3)-=2.*M_PI;
        // while (deltaX(3)<-M_PI) deltaX(3)+=2.*M_PI;

        P_ = P_ + (weights_(i) * deltaX) * deltaX.transpose();
    }
}

void
UKF::UpdateState(Eigen::MatrixXd &Zsig_, Eigen::VectorXd &z_pred_, Eigen::MatrixXd &S_, Eigen::VectorXd &z_, int n_z_) {
    // create matrix for cross correlation Tc_
    MatrixXd Tc_ = MatrixXd(n_x_, n_z_);
    Tc_.fill(0);

    VectorXd deltaX_ = VectorXd(n_x_);
    VectorXd deltaZ_ = VectorXd(n_x_);

    // calculate cross correlation matrix
    for (int i = 0; i < Xsig_pred_.cols(); i++) {
        deltaX_ = Xsig_pred_.col(i) - x_;
        deltaZ_ = Zsig_.col(i) - z_pred_;

        //In case of radar the values need to be normalized
        if (n_z_ == 3) {
            AngleNormalization(deltaZ_(1));
        }

        Tc_ = Tc_ + weights_(i) * deltaX_ * deltaZ_.transpose();
    }

    // calculate Kalman gain K_;
    MatrixXd K_ = MatrixXd(n_x_, n_z_);
    K_ = Tc_ * S_.inverse();

    // update state mean and covariance matrix
    x_ = x_ + K_ * (z_ - z_pred_);
    P_ = P_ - K_ * S_ * K_.transpose();
}

void UKF::AngleNormalization(double &angle) {
    while (angle > M_PI) angle -= 2. * M_PI;
    while (angle < -M_PI) angle += 2. * M_PI;
}


