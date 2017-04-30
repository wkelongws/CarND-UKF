#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  
  is_initialized_ = false;
  
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_(2) = 0;
  x_(3) = 0;
  x_(4) = 0;
  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ <<   0.1, 0, 0, 0, 0,
          0, 0.1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
  
//  time_us_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;

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
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  
  n_x_ = 5;
  
  n_aug_ = 7;
  
  lambda_ = 3-n_aug_;
  
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);
  
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(0.5/(lambda_ + n_aug_));
  weights_(0) = lambda_/(lambda_ + n_aug_);
  
  //  NIS_radar_ = 0;
  
  //  NIS_laser_ = 0;
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    cout << "UKF: " << endl;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
       Convert radar from polar to cartesian coordinates and initialize state.
       */
      
      float rho = measurement_pack.raw_measurements_[0];
      float thi = measurement_pack.raw_measurements_[1];
      
      x_(0) = rho*cos(thi);
      x_(1) = rho*sin(thi);
      
      time_us_ = measurement_pack.timestamp_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
       Initialize state.
       */
      //set the state with the initial location
      
      x_(0) = measurement_pack.raw_measurements_[0];
      x_(1) = measurement_pack.raw_measurements_[1];
      time_us_ = measurement_pack.timestamp_;
      
    }
    
    cout << "Initialize:" << endl;
    cout << "x_ = " << x_ << endl;
    cout << "P_ = " << P_ << endl;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  
  double delta_t = (measurement_pack.timestamp_ - time_us_) / 1000000.0;
  time_us_ = measurement_pack.timestamp_;
  
  // prediction step
  Prediction(delta_t);

  cout << "Predict:" << endl;
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
  // update step
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    cout << "Radar:" << endl;
    UpdateRadar(measurement_pack);
  } else {
    cout << "Lidar:" << endl;
    UpdateLidar(measurement_pack);
  }

  cout << "Update:" << endl;
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);
  
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.fill(0.0);
  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  
  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * A.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A.col(i);
  }
  
  for (int i =0; i<2 * n_aug_ + 1; i++)
  {
    VectorXd x_k = Xsig_aug.col(i).head(5);
    double v_a = Xsig_aug.col(i)(5);
    double v_yaw = Xsig_aug.col(i)(6);
    VectorXd x_delta = VectorXd(5);
    VectorXd noise = VectorXd(5);
    if (fabs(x_k(4)) <= 0.001)
    {
      x_delta <<  x_k(2)*cos(x_k(3))*delta_t,
                  x_k(2)*sin(x_k(3))*delta_t,
                  0,
                  x_k(4)*delta_t,
                  0;
      noise <<    0.5*delta_t*delta_t*cos(x_k(3))*v_a,
                  0.5*delta_t*delta_t*sin(x_k(3))*v_a,
                  delta_t*v_a,
                  0.5*delta_t*delta_t*v_yaw,
                  delta_t*v_yaw;
    }
    else
    {
      x_delta <<  x_k(2)/x_k(4)*(sin(x_k(3)+x_k(4)*delta_t)-sin(x_k(3))),
                  x_k(2)/x_k(4)*(-cos(x_k(3)+x_k(4)*delta_t)+cos(x_k(3))),
                  0,
                  x_k(4)*delta_t,
                  0;
      noise <<    0.5*delta_t*delta_t*cos(x_k(3))*v_a,
                  0.5*delta_t*delta_t*sin(x_k(3))*v_a,
                  delta_t*v_a,
                  0.5*delta_t*delta_t*v_yaw,
                  delta_t*v_yaw;
    }
    Xsig_pred_.col(i) = x_k + x_delta + noise;
  }
  
  x_.fill(0.0);
  P_.fill(0.0);
  
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    x_ += Xsig_pred_.col(i)*weights_(i);
  }
  
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    VectorXd dif = Xsig_pred_.col(i)-x_;
    while (dif(3)> M_PI) dif(3)-=2.*M_PI;
    while (dif(3)<-M_PI) dif(3)+=2.*M_PI;
    P_ += weights_(i)*dif*dif.transpose();
  }
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  
  int n_z_ = 2;
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
  Zsig.fill(0.0);
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_,n_z_);
  S.fill(0.0);
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double yawd = Xsig_pred_(4,i);
    
    double rho = sqrt(p_x*p_x+p_y*p_y);
    double phi = atan2(p_y,p_x);
    double rhod = (p_x*cos(yaw)*v+p_y*sin(yaw)*v)/rho;
    
    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;
    
    z_pred += weights_(i)*Zsig.col(i);
  }
  
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R <<    std_laspx_*std_laspx_,  0,
          0,  std_laspy_*std_laspy_;
  
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i)*z_diff*z_diff.transpose();
  }
  
  S += R;
  
  
  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z_);
  z <<  meas_package.raw_measurements_[0],   //px
        meas_package.raw_measurements_[1];   //py
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  Tc.fill(0.0);
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i)-x_;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    VectorXd z_diff = Zsig.col(i)-z_pred;
    Tc += weights_(i)*x_diff*z_diff.transpose();
  }
  MatrixXd K = Tc*S.inverse();
  
  VectorXd z_diff_ = z - z_pred;
  x_ += K*z_diff_;
  P_ += -K*S*K.transpose();
  
  NIS_laser_ = z_diff_.transpose()*S.inverse()*z_diff_;
  
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  
  int n_z_ = 3;
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
  Zsig.fill(0.0);
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_,n_z_);
  S.fill(0.0);
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double yawd = Xsig_pred_(4,i);
    
    double rho = sqrt(p_x*p_x+p_y*p_y);
    double phi = atan2(p_y,p_x);
    double rhod = (p_x*cos(yaw)*v+p_y*sin(yaw)*v)/rho;
    
    Zsig(0,i) = rho;
    Zsig(1,i) = phi;
    Zsig(2,i) = rhod;
    
    z_pred += weights_(i)*Zsig.col(i);
  }

  MatrixXd R = MatrixXd(n_z_, n_z_);
  R <<    std_radr_*std_radr_,  0,                        0,
          0,                    std_radphi_*std_radphi_,  0,
          0,                    0,                        std_radrd_*std_radrd_;
  
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    S += weights_(i)*z_diff*z_diff.transpose();
  }
  
  S += R;
  
  
  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z_);
  z <<  meas_package.raw_measurements_[0],   //rho in m
        meas_package.raw_measurements_[1],   //phi in rad
        meas_package.raw_measurements_[2];   //rho_dot in m/s
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  Tc.fill(0.0);
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i)-x_;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    VectorXd z_diff = Zsig.col(i)-z_pred;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    Tc += weights_(i)*x_diff*z_diff.transpose();
  }
  MatrixXd K = Tc*S.inverse();
  
  VectorXd z_diff_ = z - z_pred;
  //angle normalization
  while (z_diff_(1)> M_PI) z_diff_(1)-=2.*M_PI;
  while (z_diff_(1)<-M_PI) z_diff_(1)+=2.*M_PI;
  x_ += K*z_diff_;
  P_ += -K*S*K.transpose();
  
  NIS_radar_ = z_diff_.transpose()*S.inverse()*z_diff_;
  
}
