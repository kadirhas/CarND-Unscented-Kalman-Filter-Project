#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
  * Initializes Unscented Kalman filter
  * This is scaffolding, do not modify
*/
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;
  
  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  
  // initial state vector
  x_ = VectorXd(5);
  
  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;
  
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
    TODO:
    
    Complete the initialization. See ukf.h for other member properties.
    
    Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = 5;
  n_aug_ = 7;
  n_aug_size_ = 2*n_aug_+1;
  lambda_ = 3 - n_aug_;
  
  weights_ = VectorXd(2 * n_aug_ + 1);
  Xsig_pred_ = MatrixXd(n_x_, n_aug_size_);
  P_ = MatrixXd::Identity(n_x_,n_x_);
  
  //set weights
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i = 1; i<(n_aug_size_); i++)
  {
    weights_(i) = 0.5/(lambda_+n_aug_);
  }
  is_initialized_ = false;
}

UKF::~UKF() {}

/**
  * @param {MeasurementPackage} meas_package The latest measurement data of
  * either radar or laser.
*/
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
    TODO:
    
    Complete this function! Make sure you switch between lidar and radar
    measurements.
  */
  // call the predict function
  //if radar else lidar thing must be done here
  if (!is_initialized_) {
    // first measurement
    if ((meas_package.sensor_type_ == MeasurementPackage::RADAR)) {
      /**
        Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rhodt = meas_package.raw_measurements_[2];
      double vx = rhodt*cos(meas_package.raw_measurements_[1]);
      double vy = rhodt*sin(meas_package.raw_measurements_[1]);
      //sqrt(vx*vx+vy*vy)
      x_ << meas_package.raw_measurements_[0]*cos(meas_package.raw_measurements_[1]), meas_package.raw_measurements_[0]*sin(meas_package.raw_measurements_[1]), sqrt(vx*vx+vy*vy), 0., 0.;
      time_us_ = meas_package.timestamp_;
      
      // done initializing, no need to predict or update
      is_initialized_ = true;
      cout << "initialized with radar" << endl;
    }
    else if ((meas_package.sensor_type_ == MeasurementPackage::LASER)) {
      /**
        Initialize state.
      */
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0., 0., 0.;
      time_us_ = meas_package.timestamp_;
      
      // done initializing, no need to predict or update
      is_initialized_ = true;
      cout << "initialized with lidar" << endl;
    }
    
    return;
  }
  
  double dt = (meas_package.timestamp_ - time_us_)/ 1000000.0;; //dt - expressed in seconds
  time_us_ = meas_package.timestamp_;
  Prediction(dt);
  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_) {
    UpdateRadar(meas_package);
    //UpdateRadar2(meas_package);
  }
  else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_) {
    UpdateLidar(meas_package);
  }
  
  
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
  //F must be defined here
  //predict sigma points, states and state covariance matrices
  //generate augmented sigma points
  VectorXd x_aug(n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_aug_size_);
  MatrixXd P_aug = MatrixXd(7, 7);
  x_aug.head(5) = x_;
  x_aug(5) = 0;//std_a;
  x_aug(6) = 0;//std_yawdd;
  Xsig_aug.fill(0);
  Xsig_aug.col(0) = x_aug;
  
  P_aug.fill(0.);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  
  MatrixXd a1( n_aug_, n_aug_ );
  
  //note: this might be the issue, lambda_+n_aug_ might need to be squared outside
  a1=(lambda_ + n_aug_)*P_aug;
  a1 = a1.llt().matrixL();
  //a1 = P_aug.llt().matrixL();
  for (int i = 0; i<n_aug_; i++)
  {
    Xsig_aug.col(1+i) = x_aug + a1.col(i);
    Xsig_aug.col(1+i+n_aug_) = x_aug - a1.col(i);
  }
  
  //predict sigma points
  
  for (int i = 0; i < n_aug_size_; i++)
  {
    VectorXd xtemp = VectorXd(5);
    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawdt = Xsig_aug(4,i);
    double std_a = Xsig_aug(5,i);
    double std_yawdt = Xsig_aug(6,i);
    if (fabs(yawdt) < 0.001)
    {
      xtemp(0) = v*cos(yaw)*delta_t;
      xtemp(1) = v*sin(yaw)*delta_t;
      xtemp(2) = 0;
      xtemp(3) = yawdt * delta_t;
      xtemp(4) = 0;
    }
    else
    {
      xtemp(0) = (v/yawdt)*(sin(yaw+yawdt*delta_t)-sin(yaw));
      xtemp(1) = (v/yawdt)*(-cos(yaw+yawdt*delta_t)+cos(yaw));
      xtemp(2) = 0;
      xtemp(3) = yawdt * delta_t;
      xtemp(4) = 0;
    }
    xtemp(0) += 0.5*delta_t*delta_t*cos(yaw)*std_a;
    xtemp(1) += 0.5*delta_t*delta_t*sin(yaw)*std_a;
    xtemp(2) += delta_t*std_a;
    xtemp(3) += 0.5*delta_t*delta_t*std_yawdt;
    xtemp(4) += delta_t*std_yawdt;
    Xsig_pred_.col(i) = Xsig_aug.col(i).head(5)+xtemp;
  }
  //predict mean and covariance
  
  //predict state mean
  x_.fill(0);
  for(int i = 0; i<(n_aug_size_); i++)
  {
    x_ += weights_(i)*Xsig_pred_.col(i);
    
  }
  P_.fill(0);
  //predict state covariance matrix
  for(int i = 0; i<(n_aug_size_); i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //normalize angles
    
    while (x_diff(3)> M_PI)x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    P_ +=  weights_(i) * x_diff * x_diff.transpose() ;
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
  
  int n_z = meas_package.raw_measurements_.size();
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  
  MatrixXd Zsig = MatrixXd(n_z, n_aug_size_);
  Zsig = Xsig_pred_.topRows(n_z); //check its size?
  
  for (int i = 0 ; i<n_aug_size_; i++ )
  {
    z_pred += weights_(i)*Zsig.col(i);
  }
  //calculate innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0);
  
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.);
  for (int i = 0 ; i<n_aug_size_; i++ )
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    S += weights_(i) * z_diff * z_diff.transpose();
    
    // calculate cross correlation matrix
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //normalize angles
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    Tc += weights_(i)*(x_diff)*(z_diff.transpose());
  }
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0);
  
  R(0,0) = std_laspx_*std_laspx_;
  R(1,1) = std_laspy_*std_laspy_;
  S += R;
  
  
  // Update with measurement
  
  //calculate Kalman gain K;
  MatrixXd K;
  K = Tc*S.inverse();
  //update state mean and covariance matrix
  
  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  
  x_ = x_+K*z_diff;
  P_ = P_-K*S*K.transpose();
  
  //NIS calculation
  double NIS_lidar;
  NIS_lidar = z_diff.transpose()*S.inverse()*z_diff;
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
  
  //transform sigma points into measurement space
  int n_z = meas_package.raw_measurements_.size();
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  
  MatrixXd Zsig = MatrixXd(n_z, n_aug_size_);
  
  for (int i = 0 ; i<n_aug_size_; i++ )
  {
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double yawdt = Xsig_pred_(4,i);
    Zsig(0,i) =  sqrt(px*px + py*py);
    Zsig(1,i) = atan2(py,px);
    Zsig(2,i) = (px*cos(yaw)*v + py*sin(yaw)*v)/sqrt(px*px+py*py);//Zsig(0,i);
    //calculate mean predicted measurement
    
    z_pred += weights_(i)*Zsig.col(i);
    
  }
  
  while (z_pred(1)> M_PI) z_pred(1)-=2.*M_PI;
  while (z_pred(1)<-M_PI) z_pred(1)+=2.*M_PI;
  //calculate innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0);
  
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.);
  
  for (int i = 0 ; i<n_aug_size_; i++ )
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    S += weights_(i) * z_diff * z_diff.transpose();
    
    // calculate cross correlation matrix
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //normalize angles
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    Tc += weights_(i)*(x_diff)*(z_diff.transpose());
  }
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0);
  
  R(0,0) = std_radr_*std_radr_;
  R(1,1) = std_radphi_*std_radphi_;
  R(2,2) = std_radrd_*std_radrd_;
  S += R;
  
  
  // Update with measurement
  
  //calculate Kalman gain K;
  MatrixXd K;
  K = Tc*S.inverse();
  //update state mean and covariance matrix
  
  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  
  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  
  x_ = x_+K*z_diff;
  P_ = P_-K*S*K.transpose();
  
  //NIS calculation
  double NIS_radar;
  NIS_radar = z_diff.transpose()*S.inverse()*z_diff;
}
