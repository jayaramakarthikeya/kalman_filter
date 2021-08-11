#include "../Eigen/Dense"
#include <iostream>
#include <cmath>
#include "tracking.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tracking::Tracking(){
    is_initialized_ = false;
    previous_timestamp_ = 0;

    kf_.x_ = VectorXd(4);

    //state covariance matrix
    kf_.P_ = MatrixXd(4,4);
    kf_.P_ << 1,0,0,0,
              0,1,0,0,
              0,0,1000,0,
              0,0,0,1000;

    //measurement covariance matrix
    kf_.R_ = MatrixXd(2,2);
    kf_.R_ << 0.0225 , 0,
              0,0.0225;

    //measuremnt matrix
    kf_.H_ = MatrixXd(2,4);
    kf_.H_ << 1,0,0,0,
              0,1,0,0;

    //state transition matrix
    kf_.F_ = MatrixXd(4,4);
    kf_.F_ << 1,0,1,0,
              0,1,0,1,
              0,0,1,0,
              0,0,0,1;

    //accelaration noise components
    noise_ax = 5;
    noise_ay = 5;
}

Tracking::~Tracking(){
}

void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack){
    if(!is_initialized_){
        kf_.x_ << measurement_pack.raw_measurement_[0] ,  measurement_pack.raw_measurement_[1],0,0;
        is_initialized_ = true;
        previous_timestamp_ = measurement_pack.timestamp_;
        return;
    }

    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    kf_.F_(0,2) = dt;
    kf_.F_(1,3) = dt;

    kf_.Q_ = MatrixXd(4,4);
    kf_.Q_ << (pow(dt,4)/4)*noise_ax,0,(pow(dt,3)/2)*noise_ax,0,
              0,(pow(dt,4)/4)*noise_ay,0,(pow(dt,3)/2)*noise_ay,
              (pow(dt,3)/2)*noise_ax,0,(pow(dt,2))*noise_ax,0,
              0,(pow(dt,3)/2)*noise_ay,0,(pow(dt,2))*noise_ay;

    kf_.Predict();
    kf_.Update(measurement_pack.raw_measurement_);

    cout << "x= " << kf_.x_ << endl;
    cout << "P= " << kf_.P_ << endl;
}