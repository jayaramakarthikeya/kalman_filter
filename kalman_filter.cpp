#include "kalman_filter.h"

Kalman_filter::Kalman_filter(){
}

Kalman_filter::~Kalman_filter(){
}

void Kalman_filter::Predict(){
    x_ = F_*x_;
    MatrixXd F_t = F_.transpose();
    P_ = F_*P_*F_t + Q_;
    
}

void Kalman_filter::Update(const VectorXd &z){
    VectorXd y = z - H_*x_;
    MatrixXd H_t = H_.transpose();
    MatrixXd S_ = H_*P_*H_t + R_;
    MatrixXd K = P_*H_t*S_.inverse();
    x_ = x_ + K*y;
    long x_size = x_.size();
    MatrixXd I_ = MatrixXd::Identity(x_size,x_size);
    P_ = (I_ - K*H_)*P_;
}