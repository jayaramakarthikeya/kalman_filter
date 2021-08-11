#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "../Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Kalman_filter {
    public:
        //state vector
        VectorXd x_;

        //state covariance matrix
        MatrixXd P_;

        //state transition matrix
        MatrixXd F_;

        //process covariance amtrix
        MatrixXd Q_;

        //measurement matrix
        MatrixXd H_;

        //measurement covariance matrix
        MatrixXd R_;

        Kalman_filter();

        //destructor
        virtual ~Kalman_filter();

        /**
         * Function predict the state and state covariance matrix
         * using process model
         */
        void Predict();

        /**
         * Updates the state and z at mesurement time k+1
         */
        void Update(const VectorXd &z);
};

#endif