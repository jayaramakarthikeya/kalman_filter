#include<iostream>
#include "../Eigen/Dense"
#include <vector>
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using namespace std;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main(){
    
    VectorXd predicted_state(4);
    predicted_state << 1, 2,0.2,0.4;

    MatrixXd Hj = CalculateJacobian(predicted_state);
    cout << "Hj = " << endl << Hj << endl;
    
    return 0;
}

MatrixXd CalculateJacobian(const VectorXd& predicted_state){

    MatrixXd Hj(3,4);

    float px = predicted_state(0);
    float py = predicted_state(1);
    float vx = predicted_state(2);
    float vy = predicted_state(3);

    try
    {
        /* code */
        float dr_dpx = px/(sqrt(px*px+py*py));
        float dr_dpy = py/(sqrt(px*px+py*py));
        float dphi_dx = -py/sqrt(px*px+py*py);
        float dphi_dy = px/sqrt(px*px+py*py);
        float drdot_dpx = (py*(vx*py-px*vy)/pow(px*px+py*py,3/2.0));
        float drdot_dpy = (px*(vy*px-py*vx)/pow(px*px+py*py,3/2.0));
        float drdot_dvx = dr_dpx;
        float drdot_dvy = dr_dpy;

        Hj << dr_dpx,dr_dpy,0,0,
            dphi_dx,dphi_dy,0,0,
            drdot_dpx,drdot_dpy,drdot_dvx,drdot_dvy;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        Hj = MatrixXd::Zero(3,4);
        return Hj;
    }
    return Hj;
}
