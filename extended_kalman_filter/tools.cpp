#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse = VectorXd::Zero(4);

  if(estimations.size() == 0){
     cout << "Estimation vector is empty " << endl;
     return rmse;
  }
  else if(estimations.size() != ground_truth.size()){
     cout << "Estimation vector size is not equal to ground truth vector size" << endl;
     return rmse;
  }

  for(int i=0;i<estimations.size();i++) {
     VectorXd sum = estimations[i] - ground_truth[i];
     sum = sum.array()*sum.array();
     rmse += sum;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  try
  {
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