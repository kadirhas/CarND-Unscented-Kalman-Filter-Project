#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // normalize the angle here

  if(estimations.size() != ground_truth.size()
      || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
      }
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        
        VectorXd errors = estimations[i]-ground_truth[i];
        
        errors = errors.array()*errors.array();
        rmse += errors;
    
  }

  //calculate the mean
  // ... your code here
  rmse = rmse.array()/estimations.size();

  //calculate the squared root
  // ... your code here
    rmse = rmse.array().sqrt();
  //return the result
  return rmse;
}
