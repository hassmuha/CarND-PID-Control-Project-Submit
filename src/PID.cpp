#include "PID.h"
#include <iostream>


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_value, double Ki_value, double Kd_value) {
  Kp = Kp_value;
  Ki = Ki_value;
  Kd = Kd_value;
  // put iteration value to 0
  iter = 0;
  total_sq_error = 0;
  //Coefficients
  p_error = 0;
  i_error = 0;
  d_error = 0;

}

void PID::TwiddleInit(double *d_params_) {
  SteadyState = 0;
  best_error = 1e32;
  dp_sum = 1e32;
  param_idx = 0;
  last_state = 0;
  next_state = 0;
  for (int i = 0; i < N_PARAMS; i++)
	{
    d_params[i] = d_params_[i];
	}
  //intializing the twiddle parameters
  params[0] = Kp;
  params[1] = Ki;
  params[2] = Kd;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error += cte;
  // put d_error to 0 for the first iteration
  if (iter == 0) {
    d_error = 0;
  } else {
    d_error = cte - cte_last;
  }
  cte_last = cte;
  iter++;
  if (iter > CoefUpdate/2){ //wait for half of iterations to converge the algorithm based on new coefficients
    total_sq_error += cte*cte;
  }
}

double PID::TotalError() {
  return -Kp*p_error -Ki*i_error -Kd*d_error;
}

void PID::Twiddle(double tol, double mcte) {
  // if the sum of d_params already correspond to tolerance level, means the parameters has been optimized
  if (dp_sum < tol) {
    return;
  }
  if (next_state == 2) {
    // this is main evaluation to check whether to accept the new value for the parameter
    if (mcte < best_error) {
      best_error = mcte;
      d_params[param_idx] *= 1.1;
      next_state = 0;
    } else {
      // last_state = 1 means that the parameter has been checked for both increased and decreased value
      // and now as there is no improvement so change the d_param to lower value for fine tunning
      if (last_state == 1) {
        // update the para
        params[param_idx] += d_params[param_idx];
        d_params[param_idx] *= 0.9;
        next_state = 0;
      } else {
        // last_state is 0 and we have already checked parameter with increased value and it does not bring any improvement
        // change the parameter with lower value
        next_state = 1;
      }
    }
    last_state = 2;
  }

  // In this state increase the one params[param_idx] by d_params[param_idx]
  if (next_state == 0){ // this means increase the param_idx and add the d_param and evaluate in next iteration
    if (last_state == 2) {    // only false for the first time otherwise always true
      param_idx++;
      if (param_idx == N_PARAMS) {
        param_idx = 0;
      }
    }
    params[param_idx] += d_params[param_idx];
    last_state = 0;
    next_state = 2; //to evaluate the result in next state
  } else if (next_state == 1){ // subtract the d_param and evaluate in next iteration
    params[param_idx] -= 2*d_params[param_idx];
    last_state = 1;
    next_state = 2; //to evaluate the result in next state
  }

  // calculate the sum in the end, no need to calculate when the dp_sum already reached the tolerance level
  dp_sum = 0;
  for (int i = 0; i < N_PARAMS; i++)
	{
    dp_sum += d_params[i];
	}
}
