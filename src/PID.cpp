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
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error = cte + cte_last;
  d_error = cte - cte_last;
  cte_last = cte;
}

double PID::TotalError() {
  return -Kp*p_error -Ki*i_error -Kd*d_error;
}


def twiddle(tol=0.2):
  p = [0, 0, 0]
  dp = [1, 1, 1]
  robot = make_robot()
  x_trajectory, y_trajectory, best_err = run(robot, p)

  it = 0
  while sum(dp) > tol: //replace with if
      print("Iteration {}, best error = {}".format(it, best_err))
      //for i in range(len(p)):


          p[i] += dp[i]
          robot = make_robot()
          x_trajectory, y_trajectory, err = run(robot, p)

          if err < best_err:
              best_err = err
              dp[i] *= 1.1
          else:
              p[i] -= 2 * dp[i]
              robot = make_robot()
              x_trajectory, y_trajectory, err = run(robot, p)

              if err < best_err:
                  best_err = err
                  dp[i] *= 1.1
              else:
                  p[i] += dp[i]
                  dp[i] *= 0.9
          if (i < len(p)-1) {
            i++
          } else {
            i = 0
          }
      it += 1
  return p
