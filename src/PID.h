#ifndef PID_H
#define PID_H

#define N_PARAMS	3


class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double cte_last;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle
  */
  int iter;
  int CoefUpdate;
  double total_sq_error;
  int SteadyState;

  double best_error;
  double dp_sum;
  int param_idx;
  int last_state;
  int next_state;
  double params[N_PARAMS];
  double d_params[N_PARAMS];

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_value, double Ki_value, double Kd_value);

  /*
  * Initialize Twiddle.
  */
  void TwiddleInit(double *d_params_);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Calculate the Paramerter optimization using Twiddle.
  */
  void Twiddle(double tol, double mcte);
  //tol = tolerance
  //mcte = mean cte
};

#endif /* PID_H */
