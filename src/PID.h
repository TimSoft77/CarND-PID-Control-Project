#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double i_error;
  double i_error_threshold; // If abs(cte) > this, i_error set to 0
  double d_error;

  /*
  * To track if the errors have been initialized
  */
  bool is_init = false;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Sets the coefficients for the controller
  */
  void Init(double kp, double ki, double kd, double i_err_thresh);

  /*
  * Calculates the new steering angle and updates the errors
  */
  double CalculateControlOutput(double cte);

};

#endif /* PID_H */
