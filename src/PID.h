#ifndef PID_H
#define PID_H

#include <vector>


class PID {
public: 
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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
   * Twiddle control parameters
   */
  void initTwiddle(double dKp_, double dKi_, double dKd_, double maxCTE_);
  bool Twiddle(double cte);
  bool getTwiddleFinished();
  
private:
    
  void stepTwiddle();
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * flags for antiwindup
   */
  bool b_maxLimit;
  bool b_minLimit;
  
  /*
   *variables for twiddle
   */
  double dKp, dKi, dKd;
  double KpBest, KiBest, KdBest;
  double bestErr, collErr, maxCTE;
  unsigned int itorTwiddle, nStepsTwiddle, bestSteps;
  bool up;
  
};

#endif /* PID_H */
