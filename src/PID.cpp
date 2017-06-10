#include "PID.h"
#include <algorithm>    // std::min
#include <iostream>     // std::cout
#include <limits>
#include <cmath>

#define TWIDDLE_STEPS 1200
#define TWIDDLE_MIN_STEPS 300
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() 
{
  b_maxLimit = false;
  b_minLimit = false;
}

PID::~PID() {}

void PID::Init(double inKp, double inKi, double inKd) {
  //set controller weights
  Kp = inKp;
  Ki = inKi;
  Kd = inKd;
  
  // init errors
  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  //reset antiwindup flags
  b_maxLimit = false;
  b_minLimit = false;
}

void PID::UpdateError(double cte) {
  //D (last cte is p_error)
  d_error = cte - p_error;
  
  //P
  p_error = cte;
  
  //I including anti-windup
  if (b_maxLimit)
    i_error = std::min(i_error, i_error+cte);
  else if (b_minLimit)
    i_error = std::max(i_error, i_error+cte);
  else
    i_error += cte;
}


double PID::TotalError() {
  double total = Kp*p_error + Ki*i_error + Kd*d_error;
  if (total > 1) 
    {
      b_maxLimit = true;
      b_minLimit = false;
      total = 1;
    }
  else if (total < -1)
    {
      b_maxLimit = false;
      b_minLimit = true;
      total = -1;
    }
  else
    {
      b_maxLimit = false;
      b_minLimit = false;
    }
  
  return total;
}

void PID::initTwiddle(double dKp_, double dKi_, double dKd_, double maxCTE_) {
  maxCTE = maxCTE_;
  dKp = dKp_;
  dKi = dKi_;
  dKd = dKd_;
  bestErr = std::numeric_limits<double>::max();
  bestSteps = 0;
  collErr = 0.0;
  itorTwiddle = 0;
  nStepsTwiddle = 0;
  up = true;
}


bool PID::Twiddle(double cte)
{
  collErr += fabs(cte);
  ++nStepsTwiddle;
  //std::cout << "twiddle collection steps: " << nStepsTwiddle << std::endl;
  if (nStepsTwiddle>=TWIDDLE_STEPS || cte>maxCTE)
    {
      stepTwiddle();
      return true;
    }
  return false;
    
}

void PID::stepTwiddle()
{
  double err = collErr/nStepsTwiddle;
  unsigned int numSteps = nStepsTwiddle;
  // init errors
  p_error = 0;
  i_error = 0;
  d_error = 0;
  //reset antiwindup flags
  b_maxLimit = false;
  b_minLimit = false;
  collErr = 0.0;
  nStepsTwiddle = 0;

  if(bestErr > 1e8) {
          bestErr = err;
          std::cout << "New best Kp: " << Kp << " Kd: " << Kd
                                          << " Ki: " << Ki << " Error: " << bestErr
                                          << " Sum dKi: " << dKp + dKi + dKd << std::endl;
          Kp += dKp;
          return;
  }
  i_error = 0;
  d_error = 0;
  p_error = 0;
  //if(numSteps >= bestSteps || (err < bestErr && numSteps >= bestSteps)) {
  if(err < bestErr && numSteps >= bestSteps) {
          bestErr = err;
          bestSteps = numSteps;
          KpBest = Kp;
          KiBest = Ki;
          KdBest = Kd;
          if(itorTwiddle == 0) dKp *= 1.1;
          else if(itorTwiddle == 1) dKi *= 1.1;
          else dKd *= 1.1;
          itorTwiddle = (itorTwiddle+1)%3;
          up = true;
          std::cout << "twiddle collection steps: " << numSteps << std::endl;
          std::cout << "New best Kp: " << Kp << " Kd: " << Kd
                          << " Ki: " << Ki << " Error: " << bestErr
                          << " Sum dKi: " << dKp + dKi + dKd << std::endl;

  } else {
          std::cout << "twiddle collection steps: " << numSteps << std::endl;
          std::cout << "Skipped Kp: " << Kp << " Kd: " << Kd
                                                          << " Ki: " << Ki << " Error: " << err
                                                          << " Sum dKi: " << dKp + dKi + dKd << std::endl;
          std::cout << "best twiddle collection steps: " << bestSteps << std::endl;
          std::cout << "Best Kp: " << KpBest << " Kd: " << KdBest
                          << " Ki: " << KiBest << " Error: " << bestErr << std::endl;
          
          if(up == true) up = false;
          else {
                  if (itorTwiddle == 0) {
                          Kp += dKp;
                          dKp *= 0.9;
                  }
                  else if (itorTwiddle == 1) {
                          Ki += dKi;
                          dKi *= .9;
                  }
                  else {
                          Kd += dKd;
                          dKd *= .9;
                  }
                  itorTwiddle = (itorTwiddle + 1) % 3;
                  up = true;
          }
  }
  if(itorTwiddle == 0) {
          if(up) Kp += dKp;
          else Kp-= 2*dKp;
  }
  if(itorTwiddle == 1) {
          if(up) Ki += dKi;
          else Ki -= 2*dKi;
  }
  if(itorTwiddle == 2) {
          if(up) Kd += dKd;
          else Kd-= 2*dKd;
  }
}