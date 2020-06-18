#include <iostream>
#include <math.h>
#include <algorithm>
#include "PID.h"
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  /*Set Kp, Ki and Kd to initial values passed by controller system. These are passed from main.cpp*/
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;
  // intial error of P I D is zero 
  p_error = 0;
  i_error = 0;
  d_error = 0;

  // Previous cte.
  prev_cte = 0.0;

  // Counters.
  counter = 0;
  errorSum = 0.0;
  minError = std::numeric_limits<double>::max();
  maxError = std::numeric_limits<double>::min();
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
 // Proportional error.
  p_error = cte;

  // Integral error.
  i_error += cte;

  // Diferential error.
  d_error = cte - prev_cte;
  prev_cte = cte;

  errorSum += cte;
  counter++;

  if ( cte > maxError ) {
    maxError = cte;
  }
  if ( cte < minError ) {
    minError = cte;
  }
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double steer = -(Kp * p_error)-(Kd * d_error)-(Ki * i_error);  // TODO: Add your total error calc here!
  if (steer < -1) {
    steer = -1;
  }
  if (steer > 1) {
    steer = 1;
  }
  return steer;
}