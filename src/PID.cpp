#include <iostream>
#include <math.h>
#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_,bool run_twiddle) {
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

  if(run_twiddle){
    tolerance = 0.005;
    delta_p = -0.01;
  }
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double steer = (Kp * p_error)+(Kd * d_error)+(Ki * i_error);  // TODO: Add your total error calc here!
  if (steer < -1) {
    steer = -1;
  }
  if (steer > 1) {
    steer = 1;
  }
  return steer;
}

void PID::Twiddle(double total_error,double hyperparameter){
  static double best_error = 100000;
  static bool is_twiddle_init = false;
  static bool is_twiddle_reset = false;

  static double last_hyper_parameter = 0;

  std::cout<<"best error = "<<best_error<<std::endl;
  std::cout<<"delta p = "<<delta_p<<std::endl;
  if(!is_twiddle_init){
    std::cout<<"Twiddle init";
    best_error = total_error;
    is_twiddle_init = true;
    return;
  }

  if(fabs(delta_p)>tolerance){
    if(is_twiddle_reset){
      last_hyper_parameter = hyperparameter;
      hyperparameter += delta_p;
      std::cout<<"Hyperparameter magnitude increased!"<<std::endl;
      is_twiddle_reset = false;
    }else {
      if(total_error < best_error){
        delta_p *=1.1;
        is_twiddle_reset = true;
        best_error = total_error;
      }else {
        if(fabs(last_hyper_parameter) <fabs(hyperparameter)){
          last_hyper_parameter =hyperparameter;
          hyperparameter -= 2.0 * delta_p;
          std::cout<<"Hyperparameter magnitude decreased!"<<std::endl;
        }else {
          last_hyper_parameter = hyperparameter;
          hyperparameter +=delta_p;
          delta_p *=0.9;
          std::cout<<"Hyperparameter magnitude kept same!"<<std::endl;
  				is_twiddle_reset = true;
        }
      }
    }
  }
}