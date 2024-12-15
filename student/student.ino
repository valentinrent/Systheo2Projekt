#include <SignalSource.h>
#include <math.h>


// Global Variables
//TODO
double dervlasterrorval;
unsigned long dervlasttimestamp;


unsigned long integrallasttimestamp;
double errintegral;

double T_krit = 0.312;
double K_krit = 1.4;

double controltorque;
double T_i = 0.8*T_krit;
double T_d = 0.1*T_krit;
double K_p = 0.35*K_krit;
double K_i = K_p/T_i;
double K_d = K_p * T_d;



//Resets the controller state when an experiment is started.
void reset(){
  //TODO
}

/**
 * Rescales the position value from the motor.
 *
 * @param position The original position value to be rescaled.
 * @return The rescaled position value, adjusted to remain within a range of 0 to 360.
 */
float rescalepos(int16_t position){  //rescaling function
  double angle = (360*(double)position)/1010.00;
  //set to down to 180 degrees
  angle -= 8.91;
  if(angle<0){angle += 360;} 
  return angle;
}

/**
 * Calculates the error between the desired setpoint and the current position.
 *
 * @param setpoint The desired target value.
 * @param currentpos The current position value.
 * @return The error value, normalized to account for smallest angle to target value from current value.
 */
int16_t calculate_error(int16_t setpoint, int16_t currentpos){
  //TODO
  int16_t angle = 0;
  currentpos = rescalepos(currentpos);
  angle = setpoint - currentpos;
  if(angle > 180){
    angle -= 360;
  }
  if(angle < -180){
      angle += 360;
    }
  
  
  return angle;
}

/**
 * Calculates the derivative of the error for use in a PID controller.
 *
 * @param error The current error value.
 * @return The change in error (delta) since the last call.
 */
int16_t error_derivative(int16_t error){
  //TODO
  int16_t deriv = (error-dervlasterrorval);
  dervlasterrorval = error;
  return deriv;
}

/**
 * Calculates the integral of the error for use in a PID controller.
 *
 * @param error The current error value.
 * @return The accumulated error over time (integral).
 */
int16_t error_integral(int16_t error){
  //TODO
  errintegral += error*(double)(millis()-integrallasttimestamp)/1000.0;
  integrallasttimestamp = millis();
  return errintegral;
}

/**
 * Implements a basic controller using proportional, integral, and derivative (PID) control.
 *
 * @param error The current error value.
 * @param error_i The integral of the error.
 * @param error_d The derivative of the error.
 * @param measured_disturbance The measured disturbance value.
 * @return The calculated control torque.
 */
int16_t controller(int16_t error, int16_t error_i, int16_t error_d, int16_t measured_disturbance){
  //TODO

  controltorque = (K_p*error) + (K_i * error_i * T_s) + (K_d * error_d / T_s) - 2 - 1*measured_disturbance;
  //controltorque = (K_p*error) + (K_i * error_i) + (K_d * error_d) - 2 ;
  
  return controltorque;
}

/**
 * Placeholder for the main program loop. Do not change this.
 */
void loop(){
  start_loop();
}

/**
 * Sets up the controller by registering student-defined functions and initializing signals.
 * Do not change this.
 */
void setup(){
  register_student_fcns((studentFcns){
    .calculate_position=rescalepos,
    .calculate_error=calculate_error,
    .calculate_error_integral=error_integral,
    .calculate_error_derivative=error_derivative,
    .calculate_control=controller,
    .reset=reset});
  setup_signal();
}
