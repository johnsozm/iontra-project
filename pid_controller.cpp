#include "pid_controller.h"

PIDController::PIDController(PIDParameters _params) {
    params = _params;
}

void PIDController::initialize(double _target, double _state, double _timestamp) {
	target = _target;
	last_error = _target - _state;
	last_timestamp = _timestamp;
	error_integral = 0;
}

double PIDController::calculateOutput(double state, double timestamp) {
    //Calculate numerical derivative and integral
    //TODO: Is this guaranteed to be polled at a regular interval? Could use finite differences here if so
    double error = target - state;
    double dt = timestamp - last_timestamp;
    double error_derivative = (error - last_error) / dt;
    error_integral += ((error + last_error) / 2) * dt;

    //Calculate response and update log variables
    double response = params.k_p * error + params.k_d * error_derivative + params.k_i * error_integral;
    last_timestamp = timestamp;
    last_error = error;
    return response;
}
