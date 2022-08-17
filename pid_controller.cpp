#include "pid_controller.h"
PIDController::PIDController(PIDParameters _params, double _target, double timestamp, double state) {
    params = _params;
    target = _target;
    last_timestamp = timestamp;
    last_error = state - target;
    error_integral = 0;
}

double PIDController::calculateOutput(double state, double timestamp) {
    //Calculate numerical derivative and integral
    double error = state - target;
    double dt = timestamp - last_timestamp;
    double error_derivative = (error - last_error) / dt;
    error_integral += ((error + last_error) / 2) * dt;

    //Calculate response and update log parameters
    double response = params.k_p * error + params.k_d * error_derivative + params.k_i * error_integral;
    last_timestamp = timestamp;
    last_error = error;
    return response;
}

void PIDController::updateTarget(double _target) {
    target = _target;
}