//Struct containing PID control parameters
struct PIDParameters {
    double k_p;
    double k_i;
    double k_d;
};

class PIDController {
    public:
        //Initialize new PID controller from parameters
        PIDController(PIDParameters _params, double _target, double current_timestamp, double current_state);
        //Get output value based on current system state
        double calculateOutput(double current_state, double current_timestamp);
        //Update target value
        void updateTarget(double _target);
    private:
        //Currently used PID parameters
        PIDParameters params;
        //Current target value
        double target;
        //Timestamp of last query
        double last_timestamp;
        //State from last query
        double last_error;
        //Cumulative error integral
        double error_integral;
};