/**
* Struct for basic PID operation parameters.
*
* k_p: Proportional response coefficient.
* k_i: Integral response coefficient.
* k_d: Derivative response coefficient.
*/
struct PIDParameters {
    double k_p;
    double k_i;
    double k_d;
}; //TODO: Should this be in k_p/t_i/t_d form instead?

class PIDController {
    public:
        /**
        * Constructor function. Creates a new controller instance with the given parameters.
        *
        * _params: a PIDParameters struct containing the PID response parameters.
        *
        * Returns: A PIDController instance using the given parameters.
        */
        PIDController(PIDParameters _params);
        
        /**
        * Initializes the controller instance with the current system state and timestamp as a baseline.
        * Should be called immediately before use.
        *
        * _target: The target system state.
        * _state: The current system state.
        * _timestamp: The current time.
        */
        void initialize(double _target, double _state, double _timestamp);
        
        
        /**
        * Calculates the controller response to the current system state and timestamp.
        *
        * _state: The current system state.
        * _timestamp: The current time.
        *
        * Returns: The calculated controller response.
        */
        double calculateOutput(double _state, double _timestamp);
        
        //Getter and setter methods
        PIDParameters getParams() {return params;}
        void setParams(PIDParameters _params) {params = _params;}
        double getTarget() {return target;}
        void setTarget(double _target) {target = _target;}
        
    private:
        //PID parameters
        PIDParameters params;
        //Current target value
        double target;
        //Time of last query
        double last_timestamp;
        //State from last query
        double last_error;
        //Cumulative error integral
        double error_integral;
};
