#include <iostream>
#include <fstream>
#include <cmath>
#include "pid_controller.h"

using namespace std;


/**
* Runs PID controller against a target controlled by the input function. Results are logged to the given filename.
*
* _params: The parameters to be used for the PID controller.
* _output_file: The file to which simulation results should be written.
* _max_t: The maximum time for which to run the simulation.
* _dt: The timestep to be used.
* _should_stabilize: Whether this case is expected to stabilize close to the target.
* _targetFunction: A function which takes the current time as a double and yields the target as a double.
* 
* Returns: True if the system has stabilized to within 0.05 or the _should_stabilize flag is false, false otherwise.
*/
bool testGenericFunction(PIDParameters _params, string _output_file, double _max_t, double _dt, bool _should_stabilize, double (*_targetFunction)(double)) {
	//Write file header
    ofstream out;
    out.open(_output_file);
    out << "Time,Target,State" << endl;
    
    double t = 0;
    double dt = 0.01;
    double state = 0;
    double velocity = 0;
    double target = 0;
    
    PIDController controller(_params);
    controller.initialize(target, state, t);
    t += dt;
    
    while (t < _max_t) {
    	target = _targetFunction(t);
    	controller.setTarget(target);
    	double output = controller.calculateOutput(state, t);
    	velocity += output * dt;
    	state += velocity * dt;
    	
    	out << t << "," << target << "," << state << endl;
    	t += dt;
    }
     
    out.close();
    
    return !_should_stabilize || abs(target - state) < 0.05;
}

/**
* Generates a step function which steps from 0 to 1 at t=1s.
*
* _t: The time at which to get the target.
*/
double stepFunction(double _t) {
	if (_t < 1) {
		return 0;
	}
	else {
		return 1;
	}
}

/**
* Generates a square wave between 0 and 1, switching every 1s.
*
* _t: The time at which to get the target.
*/
double squareWave(double _t) {
	int f = (int)floor(_t/10);
	if (f % 2 == 0) {
		return 1;
	}
	else {
		return 0;
	}
}

/**
* Generates an uneven square wave between 0 and either 0.5 or 1, switching every 1s.
*
* _t: The time at which to get the target.
*/
double unevenSquareWave(double _t) {
	int f = (int)floor(_t/10);
	if (f % 2 == 0) {
		if (f % 4 == 0) {
			return 0.5;
		}
		else {
			return 1;
		}
	}
	else {
		return 0;
	}
}

/**
* Generates a sine wave of period 1s.
*
* _t: The time at which to get the target.
*/
double sinWave(double _t) {
	return sin(_t * 6.283185);
}

/**
* Runs all test cases.
*/
int main() {
	PIDParameters params {
    	k_p: 5,
    	k_i: 5,
    	k_d: 5
    };
    
    if (!testGenericFunction(params, "test_results/step_function.csv", 10, 0.01, true, stepFunction)) {
    	return 1;
    }
    if (!testGenericFunction(params, "test_results/square_wave.csv", 100, 0.01, true, squareWave)) {
    	return 1;
    }
    if (!testGenericFunction(params, "test_results/uneven_square_wave.csv", 100, 0.01, true, unevenSquareWave)) {
    	return 1;
    }
    if (!testGenericFunction(params, "test_results/sinusoid.csv", 10, 0.01, false, sinWave)) {
	    return 1;
    }
    
    return 0;
}
