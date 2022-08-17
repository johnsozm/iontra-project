#include <iostream>
#include <fstream>
#include <cmath>
#include "pid_controller.h"

using namespace std;

/**
* Runs PID controller against a target which suddenly steps to 1.
*
* Results are logged to test_results/step_function.csv.
* Returns: True if the system has stabilized to within 0.05, false otherwise.
*/
bool testStepFunction() {
	//Write file header
    ofstream out;
    out.open("test_results/step_function.csv");
    out << "Time,Target,State" << endl;
    
    PIDParameters params {
    	k_p: 5,
    	k_i: 5,
    	k_d: 5
    };
    
    double t = 0;
    double dt = 0.01;
    double state = 0;
    double velocity = 0;
    double target = 0;
    
    PIDController controller(params);
    controller.initialize(target, state, t);
    
    for (int i = 0; i < 1000; i++) {
    	t += dt;
    	if (i == 100) {
    		target = 1;
    	}
    	controller.updateTarget(target);
    	double output = controller.calculateOutput(state, t);
    	velocity += output * dt;
    	state += velocity * dt;
    	
    	out << t << "," << target << "," << state << endl;
    }
     
    out.close();
    
    return abs(target - state) < 0.05;
}

/**
* Runs PID controller against a target which changes as a square wave.
*
* Results are logged to test_results/square_wave.csv.
* Returns: True if the system has stabilized to within 0.05, false otherwise.
*/
bool testSquareWave() {
    //Write file header
    ofstream out;
    out.open("test_results/square_wave.csv");
    out << "Time,Target,State" << endl;
    
    PIDParameters params {
    	k_p: 5,
    	k_i: 5,
    	k_d: 5
    };
    
    double t = 0;
    double dt = 0.01;
    double state = 0;
    double velocity = 0;
    double target = 0;
    
    PIDController controller(params);
    controller.initialize(target, state, t);
    
    for (int i = 0; i < 10000; i++) {
    	t += dt;
    	if (i % 1000 == 0) {
    		target = 1 - target;
    	}
    	controller.updateTarget(target);
    	double output = controller.calculateOutput(state, t);
    	velocity += output * dt;
    	state += velocity * dt;
    	
    	out << t << "," << target << "," << state << endl;
    }
     
    out.close();
    
    return abs(target - state) < 0.05;
}

/**
* Runs PID controller against a target which changes as an uneven square wave.
*
* Results are logged to test_results/uneven_square_wave.csv.
* Returns: True if the system has stabilized to within 0.05, false otherwise.
*/
bool testUnevenSquareWave() {
    //Write file header
    ofstream out;
    out.open("test_results/uneven_square_wave.csv");
    out << "Time,Target,State" << endl;
    
    PIDParameters params {
    	k_p: 5,
    	k_i: 5,
    	k_d: 5
    };
    
    double t = 0;
    double dt = 0.01;
    double state = 0;
    double velocity = 0;
    double target = 0;
    
    PIDController controller(params);
    controller.initialize(target, state, t);
    
    for (int i = 0; i < 10000; i++) {
    	t += dt;
    	if (i % 1000 == 0) {
    		if (target > 0) {
    			target = 0;
    		}
    		else {
    			if (i % 4000 == 0) {
    				target = 0.5;
    			}
    			else {
    				target = 1.0;
    			}
    		}
    	}
    	controller.updateTarget(target);
    	double output = controller.calculateOutput(state, t);
    	velocity += output * dt;
    	state += velocity * dt;
    	
    	out << t << "," << target << "," << state << endl;
    }
     
    out.close();
    
    return abs(target - state) < 0.05;
}

/**
* Runs PID controller against a target which changes in a sinusoidal fashion.
*
* Full results are logged to test_results/sinusoid.csv.
*/
void testSinusoid() {
    //Write file header
    ofstream out;
    out.open("test_results/sinusoid.csv");
    out << "Time,Target,State" << endl;
    
    PIDParameters params {
    	k_p: 5,
    	k_i: 5,
    	k_d: 5
    };
    
    double t = 0;
    double dt = 0.01;
    double state = 0;
    double velocity = 0;
    double target = 0;
    
    PIDController controller(params);
    controller.initialize(target, state, t);
    
    for (int i = 0; i < 1000; i++) {
    	t += dt;
    	target = sin(t * 6.283185);
    	controller.updateTarget(target);
    	double output = controller.calculateOutput(state, t);
    	velocity += output * dt;
    	state += velocity * dt;
    	
    	out << t << "," << target << "," << state << endl;
    }
     
    out.close();
}

/**
* Runs all test cases.
*/
int main() {
    if (!testStepFunction()) {
    	return 1;
    }
    if (!testSquareWave()) {
    	return 1;
    }
    if (!testUnevenSquareWave()) {
    	return 1;
    }
    testSinusoid();
    
    return 0;
}
