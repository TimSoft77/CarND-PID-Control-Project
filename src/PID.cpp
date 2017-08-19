#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd, double i_err_thresh) {
	Kp = kp;
	Ki = ki;
	Kd = kd;
	i_error_threshold = i_err_thresh;
}

double PID::CalculateControlOutput(double cte) {
	if (!is_init) {
		i_error = cte;
		d_error = cte;
		is_init = true;
		return -Kp*cte - Ki*i_error;
	}
	else {
		if (fabs(cte) > i_error_threshold) {
			i_error = 0; // Leave out and reset i control if error too big
		}
		else {
			i_error += cte;
		}
		double output = -Kp*cte - Ki*i_error - Kd*(cte - d_error); // This implementation assumes a constant dt
		d_error = cte;
		return output;
	}
}

