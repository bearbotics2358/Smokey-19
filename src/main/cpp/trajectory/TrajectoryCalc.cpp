// TrajectoryCalc.cpp - helper class to calculate trajectories for the shooter

#include <units/math.h>
#include "trajectory/TrajectoryCalc.h"

void TrajectoryCalc::init()
{
	table.init();
}

// NOTE: This code is based on the anglevsdistance table having angles that start at 0, go to 90 degrees, with
// a step of 1 degree
//
// looking at distances for 2 angles, and picking the one that is closer to the target distance
// - if the distance for the current angle is closer to the target distance, continue, using it as the closest so far
// -  else return the angle for the closest to target distance

// if it is not able to find an angle that will reach the target distance, it returns the angle for the longest shot possible

// CONSIDER
// perhaps return success/failure/reason code, and angle as a parameter??
// and return angle for longest shot possible??
// perhaps return best angle and also return an error result, or predicted distance??
units::degree_t TrajectoryCalc::get_angle(units::foot_t distance, units::revolutions_per_minute_t rpm)
{
	int rpm_indx = get_rpm_index(rpm.value());
	units::degree_t theta_ret = 0_deg;
	int theta_indx = 0;
	units::foot_t dist = 0_ft;
	units::foot_t dist_best = -99_ft;
	int theta_indx_best = 0;

	// starting at 90 degrees and moving to lower angles, find the best match
	// stop once the table distance:
	// - is shorter than the previous

	for(theta_indx = 90; theta_indx >= 0; theta_indx -= 1) {
		dist = units::foot_t(table.data[theta_indx][rpm_indx]);

		// look for best match from 2 adjacent entries in the table
		// now see which is closer
		if(units::math::fabs(distance - dist) < units::math::fabs(distance - dist_best) + 0.01_ft) { // 0.01 'cause comparing floating point numbers
			// this one is better
			theta_indx_best = theta_indx;
			dist_best = dist;
			// go again
			continue;
		} else {
			// this one is worse, use the previous one
			theta_ret = units::degree_t(theta_indx_best);
			// done
			break;
		}
	}

	return theta_ret;
}

// get index into the angle_vs_speed table for the rpm column for the rpm that is <= to input
int TrajectoryCalc::get_rpm_index(double rpm)
{
	int indx = 0;

	if(rpm > table.get_rpm_max()) {
		rpm = table.get_rpm_max();
	}

	indx = (rpm - table.get_rpm_min()) / table.get_rpm_inc();

	if(indx < 0) {
		indx = 0;
	}

	return indx;
}

