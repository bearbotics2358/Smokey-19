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

TrajectoryInfo TrajectoryCalc::compute_trajectory(TrajectoryInfo inputs) 
{
	units::degree_t elevation = 0_deg;
	units::foot_t dist = 0_ft;
	units::foot_t dist_best = -99_ft;
	int rpm_indx = 0;
	int rpm_indx_best = 0;
	int theta_indx = 0;
	units::foot_t dist_error = 0_ft;

	inputs.return_value = TRAJECTORY_SUCCESS;

	// will the current values result in a successful shot?
	elevation = inputs.elevation_angle;
	theta_indx = (int)elevation.value();
	rpm_indx = get_rpm_index(inputs.wheel_rpm);
	dist = units::foot_t(table.data[theta_indx][rpm_indx]);

	if(units::math::fabs(dist - inputs.distance) < SUCCESSFUL_SHOT_RADIUS) {
		inputs.return_value = TRAJECTORY_SUCCESS;
	} else if(dist < inputs.distance) {
		inputs.return_value = TRAJECTORY_FAILURE_SHOT_SHORT;
	} else {
		inputs.return_value = TRAJECTORY_FAILURE_SHOT_LONG;
	}

	// Step 1 - compute the elevation angle with no limits
	elevation = get_angle(inputs.distance, inputs.wheel_rpm);
	inputs.elevation_angle = elevation;
	theta_indx = (int)elevation.value();
	rpm_indx = get_rpm_index(inputs.wheel_rpm);
	dist = units::foot_t(table.data[theta_indx][rpm_indx]);

	// Step 2 - update values based on robot moving at speed
	if((units::math::fabs(inputs.vx) > ROBOT_SPEED_THRESHOLD) || (units::math::fabs(inputs.vy) > ROBOT_SPEED_THRESHOLD)) {
		// compensate the elevation angle, turret angle, and wheel speed due to robot moving at speed
	} 

	// Step 3 - adjust for elevation angle limits or 
	if((elevation >= ELEVATION_ANGLE_MIN) && (elevation <= ELEVATION_ANGLE_MAX)) {
		// no adjustment needed for angle
		// check if distance is ok

		dist = units::foot_t(table.data[theta_indx][rpm_indx]);
		if((inputs.distance - dist) > SUCCESSFUL_SHOT_RADIUS) {
			// shot is too short to go into hub

			// set the shot to a mid angle and look from lowest RPM to highest
			// looking at table, this will find a solution for 7 ft to 20 ft
			elevation = ELEVATION_ANGLE_MID;
			theta_indx = (int)elevation.value();
			inputs.elevation_angle = elevation;

			// now step thru RPM's to find the best shot
			printf("inputs.wheel_rpm: %lf  rpm index: %d\n", inputs.wheel_rpm.value(), get_rpm_index(inputs.wheel_rpm));

			dist_best = -99_ft;
			rpm_indx_best = 0;
			// NOTE: looking at the data table, this will always find a solution before hitting the max RPM value in the table
			// therefore no special action is taken for not finding a solution
			for(rpm_indx = 0; rpm_indx < TRAJECTORY_TABLE_SPEEDS; rpm_indx++) {
				printf("theta: %d   rpm: %lf   dist: %lf\n", theta_indx, (rpm_indx * 25 + 1000.0), table.data[theta_indx][rpm_indx]);
				dist = units::foot_t(table.data[theta_indx][rpm_indx]);		
				printf("rpm_indx: %d dist: %lf\n", rpm_indx, dist.value());

				// look for best match from 2 adjacent entries in the table
				// now see which is closer
				if(units::math::fabs(inputs.distance - dist) < units::math::fabs(inputs.distance - dist_best) + 0.01_ft) { // 0.01 'cause comparing floating point numbers
					// this one is better
					rpm_indx_best = rpm_indx;
					dist_best = dist;
					// go again
					continue;
	
				} else {
					// this one is worse, use the previous one
					inputs.wheel_rpm = get_rpm(rpm_indx_best);
					// done
					break;
				}
			}
		}
		
	} else {
		// adjust motor speed to bring elevation angle within limits
		dist = units::foot_t(table.data[theta_indx][rpm_indx]);
		if((elevation < ELEVATION_ANGLE_MIN)) {
			// need to speed up motor

			// search the table, at the minimum elevation angle, starting at the current wheel speed and moving to higher speeds,
			// find the best match
			// stop once the table distance error:
			// - is worse than the previous

			// starting at ELEVATION_ANGLE_MIN can end up on the wrong side of the balistic arc
			// starting at ELEVATION_ANGLE_MID looks to be on the correct side of the arc
			elevation = ELEVATION_ANGLE_MID;
			theta_indx = (int)elevation.value();
			inputs.elevation_angle = elevation;
			printf("inputs.wheel_rpm: %lf  rpm index: %d\n", inputs.wheel_rpm.value(), get_rpm_index(inputs.wheel_rpm));

			dist_best = -99_ft;
			rpm_indx_best = 0;
			// NOTE: looking at the data table, this will always find a solution before hitting the max RPM value in the table
			// therefore no special action is taken for not finding a solution
			for(rpm_indx = get_rpm_index(inputs.wheel_rpm); rpm_indx < TRAJECTORY_TABLE_SPEEDS; rpm_indx++) {
				printf("theta: %d   rpm: %lf   dist: %lf\n", theta_indx, (rpm_indx * 25 + 1000.0), table.data[theta_indx][rpm_indx]);
				dist = units::foot_t(table.data[theta_indx][rpm_indx]);		
				printf("rpm_indx: %d dist: %lf\n", rpm_indx, dist.value());

				// look for best match from 2 adjacent entries in the table
				// now see which is closer
				if(units::math::fabs(inputs.distance - dist) < units::math::fabs(inputs.distance - dist_best) + 0.01_ft) { // 0.01 'cause comparing floating point numbers
					// this one is better
					rpm_indx_best = rpm_indx;
					dist_best = dist;
					// go again
					continue;
				} else {
					// this one is worse, use the previous one
					inputs.wheel_rpm = get_rpm(rpm_indx_best);
					// done
					break;
				}
			}


		} else if((elevation > ELEVATION_ANGLE_MAX)){
			// need to slow down motor

			// search the table, at the maximum elevation angle, starting at the current wheel speed and moving to slower speeds,
			// find the best match
			// stop once the table distance error:
			// - is worse than the previous

			elevation = ELEVATION_ANGLE_MAX;
			theta_indx = (int)elevation.value();
			inputs.elevation_angle = elevation;

			dist_best = -99_ft;
			rpm_indx_best = 0;
			// NOTE: looking at the data table, this will always find a solution before hitting the min RPM values in the table
			// therefore no special action is taken for not finding a solution
			for(rpm_indx = get_rpm_index(inputs.wheel_rpm); rpm_indx >= 0 ; rpm_indx--) {
				dist = units::foot_t(table.data[theta_indx][rpm_indx]);		

				// look for best match from 2 adjacent entries in the table
				// now see which is closer
				if(units::math::fabs(inputs.distance - dist) < units::math::fabs(inputs.distance - dist_best) + 0.01_ft) { // 0.01 'cause comparing floating point numbers
					// this one is better
					rpm_indx_best = rpm_indx;
					dist_best = dist;
					// go again
					continue;
				} else {
					// this one is worse, use the previous one
					inputs.wheel_rpm = get_rpm(rpm_indx_best);
					// done
					break;
				}
			}
		}
	}

	return inputs;
}

// get index into the angle_vs_speed table for the rpm column for the rpm that is <= to input
int TrajectoryCalc::get_rpm_index(units::revolutions_per_minute_t rpm) {
	return get_rpm_index(rpm.value());
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

units::revolutions_per_minute_t TrajectoryCalc::get_rpm(int rpm_index) {
	units::revolutions_per_minute_t ret;

	ret = (units::revolutions_per_minute_t)(table.get_rpm_min() + rpm_index * table.get_rpm_inc());
	return ret;
}