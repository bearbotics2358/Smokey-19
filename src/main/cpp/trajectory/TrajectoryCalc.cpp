// TrajectoryCalc.cpp - helper class to calculate trajectories for the shooter

#include <units/math.h>
#include <stdio.h>

#include "trajectory/model.h"
#include "trajectory/trajectory_table_defs.h"
#include "trajectory/trajectory_data.h"
#include "trajectory/TrajectoryCalc.h"

void TrajectoryCalc::init()
{
	set_model(AIR_DRAG);
	set_shoot_on_the_move_enabled(true);
	set_constant_shooter_elevation(true);
	set_ball_compression(0.3950);

	// debug_found_it = false;

}

enum model_t TrajectoryCalc::set_model(enum model_t model)
{
	m_model = model;

	return m_model;
}

enum model_t TrajectoryCalc::get_model()
{
	return m_model;
}

void TrajectoryCalc::set_shoot_on_the_move_enabled(bool enable)
{
	m_shoot_on_the_move_enabled = enable;
}

void TrajectoryCalc::set_constant_shooter_elevation(bool enable)
{
	m_constant_shooter_elevation_enabled = enable;
}

double TrajectoryCalc::set_ball_compression(double compression)
{
	m_ball_compression = compression;

	return m_ball_compression;
}

double TrajectoryCalc::get_ball_compression()
{
	return m_ball_compression;
}


// NOTE: This code is based on the angle-vs-launch-speed distance table having angles that start at 0, go to 90 degrees, with
// a step of 1 degree
//
// looking at distances for 2 angles, and picking the one that is closer to the target distance
// - if the distance for the current angle is closer to the target distance, continue, using it as the closest so far
// -  else return the angle for the closest to target distance

// if it is not able to find an angle that will reach the target distance, it returns the angle for the longest shot possible

units::degree_t TrajectoryCalc::get_angle(units::foot_t distance, units::revolutions_per_minute_t rpm)
{
	units::degree_t theta_ret = 0_deg;
	int theta_index = 0;
	units::foot_t dist = 0_ft;
	units::foot_t dist_best = -99_ft;
	int theta_index_best = 0;
	units::feet_per_second_t v_launch = 0_fps;
	int v_launch_index = 0;
	bool found_first_value = false;
	units::foot_t dist_last = 0_ft;
	int theta_index_last = 0;
	double dist_error = 0;
	double dist_error_last = 0;
	double dist_error_delta = 0;
	double dist_error_delta_last = 0;

	// starting at 0 degrees and moving to higher angles, find the best match
	// stop once the table distance:
	// - error is worse that the best so far

	// look at distance error, and the delta to the distance error from the last distance
	// if the sign of the delta distance error changes from negative to positive, done
	// use the last value (lower angle)


	v_launch = wheel_rpm_to_v_launch(rpm);
	v_launch_index = get_v_launch_index(v_launch);

	found_first_value = false;
	for(theta_index = 0; theta_index <= TRAJECTORY_TABLE_ANGLES - 1; theta_index++) {
		dist = units::foot_t(data.distance[m_model][theta_index][v_launch_index]);
		if(dist.value() == 0.0) {
			// skip entries that don't go as high as the hub
			continue;
		}

		if(found_first_value == false) {
			// found 1st non-zero distance value
			found_first_value = true;

			theta_index_best = theta_index;
			dist_best = dist;

			theta_index_last = theta_index;
			dist_last = dist;
			dist_error = units::math::fabs(distance - dist).value();
			dist_error_last = 0;
			dist_error_delta = 0;
			dist_error_delta_last = 0;
			// go again
			continue;
		}

		/*
		*********************************************************************************************

		If search inital distance, after the "0's":
		- check the distance
		- if won't go in (or maybe 1/2 that?)
		-- start the reverse direction (like we used to, from 90 to 0)


		*/



		/*
		if(rpm.value() < 10) {
			printf("  in get_angle: theta: %2d distance-dist %6.3lf distance-dist_best: %6.3lf distance: % 6.3lf\n", theta_indx,
				units::math::fabs(distance-dist) + 0.01_ft, units::math::fabs(distance-dist_best), distance);
		}
		*/

		// look for best match from 2 adjacent entries in the table
		// now see which is closer

		// look for sign change from negative to positive in dist_error_delta
		dist_error = units::math::fabs(distance - dist).value();
		dist_error_delta = dist_error - dist_error_last;
		/*
		if(!debug_found_it) {
			printf("theta: %3d  dist: %6.2lf  dist_error: %6.2lf  dist_error_delta: %6.2lf  theta_last: %3d  dist_last: %6.2lf  dist_error_last: %6.2lf  dist_error_delta_last: %6.2lf\n", 
							theta_index, dist.value(), dist_error, dist_error_delta, theta_index_last, dist_last.value(), dist_error_last, dist_error_delta_last);
		}
		*/
		if((dist_error_delta > 0) && (dist_error_delta_last < 0)) {
			// we're done, use previous value
			theta_ret = units::degree_t(theta_index_last);
			// printf("found it\n");
			// debug_found_it = 1;
			break;
		}
		dist_last = dist;
		dist_error_last = dist_error;
		dist_error_delta_last = dist_error_delta;
		theta_index_last = theta_index;
		continue;

		/*
		if(units::math::fabs(distance - dist) + 0.01_ft < units::math::fabs(distance - dist_best)) { // 0.01 'cause comparing floating point numbers
			// this one is better
			theta_index_best = theta_index;
			dist_best = dist;
			// go again
			continue;
		} else {
			// this one is worse, use the previous one
			theta_ret = units::degree_t(theta_index_best);
			// done
			break;
		}
		*/
	}

	return theta_ret;
}

TrajectoryInfo TrajectoryCalc::compute_trajectory(TrajectoryInfo inputs)
{
	units::degree_t elevation = 0_deg;
	units::foot_t dist = 0_ft;
	int v_launch_index = 0;
	int theta_index = 0;
	units::foot_t dist_error = 0_ft;

	inputs.return_value = TRAJECTORY_SUCCESS;

	// will the current values result in a successful shot?
	elevation = inputs.elevation_angle;
	theta_index = (int)elevation.value();
	v_launch_index = wheel_rpm_to_v_launch_index(inputs.wheel_rpm);
	dist = units::foot_t(data.distance[m_model][theta_index][v_launch_index]);

	if(units::math::fabs(dist - inputs.distance) < SUCCESSFUL_SHOT_RADIUS) {
		inputs.return_value = TRAJECTORY_SUCCESS;
	} else if(dist < inputs.distance) {
		inputs.return_value = TRAJECTORY_FAILURE_SHOT_SHORT;
	} else {
		inputs.return_value = TRAJECTORY_FAILURE_SHOT_LONG;
	}

	// Step 1 - compute the elevation angle with no limits
	if(m_constant_shooter_elevation_enabled == 0) {
		elevation = get_angle(inputs.distance, inputs.wheel_rpm);
		/*
		if(inputs.wheel_rpm.value() < 10) {
			printf("  get_angle: %lf input: %lf\n", elevation.value(), inputs.elevation_angle.value());
		}
			*/
		inputs.elevation_angle = elevation;
		theta_index = (int)elevation.value();
		v_launch_index = wheel_rpm_to_v_launch_index(inputs.wheel_rpm);
		dist = units::foot_t(data.distance[m_model][theta_index][v_launch_index]);
	}

	// Step 2 - update values based on robot moving at speed
	if(m_shoot_on_the_move_enabled && (units::math::fabs(inputs.vx) > ROBOT_SPEED_THRESHOLD) || (units::math::fabs(inputs.vy) > ROBOT_SPEED_THRESHOLD)) {
		// compensate the elevation angle, turret angle, and wheel speed due to robot moving at speed
	}

	// Step 3 - adjust for elevation angle limits or
	if(m_constant_shooter_elevation_enabled || ((elevation >= ELEVATION_ANGLE_MIN) && (elevation <= ELEVATION_ANGLE_MAX))) {
		// no adjustment needed for angle
		// check if distance is ok

		dist = units::foot_t(data.distance[m_model][theta_index][v_launch_index]);

		if(m_constant_shooter_elevation_enabled || (units::math::fabs(inputs.distance - dist) > SUCCESSFUL_SHOT_RADIUS)) {
			// shot is too short or too long to go into hub
			// for now, go to constant angle

			if(m_constant_shooter_elevation_enabled) {
				elevation = inputs.elevation_angle;
				theta_index = (int)elevation.value();
			} else {
				// set the shot to a mid angle and look from lowest RPM to highest
				// looking at table, this will find a solution for 7 ft to 20 ft
				elevation = ELEVATION_ANGLE_MID;
				theta_index = (int)elevation.value();
				inputs.elevation_angle = elevation;
			}

			// now step thru RPM's to find the best shot
			// printf("inputs.wheel_rpm: %lf  rpm index: %d\n", inputs.wheel_rpm.value(), get_rpm_index(inputs.wheel_rpm));

			inputs = find_best_launch_speed(inputs, theta_index);
		}

	} else {
		// elevation angle not acceptable
		// adjust motor speed to bring elevation angle within limits
		dist = units::foot_t(data.distance[m_model][theta_index][v_launch_index]);

		elevation = ELEVATION_ANGLE_MID;
		theta_index = (int)elevation.value();
		inputs.elevation_angle = elevation;
		// printf("inputs.wheel_rpm: %lf  rpm index: %d\n", inputs.wheel_rpm.value(), get_rpm_index(inputs.wheel_rpm));

		inputs = find_best_launch_speed(inputs, theta_index);

	}

	// NOTE: not sure this is correct for all cases
	v_launch_index = wheel_rpm_to_v_launch_index(inputs.wheel_rpm);
	inputs.t = (units::second_t)(data.t[m_model][(int)inputs.elevation_angle.value()][v_launch_index]);

	return inputs;
}

// this only changes launch speed (wheel speed), not launch angle theta
TrajectoryInfo TrajectoryCalc::find_best_launch_speed(TrajectoryInfo inputs, int theta_index)
{
	units::foot_t dist_best = -99_ft;
	int v_launch_index_best = 0;
	int v_launch_index = 0;
	units::foot_t dist = 0_ft;

	dist_best = -99_ft;
	v_launch_index_best = 0;
	// NOTE: looking at the data table, @ 65 degrees this will always find a solution before hitting the max RPM value in the table
	// therefore no special action is taken for not finding a solution
	for(v_launch_index = 0; v_launch_index < TRAJECTORY_TABLE_SPEEDS; v_launch_index++) {
		// printf("theta: %d   rpm: %lf   dist: %lf\n", theta_indx, (rpm_indx * 25 + 1000.0), data.distance[m_model][theta_indx][rpm_indx]);
		dist = units::foot_t(data.distance[m_model][theta_index][v_launch_index]);
		// printf("rpm_indx: %d dist: %lf\n", rpm_indx, dist.value());
		if(dist.value() == 0.0) {
			continue;
		}

		// look for best match from 2 adjacent entries in the table
		// now see which is closer
		if(units::math::fabs(inputs.distance - dist) < units::math::fabs(inputs.distance - dist_best) + 0.01_ft) { // 0.01 'cause comparing floating point numbers
			// this one is better
			v_launch_index_best = v_launch_index;
			dist_best = dist;
			inputs.wheel_rpm = v_launch_index_to_wheel_rpm(v_launch_index_best);
			// go again
			continue;

		} else {
			// this one is worse, use the previous one
			inputs.wheel_rpm = v_launch_index_to_wheel_rpm(v_launch_index_best);
			v_launch_index = v_launch_index_best;
			// done
			break;
		}
	}

	return inputs;
}

// get index into the angle_vs_speed table for the rpm column for the rpm that is <= to input
int TrajectoryCalc::get_v_launch_index(units::feet_per_second_t v_launch) {
	return get_v_launch_index(v_launch.value());
}

// get index into the angle_vs_speed table for the velocity launch column for the velocity that is <= to input
int TrajectoryCalc::get_v_launch_index(double v_launch)
{
	int index = 0;
	double v_launch_max = TRAJECTORY_V_LAUNCH_BASE + (TRAJECTORY_TABLE_SPEEDS - 1) * TRAJECTORY_V_LAUNCH_INC;

	if(v_launch > v_launch_max) {
		v_launch = v_launch_max;
	}

	index = (v_launch - TRAJECTORY_V_LAUNCH_BASE) / (1.0 * TRAJECTORY_V_LAUNCH_INC);

	if(index < 0) {
		index = 0;
	}

	return index;
}

units::feet_per_second_t TrajectoryCalc::v_launch_index_to_value(int v_launch_index) {
	units::feet_per_second_t v_launch;

	v_launch = (units::feet_per_second_t)(TRAJECTORY_V_LAUNCH_BASE + v_launch_index * TRAJECTORY_V_LAUNCH_INC);
	return v_launch;
}


units::feet_per_second_t TrajectoryCalc::wheel_rpm_to_v_launch(units::revolutions_per_minute_t rpm) {
	// convert to tangential speed of surface of the wheel
	// then multiply by the ball compression to get the linear launch speed of the ball
	units::feet_per_second_t v = (units::feet_per_second_t)(M_PI * units::foot_t(WHEEL_DIAMETER).value() * rpm.value() / 60.0 * m_ball_compression);

	return v;
}

int TrajectoryCalc::wheel_rpm_to_v_launch_index(units::revolutions_per_minute_t rpm) {
	units::feet_per_second_t v_launch = wheel_rpm_to_v_launch(rpm);
	int v_launch_index = get_v_launch_index(v_launch);

	return v_launch_index;
}

units::revolutions_per_minute_t TrajectoryCalc::v_launch_index_to_wheel_rpm(int v_launch_index) {
	units::feet_per_second_t v_launch = v_launch_index_to_value(v_launch_index);

	// divide this by the ball compression to get the tangential speed of surface of the wheel
	// then convert to the RPM for the wheel
	units::revolutions_per_minute_t rpm = units::revolutions_per_minute_t(v_launch.value() / m_ball_compression /  (M_PI * units::foot_t(WHEEL_DIAMETER).value()) * 60.0);

	return rpm;
}