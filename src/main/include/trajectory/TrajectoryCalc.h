// TrajectoryCalc.h - helper class to calculate trajectories for the shooter

#include "trajectory/TrajectoryTable.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#define ELEVATION_ANGLE_MAX 75_deg
#define ELEVATION_ANGLE_MIN 55_deg

#define ROBOT_SPEED_THRESHOLD 0.01_fps  // minimum speed to enable speed compensation

enum return_value_t {
	TRAJECTORY_SUCCESS = 0,
	TRAJECTORY_FAILURE_WHEEL_SPEED_LOW = 1,
	TRAJECTORY_FAILURE_WHEEL_SPEED_HIGH = 2,
	TRAJECTORY_FAILURE_YAW_LOW = 3,
	TRAJECTORY_FAILURE_YAW_HIGH = 4,
} ;

struct TrajectoryInfo {
	units::degree_t elevation_angle;
	units::revolutions_per_minute_t wheel_rpm;
	units::foot_t distance; // from shooter to the center of the hub
	units::feet_per_second_t vx; // field relative
	units::feet_per_second_t vy; // field relative
	units::degree_t turret_angle; // field relative
	units::degree_t hub_angle; // field relative
	enum return_value_t return_value;
};

class TrajectoryCalc {

 public:

	TrajectoryCalc() {}
	~TrajectoryCalc() {}

	void init();
	units::degree_t get_angle(units::foot_t distance, units::revolutions_per_minute_t rpm);
	TrajectoryInfo compute_trajectory(TrajectoryInfo inputs);
	int get_rpm_index(double rpm);

 private:
	TrajectoryTable table;

} ;
