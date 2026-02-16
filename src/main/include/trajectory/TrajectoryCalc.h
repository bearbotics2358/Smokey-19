// TrajectoryCalc.h - helper class to calculate trajectories for the shooter

#include "trajectory/TrajectoryTable.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#define ELEVATION_ANGLE_MAX 75_deg
#define ELEVATION_ANGLE_MIN 55_deg

#define ROBOT_SPEED_THRESHOLD 0.01_fps  // minimum speed to enable speed compensation
#define ROBOT_MAX_SPEED 19.9_fps // worst case based on Kraken X60, no FOC, 6000 RPM, R3

#define HUB_DIAMETER 41.7_in
#define FUEL_DIAMETER 6_in
#define SUCCESSFUL_SHOT_RADIUS ((HUB_DIAMETER - FUEL_DIAMETER)/2.0)

enum return_value_t {
	TRAJECTORY_SUCCESS = 0,
	TRAJECTORY_FAILURE_SHOT_SHORT = 1,
	TRAJECTORY_FAILURE_SHOT_LONG = 2,
	TRAJECTORY_FAILURE_SHOT_LEFT = 3,
	TRAJECTORY_FAILURE_SHOT_RIGHT = 4,
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
	int get_rpm_index(units::revolutions_per_minute_t rpm);
	int get_rpm_index(double rpm);
	units::revolutions_per_minute_t get_rpm(int rpm_index);

 private:
	TrajectoryTable table;

} ;
