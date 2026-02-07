// TrajectoryCalc.h - helper class to calculate trajectories for the shooter

#include "trajectory/TrajectoryTable.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

class TrajectoryCalc {

 public:

	TrajectoryCalc() {}
	~TrajectoryCalc() {}

	void init();
	units::degree_t get_angle(units::foot_t distance, units::revolutions_per_minute_t rpm);
	int get_rpm_index(double rpm);

 private:
	TrajectoryTable table;

} ;
