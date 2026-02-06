// TrajectoryCalc.h - helper class to calculate trajectories for the shooter

#include "TrajectoryTable.h"

class TrajectoryCalc {

 public:

	TrajectoryCalc() {}
	~TrajectoryCalc() {}
	
	void init();
	double get_angle(double distance, double rpm);
	int get_rpm_index(double rpm);

 private:
	double m_wheel_diameter;
	TrajectoryTable table;

} ;
