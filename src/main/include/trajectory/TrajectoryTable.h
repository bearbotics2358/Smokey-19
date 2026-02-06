// TrjectoryTable.h - class to hold the table of angle versus speed (actually wheel RPM) for the shooter


// hard coding the table to angles of 0 to 90 degrees inclusive with a step size of 1,
// and speeds (wheel RPMs) from 1000 to 4000 inclusive with a step size of 25
#define TRAJECTORY_TABLE_ANGLES 91
#define TRAJECTORY_TABLE_SPEEDS 121

#define GRAVITY 32.17

class TrajectoryTable {

 public:

	TrajectoryTable(double wheel_diameter_in = 4.0, double shooter_height_in = 17.15, double hub_height_in = 72.0);
	~TrajectoryTable() {}

	void init();
	int load_file();
	void set_wheel_diameter(double wheel_diameter_in);
	void set_shooter_height_in(double shooter_height_in);
	void set_hub_height(double hub_height_in);
	double get_wheel_diameter();
	double get_shooter_height_in();
	double get_hub_height();
	int get_rpm_min();
	int get_rpm_max();
	int get_rpm_inc();

	double data[TRAJECTORY_TABLE_ANGLES][TRAJECTORY_TABLE_SPEEDS];

 private:

	double m_wheel_diameter_in;
	double m_shooter_height_in;
	double m_hub_height_in;

	int m_theta_min;
	int m_theta_max;
	int m_theta_inc;

	int m_rpm_min;
	int m_rpm_max;
	int m_rpm_inc;

} ;
