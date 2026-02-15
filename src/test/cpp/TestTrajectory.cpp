#include <gtest/gtest.h>

#include "trajectory/TrajectoryCalc.h"

struct TrajectoryParams {
    units::foot_t distance;
    units::revolutions_per_minute_t rpm;
    double expected_angle_deg;
};

struct TrajectoryParamsCompensated {
    units::foot_t distance;
    units::degree_t angle;
    units::revolutions_per_minute_t rpm;
    units::degree_t expected_angle;
    units::revolutions_per_minute_t expected_rpm;    
};

class TestTrajectory : public testing::TestWithParam<TrajectoryParams> {
protected:
    TrajectoryCalc m_TrajectoryCalc;

public:
    TestTrajectory() {
        m_TrajectoryCalc.init();
    }
};

class TestTrajectoryCompensated : public testing::TestWithParam<TrajectoryParamsCompensated> {
protected:
    TrajectoryCalc m_TrajectoryCalc;

public:
    TestTrajectoryCompensated() {
        m_TrajectoryCalc.init();
    }
};

TEST_P(TestTrajectory, ValidateAnglesFromTrajectoryTableWithKnownDistancesAndRPMs) {
    TrajectoryParams params = GetParam();

    units::degree_t angle = m_TrajectoryCalc.get_angle(params.distance, params.rpm);

    EXPECT_DOUBLE_EQ(angle.value(), params.expected_angle_deg);
}

INSTANTIATE_TEST_SUITE_P(
    TrajectoryDistanceTest,
    TestTrajectory,
    ::testing::Values(
        // Include a small set of ranges for distance and rpms for testing. If testing on the real robot shows that
        // the trajectory calculations need to be adjusted, these may need to change (so we should avoid adding
        // a large number of them). We want a representative set to ensure that any possible code optimizations
        // (like for lookup or calculation speed) to the TrajectoryTable and TrajectoryCalc classes will give us the
        // same result.
        TrajectoryParams{5_ft, 3600_rpm, 85},
        TrajectoryParams{8_ft, 3600_rpm, 82},
        TrajectoryParams{11_ft, 3600_rpm, 78},
        TrajectoryParams{20_ft, 3600_rpm, 67},
        TrajectoryParams{5_ft, 3200_rpm, 83},
        TrajectoryParams{8_ft, 3200_rpm, 79},
        TrajectoryParams{11_ft, 3200_rpm, 74},
        TrajectoryParams{20_ft, 3200_rpm, 52}
    )
);

TEST_P(TestTrajectoryCompensated, ValidateAnglesFromTrajectoryTableWithKnownDistancesAndRPMsCompensated) {
    TrajectoryParamsCompensated params = GetParam();

    struct TrajectoryInfo trajInfo;
    
    /*
    // TrajectoryInfo
	units::degree_t elevation_angle;
	units::revolutions_per_minute_t wheel_rpm;
	units::foot_t distance; // from shooter to the center of the hub
	units::feet_per_second_t vx; // field relative
	units::feet_per_second_t vy; // field relative
	units::degree_t turret_angle; // field relative
	units::degree_t hub_angle; // field relative
	enum return_value_t return_value;

    // TrajectoryParamsCompensated
    units::foot_t distance;
    units::degree_t angle;
    units::revolutions_per_minute_t rpm;
    units::degree_t expected_angle;
    units::revolutions_per_minute_t expected_rpm;    
    */

    trajInfo.distance = params.distance;
    trajInfo.elevation_angle = params.angle;
    trajInfo.wheel_rpm = params.rpm;
 
    trajInfo = m_TrajectoryCalc.compute_trajectory(trajInfo);

    EXPECT_DOUBLE_EQ(trajInfo.elevation_angle.value(), params.expected_angle.value());
    EXPECT_DOUBLE_EQ(trajInfo.wheel_rpm.value(), params.expected_rpm.value());
}

INSTANTIATE_TEST_SUITE_P(
    TrajectoryDistanceTestCompensated,
    TestTrajectoryCompensated,
    ::testing::Values(
        // Include a small set of ranges for distance and rpms for testing. If testing on the real robot shows that
        // the trajectory calculations need to be adjusted, these may need to change (so we should avoid adding
        // a large number of them). We want a representative set to ensure that any possible code optimizations
        // (like for lookup or calculation speed) to the TrajectoryTable and TrajectoryCalc classes will give us the
        // same result.
        TrajectoryParamsCompensated{20_ft, 67_deg, 3600_rpm, 67_deg, 3600_rpm},
        TrajectoryParamsCompensated{20_ft, 52_deg, 3200_rpm, 55_deg, 3275_rpm},
        TrajectoryParamsCompensated{7_ft, 65_deg, 3600_rpm, 75_deg, 2675_rpm},
        TrajectoryParamsCompensated{20_ft, 65_deg, 3275_rpm, 55_deg, 3275_rpm},
        TrajectoryParamsCompensated{20_ft, 65_deg, 2200_rpm, 55_deg, 3275_rpm} // causes a problem 'cause 55 deg 2200 RPM is 0 part of table
    )
);

