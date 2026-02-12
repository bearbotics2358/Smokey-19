#include <gtest/gtest.h>

#include "trajectory/TrajectoryCalc.h"

class TestTrajectoryResult : public testing::Test {
protected:
    TrajectoryCalc m_TrajectoryCalc;

public:
    TestTrajectoryResult() {
        m_TrajectoryCalc.init();
    }
};

TEST_F(TestTrajectoryResult, ReturnStructByValue) {
    // It can run the loop many times without causing problems with the stack
    for (int ii = 0; ii < 10000; ii++) {
        TrajectoryInfo inputs;
        inputs.angle = 40_deg;
        inputs.rpm = 2500_rpm;
        inputs.distance = 10_ft;

        // The output object is created here on the stack and the return value from compute_trajectory_info gets copied into it
        // so there's no heap allocation.
        TrajectoryInfo output = m_TrajectoryCalc.compute_trajectory_info(inputs);

        EXPECT_DOUBLE_EQ(output.angle.value(), 47);
        EXPECT_DOUBLE_EQ(output.rpm.value(), 3500);
        EXPECT_DOUBLE_EQ(output.distance.value(), 10);
        EXPECT_TRUE(output.will_succeed);
    }
}

struct TrajectoryParams {
    units::foot_t distance;
    units::revolutions_per_minute_t rpm;
    double expected_angle_deg;
};

class TestTrajectory : public testing::TestWithParam<TrajectoryParams> {
protected:
    TrajectoryCalc m_TrajectoryCalc;

public:
    TestTrajectory() {
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
        TrajectoryParams{11_ft, 3200_rpm, 75},
        TrajectoryParams{20_ft, 3200_rpm, 51}
    )
);