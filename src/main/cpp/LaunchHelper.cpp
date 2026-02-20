#include <iostream>

#include "LaunchHelper.h"

LaunchHelper& LaunchHelper::GetInstance() {
    // Defining a static LaunchHelper object so it will only be created once
    static LaunchHelper instance;
    return instance;
}

LaunchHelper::LaunchHelper() {
}

void LaunchHelper::Init(std::function<units::degree_t()> hoodAngleSupplier,
                        std::function<units::revolutions_per_minute_t()> shooterSpeedSupplier,
                        std::function<units::degree_t()> turretAngleSupplier,
                        std::function<frc::ChassisSpeeds()> robotSpeedsSupplier,
                        std::function<frc::Pose2d()> robotPoseSupplier) {
    m_GetHoodAngle = hoodAngleSupplier;
    m_GetShooterSpeed = shooterSpeedSupplier;
    m_GetTurretAngle = turretAngleSupplier;
    m_RobotSpeedsSupplier = robotSpeedsSupplier;
    m_RobotPoseSupplier = robotPoseSupplier;

    m_TrajectoryCalc.init();

    m_Initialized = true;
}

TrajectoryInfo LaunchHelper::GetLaunchParameters() {
    TrajectoryInfo inputs;

    if (false == m_Initialized) {
        std::cout << "[ERROR] LaunchHelper::GetLaunchParameters called while uninitialized. Did you forget to call Init?" << std::endl;
        return inputs;
    }

    inputs.elevation_angle = m_GetHoodAngle();
    inputs.wheel_rpm = m_GetShooterSpeed();
    inputs.turret_angle = m_GetTurretAngle();

    // @todo When we're ready for shoot on the move, these should be
    //  updated to use m_RobotSpeedsSupplier().vx and m_RobotSpeedsSupplier().vy
    inputs.vx = 0_fps;
    inputs.vy = 0_fps;

    return m_TrajectoryCalc.compute_trajectory(inputs);
}