#pragma once

#include <functional>

#include "trajectory/TrajectoryCalc.h"
#include <frc/kinematics/ChassisSpeeds.h>

class LaunchHelper {
public:
    static LaunchHelper& GetInstance();
    void Init(std::function<units::degree_t()> hoodAngleSupplier,
              std::function<units::revolutions_per_minute_t()> shooterSpeedSupplier,
              std::function<units::degree_t()> turretAngleSupplier,
              std::function<frc::ChassisSpeeds()> robotSpeedsSupplier,
              std::function<frc::Pose2d()> robotPoseSupplier);

    TrajectoryInfo GetLaunchParameters();

    // Delete the copy constructor. LaunchHelper should not be cloneable since it is a singleton.
    LaunchHelper(const LaunchHelper&) = delete;

    // This is a singleton class so prevent assigning a LaunchHelper object
    LaunchHelper& operator=(const LaunchHelper&) = delete;

private:
    LaunchHelper();

private:
    bool m_Initialized = false;
    TrajectoryCalc m_TrajectoryCalc;

    std::function<units::degree_t()> m_GetHoodAngle;
    std::function<units::revolutions_per_minute_t()> m_GetShooterSpeed;
    std::function<units::degree_t()> m_GetTurretAngle;
    std::function<frc::ChassisSpeeds()> m_RobotSpeedsSupplier;
    std::function<frc::Pose2d()> m_RobotPoseSupplier;
};