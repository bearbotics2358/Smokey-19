#include <iostream>

#include "FieldConstants.h"
#include "LaunchHelper.h"
#include "bearlog/bearlog.h"
#include <frc/Timer.h>

LaunchHelper& LaunchHelper::GetInstance() {
    // Defining a static LaunchHelper object so it will only be created once
    static LaunchHelper instance;
    return instance;
}

LaunchHelper::LaunchHelper()
    : m_LastCacheTime{frc::Timer::GetFPGATimestamp()}
{
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

    units::second_t timestamp = frc::Timer::GetFPGATimestamp();
    if (timestamp - m_LastCacheTime < kCacheTimeout) {
        // Return the most recently calculated values until the cache is considered stale (after kCacheTimeout)
        return m_Cache;
    }

    m_LastCacheTime = timestamp;

    inputs.elevation_angle = m_GetHoodAngle();
    inputs.wheel_rpm = m_GetShooterSpeed();
    inputs.turret_angle = m_GetTurretAngle();

    frc::Translation2d hub_center = FieldConstants::GetHubCenterForMyAlliance();
    units::meter_t distance_to_hub_center = units::math::abs(m_RobotPoseSupplier().Translation().Distance(hub_center));
    inputs.distance = distance_to_hub_center;

    BearLog::Log("Distance to Hub", distance_to_hub_center);

    // @todo When we're ready for shoot on the move, these should be
    //  updated to use m_RobotSpeedsSupplier().vx and m_RobotSpeedsSupplier().vy
    inputs.vx = 0_fps;
    inputs.vy = 0_fps;

    m_Cache = m_TrajectoryCalc.compute_trajectory(inputs);

    return m_Cache;
}