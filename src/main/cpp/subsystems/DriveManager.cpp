#include "subsystems/DriveManager.h"
#include "bearlog/bearlog.h"

DriveManager::DriveManager(std::function<frc::Pose2d()> getBotPose) :
    m_GetCurrentBotPose(getBotPose)
{}

void DriveManager::Periodic() {
    if (m_driverController.B().Get()) {
        GoThroughTrench();
    } else {
        DefaultDrive();
    }
}

void DriveManager::DefaultDrive() {
    xMovement = -m_driverController.GetLeftY();
    yMovement = -m_driverController.GetLeftX();
    rotMovement = m_driverController.GetRightX();
}

void DriveManager::GoThroughTrench() {
    frc::Pose2d botPose = m_GetCurrentBotPose();
    double strafe = m_YAlignmentPID.Calculate(botPose.Y().value(), kSetpointDistance.value());
    strafe = std::clamp(strafe, -1.0, 1.0);

    BearLog::Log("Strafe PID", strafe);

    units::degree_t currentDegrees = botPose.Rotation().Degrees();
    double rotation = m_rotationalPID.Calculate(currentDegrees.value(), (0_deg).value());
    rotation = std::clamp(rotation, -1.0, 1.0);

    BearLog::Log("Rotation PID", rotation);


    xMovement = -m_driverController.GetLeftY();
    yMovement = -strafe;
    rotMovement = rotation;
}