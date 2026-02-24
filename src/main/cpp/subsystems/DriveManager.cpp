#include "subsystems/DriveManager.h"
#include "bearlog/bearlog.h"

DriveManager::DriveManager(std::function<frc::Pose2d()> getBotPose) :
    m_GetCurrentBotPose(getBotPose)
{}

void DriveManager::Periodic() {
    if (m_driverController.A().Get()) {
        GoThroughTrench();
    } else if (m_driverController.B().Get()) {
        AngleBump();
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
    double robotY = m_YAlignmentPID.Calculate(botPose.Y().value(), kSetpointDistance.value());
    robotY = std::clamp(robotY, -1.0, 1.0);

    BearLog::Log("Strafe PID", robotY);

    units::degree_t currentDegrees = botPose.Rotation().Degrees();
    double rotation = m_rotationalPID.Calculate(currentDegrees.value(), (0_deg).value());
    rotation = std::clamp(rotation, -1.0, 1.0);

    BearLog::Log("Rotation PID", rotation);


    xMovement = -m_driverController.GetLeftY();
    yMovement = -robotY;
    rotMovement = rotation;
}

void DriveManager::AngleBump() {
    frc::Pose2d botPose = m_GetCurrentBotPose();


    units::degree_t currentDegrees = botPose.Rotation().Degrees();

    double rotation = m_rotationalPID.Calculate(currentDegrees.value(), (45_deg).value());
    rotation = std::clamp(rotation, -1.0, 1.0);

    BearLog::Log("Rotation PID", rotation);


    xMovement = -m_driverController.GetLeftY();
    yMovement = -m_driverController.GetLeftX();
    rotMovement = rotation;
}