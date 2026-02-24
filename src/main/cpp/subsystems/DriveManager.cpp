#include "subsystems/DriveManager.h"
#include "bearlog/bearlog.h"
#include "subsystems/RobotZoneHelper.h"

DriveManager::DriveManager(std::function<frc::Pose2d()> getBotPose) :
    m_GetCurrentBotPose(getBotPose)
{
    kSetpointDistance = kLeftSetpointDistance;
}

void DriveManager::Periodic() {
    if (m_driverController.A().Get()) {
        if (GoThroughTrench() == false) {
            if (AngleBump() == false) {
                BearLog::Log("Driver Assist", std::string("No Avaiable Tool"));
            } else {
                BearLog::Log("Driver Assist", std::string("Turn For Bump"));
            }
        } else {
            BearLog::Log("Driver Assist", std::string("Go Through Trench"));
        }
    } else {
        DefaultDrive();
        BearLog::Log("Driver Assist", std::string("Default Drive"));
    }
}

void DriveManager::DefaultDrive() {
    xMovement = -m_driverController.GetLeftY();
    yMovement = -m_driverController.GetLeftX();
    rotMovement = m_driverController.GetRightX();
}

bool DriveManager::GoThroughTrench() {
    std::string inTrenchZone = RobotZoneHelper::isRobotInTrenchZone(m_GetCurrentBotPose());

    BearLog::Log("CurrentZone/InTrenchZone", inTrenchZone);

    if (!(inTrenchZone == "noTrenchZone")) {
        if (inTrenchZone == "inRightTrenchZone") {
            kSetpointDistance = kRightSetpointDistance;
        } else {
            kSetpointDistance = kLeftSetpointDistance;
        }


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

        return true;
    } else {
        DefaultDrive();

        return false;
    }
}

bool DriveManager::AngleBump() {
    std::string inBumpZone = RobotZoneHelper::isRobotInBumpZone(m_GetCurrentBotPose());

    BearLog::Log("CurrentZone/inBumpZone", inBumpZone);

    if (!(inBumpZone == "noBumpZone")) {
        frc::Pose2d botPose = m_GetCurrentBotPose();

        units::degree_t currentDegrees = botPose.Rotation().Degrees();

        double rotation = m_rotationalPID.Calculate(currentDegrees.value(), (45_deg).value());
        rotation = std::clamp(rotation, -1.0, 1.0);

        BearLog::Log("Rotation PID", rotation);


        xMovement = -m_driverController.GetLeftY();
        yMovement = -m_driverController.GetLeftX();
        rotMovement = rotation;

        return true;
    } else {
        DefaultDrive();

        return false;
    }
}