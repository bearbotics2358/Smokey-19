#include "subsystems/DriveManager.h"
#include "bearlog/bearlog.h"
#include "subsystems/RobotZoneHelper.h"

DriveManager::DriveManager(std::function<frc::Pose2d()> getBotPose) :
    m_GetCurrentBotPose(getBotPose)
{
    m_SetpointDistance = kLeftSetpointDistance;
}

bool DriveManager::AssistManagerA() {
    BearLog::Log("assist", std::string("assist running"));
    if (GoThroughTrench() == false) {
        if (AngleBump() == false) {
            BearLog::Log("Driver Assist", std::string("No Avaiable Tool"));
            return false;
        } else {
            BearLog::Log("Driver Assist", std::string("Turn For Bump"));
            return true;
        }
    } else {
        BearLog::Log("Driver Assist", std::string("Go Through Trench"));
        return true;
    };
}

bool DriveManager::GoThroughTrench() {
    RobotZoneHelper::TrenchZone inTrenchZone = RobotZoneHelper::isRobotInTrenchZone(m_GetCurrentBotPose());

    if (!(inTrenchZone == RobotZoneHelper::TrenchZone::NoTrenchZone)) {
        if (inTrenchZone == RobotZoneHelper::TrenchZone::InRightTrenchZone) {
            m_SetpointDistance = kRightSetpointDistance;
        } else {
            m_SetpointDistance = kLeftSetpointDistance;
        }


        frc::Pose2d botPose = m_GetCurrentBotPose();
        double robotY = m_YAlignmentPID.Calculate(botPose.Y().value(), m_SetpointDistance.value());
        robotY = std::clamp(robotY, -1.0, 1.0);

        BearLog::Log("Strafe PID", robotY);

        units::degree_t currentDegrees = botPose.Rotation().Degrees();
        double rotation = m_rotationalPID.Calculate(currentDegrees.value(), (0_deg).value());
        rotation = std::clamp(rotation, -1.0, 1.0);

        BearLog::Log("Rotation PID", rotation);


        xMovement = -m_driverController.GetLeftY();
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
            yMovement = -robotY;
        } else {
            yMovement = robotY;
        }
        rotMovement = rotation;

        return true;
    } else {
        return false;
    }
}

bool DriveManager::AngleBump() {
    RobotZoneHelper::BumpZone inBumpZone = RobotZoneHelper::isRobotInBumpZone(m_GetCurrentBotPose());

    if (!(inBumpZone == RobotZoneHelper::BumpZone::NoBumpZone)) {
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
        return false;
    }
}

bool DriveManager::TurnToHub() {
    if (RobotZoneHelper::isRobotInMyAllianceZone(m_GetCurrentBotPose())) {
        BearLog::Log("PointAtHub", true);


        frc::Pose2d botPose = m_GetCurrentBotPose();
        frc::DriverStation::Alliance currentAlliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);

        frc::Pose2d HubPose;
        units::degree_t offset = 0_deg;

        if (currentAlliance == frc::DriverStation::Alliance::kRed) {
            HubPose = redHubPose;
            offset = 180_deg;
        } else {
            HubPose = blueHubPose;
        }

        units::meter_t strafe = botPose.Y() - HubPose.Y();
        units::meter_t forward = botPose.X() - HubPose.X();
        units::degree_t angleToHub = units::degree_t(units::radian_t(atan(strafe.value()/forward.value()))) + offset;

        units::degree_t currentDegrees = botPose.Rotation().Degrees();

        double rotation = m_rotationalPID.Calculate(currentDegrees.value(), (angleToHub).value());
        rotation = std::clamp(rotation, -1.0, 1.0);

        BearLog::Log("Rotation PID", rotation);


        xMovement = -m_driverController.GetLeftY();
        yMovement = -m_driverController.GetLeftX();
        rotMovement = rotation;

        return true;
    } else {
        BearLog::Log("PointAtHub", false);

        return false;
    }
}