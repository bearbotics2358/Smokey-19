#include <frc/geometry/Pose3d.h>

static bool isRobotInBlueAllianceZone (frc::Pose2d botPose) {
    units::meter_t blueAllianceEnd = 4.625594_m;

    if (botPose.X() < blueAllianceEnd) {
        return true;
    } else {
        return false;
    }
}

static bool isRobotInRedAllianceZone (frc::Pose2d botPose) {
    units::meter_t redAllianceEnd = 11.915394_m;

    if (botPose.X() > redAllianceEnd) {
        return true;
    } else {
        return false;
    }
}

static bool isRobotInNeutralZone (frc::Pose2d botPose) {
    units::meter_t blueAllianceEnd = 4.625594_m;
    units::meter_t redAllianceEnd = 11.915394_m;

    if ((botPose.X() < redAllianceEnd) && (botPose.X() > blueAllianceEnd)) {
        return true;
    } else {
        return false;
    }
}

static std::string CurrentZone (frc::Pose2d botPose) {
    units::meter_t blueAllianceEnd = 4.625594_m;
    units::meter_t redAllianceEnd = 11.915394_m;

    if ((botPose.X() > redAllianceEnd)) {
        return "Red Alliance Zone";
    } else if (botPose.X() < blueAllianceEnd) {
        return "Blue Alliance Zone";
    } else {
        return "Neutral Zone";
    }
}