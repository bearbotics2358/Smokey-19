#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rectangle2d.h>

static bool isRobotInBlueAllianceZone (frc::Pose2d botPose) {
    frc::Rectangle2d blueAllianceZone = frc::Rectangle2d(
        frc::Pose2d(91.05_in, 158.84_in, 
        frc::Rotation2d()), 
        182.11_in, 317.69_in
    );

    return blueAllianceZone.Contains(botPose.Translation());
}

static bool isRobotInRedAllianceZone (frc::Pose2d botPose) {
    frc::Rectangle2d redAllianceZone = frc::Rectangle2d(
        frc::Pose2d(560.17_in, 158.84_in, 
        frc::Rotation2d()), 
        182.11_in, 317.69_in
    );

    return redAllianceZone.Contains(botPose.Translation());
}

static bool isRobotInNeutralZone (frc::Pose2d botPose) {
    frc::Rectangle2d neutralZone = frc::Rectangle2d(
        frc::Pose2d(325.61_in, 158.84_in, 
        frc::Rotation2d()), 
        287_in, 317.69_in
    );

    return neutralZone.Contains(botPose.Translation());
}