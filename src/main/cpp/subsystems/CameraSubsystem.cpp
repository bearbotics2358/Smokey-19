#include <subsystems/CameraSubsystem.h>

CameraSubsystem::CameraSubsystem(subsystems::CommandSwerveDrivetrain* m_drivetrain) {
    m_drivetrain = m_drivetrain;
}

void CameraSubsystem::updateRP2Data() {
    resultRP2 = rubikPi2Camera.GetLatestResult();
    if (resultRP2.HasTargets()) {
        frc::SmartDashboard::PutBoolean("RP2 Camera Has Targets", true);
        RP2BestTarget = resultRP2.GetBestTarget();
        RP2toTarget = RP2BestTarget.GetBestCameraToTarget();
        frc::SmartDashboard::PutNumber("RP2 X Pose", RP2toTarget.X().value());
        frc::SmartDashboard::PutNumber("RP2 Y Pose", RP2toTarget.Y().value());
        frc::SmartDashboard::PutNumber("RP2 ROT", units::degree_t(RP2toTarget.Rotation().Z()).value());
    } else {
        frc::SmartDashboard::PutBoolean("RP2 Camera Has Targets", false);
    }
}

units::degree_t CameraSubsystem::GetZRotation() {
    if (resultRP2.HasTargets()) {
        return units::degree_t(RP2toTarget.Rotation().Z());
    } else {
        return 0_deg;
    }
    frc::SmartDashboard::PutNumber("RP2 ROT", units::degree_t(RP2toTarget.Rotation().Z()).value());
}

units::degree_t CameraSubsystem::GetAngleToTag() {
    if (resultRP2.HasTargets()) {
        double strafe = RP2toTarget.Y().value();
        double forward = RP2toTarget.X().value();
        angle = units::degree_t(units::radian_t(atan(strafe/forward)));
        return angle;
    }
    frc::SmartDashboard::PutNumber("RP2 Angle Goal", angle.value());
}

void CameraSubsystem::Periodic() {
    updateRP2Data();
    frc::SmartDashboard::PutBoolean("Periodic Running", true);
}