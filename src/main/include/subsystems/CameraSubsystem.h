#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "photon/PhotonCamera.h"
#include "photon/PhotonUtils.h"

#include "subsystems/CommandSwerveDrivetrain.h"

class CameraSubsystem : public frc2::SubsystemBase {
    public:
        CameraSubsystem(subsystems::CommandSwerveDrivetrain* m_drivetrain);

        void updateRP2Data();
        
        void Periodic() override;

        units::degree_t GetZRotation();

        units::degree_t GetAngleToTag();

        units::degree_t angle = 0_deg;

    private:
        photon::PhotonPipelineResult resultRP2;
        photon::PhotonTrackedTarget RP2BestTarget;

        frc::Transform3d RP2toTarget;

        #define CAMERA_NAME "ARDUCAM_RP2_2"
        photon::PhotonCamera rubikPi2Camera{CAMERA_NAME};

        subsystems::CommandSwerveDrivetrain* m_drivetrain;
};