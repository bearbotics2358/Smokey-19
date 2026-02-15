#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/geometry/Pose3d.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/DigitalInput.h>
#include <frc/Encoder.h>

#include <units/length.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc2/command/button/Trigger.h>

constexpr int kTurretMotorID = 60;

class TurretSubsystem : public frc2::SubsystemBase {
    public:
        TurretSubsystem(std::function<frc::Pose2d()> getBotPose);

        void SetGoalAngle();
        units::degree_t CurrentAngle();

        units::degree_t AngleToHub();

        frc2::CommandPtr PointAtHub();

        void Periodic() override;
        void SimulationPeriodic() override;

        bool getLimitSwitch();

        void turretInit();

        bool turretInitialized = false;
        bool pointAtHubToggle = true;

        units::degree_t m_stowAngle = 0_deg;
    private:
        void GoToAngle();
        units::degree_t GetAngleFromTurns(units::turn_t rotations);
        units::turn_t GetTurnsFromAngle(units::degree_t angle);

        ctre::phoenix6::controls::PositionVoltage m_RotationVoltage{0_tr};

        frc::DigitalInput m_limitSwitch{1};

        ctre::phoenix6::hardware::TalonFX m_turretSpinMotor;
        std::function<frc::Pose2d()> m_GetCurrentBotPose;

        static constexpr double kGearRatio = 1;

        units::degree_t m_setpointAngle = 90_deg;

        // Simulation specific items
        void SimulationInit();
        frc::Mechanism2d m_Mech{1, 1};
        frc::MechanismRoot2d* m_MechRoot{m_Mech.GetRoot("turretRoot", 0.5, 0.5)};
        frc::MechanismLigament2d* m_TurretMech;
        const units::meter_t kTurretRadius = 12_in;
        frc::DCMotor m_TurretGearbox{frc::DCMotor::KrakenX60(1)};
        frc::sim::SingleJointedArmSim m_TurretSimModel{
          m_TurretGearbox,
          kGearRatio,
          frc::sim::SingleJointedArmSim::EstimateMOI(kTurretRadius, 0.1_kg),
          kTurretRadius,
          -360_deg,
          360_deg,
          false,
          10_deg,
        };

};