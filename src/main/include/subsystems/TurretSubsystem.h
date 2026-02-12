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

        frc2::CommandPtr SetGoalAngle(units::degree_t angle);
        units::degree_t CurrentAngle();
        void goToAngle();

        void Periodic() override;
        void SimulationPeriodic() override;

        bool motorLimit();

        bool getLimitSwitch();

        void turretInit();

        units::degree_t AngleToHub();

        bool turretInitialized = false;
    private:
        void GoToAngle();
        units::degree_t GetAngleFromTurns(units::turn_t rotations);
        units::turn_t GetTurnsFromAngle(units::degree_t angle);

        frc::DigitalInput m_limitSwitch{1};

        ctre::phoenix6::hardware::TalonFX m_turretSpinMotor;
        std::function<frc::Pose2d()> m_GetCurrentBotPose;

        static constexpr double kGearRatio = 113.28;

        static constexpr units::turns_per_second_t kMaxVelocity = 1.5_tps;
        static constexpr units::turns_per_second_squared_t kMaxAcceleration = 0.75_tr_per_s_sq;
        double P = 3;
        double I = 0.3;
        double D = 0.0;


        frc::TrapezoidProfile<units::turns>::Constraints m_constraints {
            kMaxVelocity, kMaxAcceleration
        };

        frc::ProfiledPIDController<units::turns> m_turretPID {
            P, I, D, m_constraints
        };

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
          -180_deg,
          180_deg,
          false,
          10_deg,
        };

};