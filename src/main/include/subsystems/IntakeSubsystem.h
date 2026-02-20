#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/Commands.h>

#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>

#include <units/length.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc2/command/button/Trigger.h>

constexpr int kIntakeMotorID = 62;
constexpr int kExtenderMotorID = 61;

class IntakeSubsystem : public frc2::SubsystemBase {
public:
        IntakeSubsystem();
        frc2::CommandPtr SpinMotor(units::volt_t volts);
        units::degree_t CurrentAngle();

        void Periodic() override;
        void SimulationPeriodic() override;

        frc2::CommandPtr SetGoalAngle();
private:
        ctre::phoenix6::controls::PositionVoltage m_RotationVoltage{0_tr};
        void GoToAngle();

        units::degree_t GetAngleFromTurns(units::turn_t rotations);
        units::turn_t GetTurnsFromAngle(units::degree_t angle);


        ctre::phoenix6::hardware::TalonFX m_intakeSpinMotor;
        ctre::phoenix6::hardware::TalonFX m_extenderMotor;

        static constexpr double kGearRatio = 1;

        static constexpr double kEGearRatio = 1;

        units::degree_t m_setpointAngle;
        static constexpr units::degree_t kStowAngle = 0_deg;
        static constexpr units::degree_t kExtendedAngle = 90_deg;
        bool isExtended = false;


        void SimulationInit();
        const units::meter_t kIntakeRadius = 1_in;
        frc::DCMotor m_IntakeGearbox{frc::DCMotor::KrakenX60(1)};
        frc::sim::SingleJointedArmSim m_IntakeSimModel{
          m_IntakeGearbox,
          kGearRatio,
          frc::sim::SingleJointedArmSim::EstimateMOI(kIntakeRadius, 0.1_kg),
          kIntakeRadius,
          -180_deg,
          180_deg,
          false,
          10_deg,
        };

        frc::Mechanism2d m_EMech{1, 1};
        frc::MechanismRoot2d* m_EMechRoot{m_EMech.GetRoot("extenderRoot", 0.5, 0.5)};
        frc::MechanismLigament2d* m_EIntakeMech;
        const units::meter_t kEIntakeRadius = 12_in;
        frc::DCMotor m_EIntakeGearbox{frc::DCMotor::KrakenX60(1)};
        frc::sim::SingleJointedArmSim m_EIntakeSimModel{
          m_EIntakeGearbox,
          kEGearRatio,
          frc::sim::SingleJointedArmSim::EstimateMOI(kEIntakeRadius, 0.1_kg),
          kEIntakeRadius,
          -180_deg,
          180_deg,
          false,
          10_deg,
        };
};