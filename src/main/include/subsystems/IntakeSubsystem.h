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
constexpr int kExtenderMotorID = 63;
constexpr units::inch_t kHopperInchesPerRotation = 1.0_in; 
constexpr units::inch_t kHopperRetractedPos = 0_in;  
constexpr units::inch_t kHopperExtendedPos = 10_in;

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem();
    frc2::CommandPtr SpinMotor(units::volt_t volts);

    frc2::CommandPtr SetHopper(bool extended);

    void Periodic() override;
    void SimulationPeriodic() override;
private:
        ctre::phoenix6::hardware::TalonFX m_intakeSpinMotor;

        ctre::phoenix6::hardware::TalonFX m_hopperExtenderMotor; 

        ctre::phoenix6::controls::PositionVoltage m_hopperPosRequest{0_tr};

        static constexpr double kGearRatio = 1;

        static constexpr units::turns_per_second_t kMaxVelocity = 1.5_tps;
        static constexpr units::turns_per_second_squared_t kMaxAcceleration = 0.75_tr_per_s_sq;


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
};