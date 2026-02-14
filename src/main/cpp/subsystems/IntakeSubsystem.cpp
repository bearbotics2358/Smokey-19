#include "subsystems/IntakeSubsystem.h"

#include "bearlog/bearlog.h"
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/util/Color8Bit.h>
#include <frc/smartdashboard/SmartDashboard.h>

IntakeSubsystem::IntakeSubsystem():
    m_intakeSpinMotor(kIntakeMotorID),
    m_hopperExtenderMotor(kExtenderMotorID)
{
    ctre::phoenix6::configs::MotorOutputConfigs motorConfigs;

    auto& talonFXConfigurator = m_intakeSpinMotor.GetConfigurator();
    ctre::phoenix6::configs::CurrentLimitsConfigs limitConfigs{};
    limitConfigs.SupplyCurrentLimit = units::current::ampere_t(1);
    limitConfigs.SupplyCurrentLimitEnable = false;
    talonFXConfigurator.Apply(limitConfigs);

    motorConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        .WithInverted(false);

    m_intakeSpinMotor.SetPosition(0_tr);

    m_intakeSpinMotor.GetConfigurator().Apply(motorConfigs);

    ctre::phoenix6::configs::TalonFXConfiguration hopperConfig;
    hopperConfig.Slot0.kP = 1.0; 
    hopperConfig.Slot0.kI = 0.0;
    hopperConfig.Slot0.kD = 0.0;
    m_hopperExtenderMotor.GetConfigurator().Apply(hopperConfig);
    m_hopperExtenderMotor.SetPosition(0_tr);

    if (frc::RobotBase::IsSimulation()) {
        SimulationInit();
    }
}

void IntakeSubsystem::Periodic() {
}

frc2::CommandPtr IntakeSubsystem::SpinMotor(units::volt_t volts) {
    return frc2::cmd::RunOnce([this, volts] {
        BearLog::Log("Intake/Volts", volts);
        m_intakeSpinMotor.SetVoltage(volts);
    });
}
frc2::CommandPtr IntakeSubsystem::SetHopper(bool extended) {
    return frc2::cmd::RunOnce([this, extended] {
        units::inch_t targetInches = extended ? kHopperExtendedPos : kHopperRetractedPos;
        units::turn_t targetTurns = targetInches / kHopperInchesPerRotation * 1_tr;
        m_hopperExtenderMotor.SetControl(m_hopperPosRequest.WithPosition(targetTurns));
    });
}
// Runs in Simulation only!
void IntakeSubsystem::SimulationInit() {
    auto& intake_sim = m_intakeSpinMotor.GetSimState();
    intake_sim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    intake_sim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);
}

// Runs in Simulation only!
void IntakeSubsystem::SimulationPeriodic() {
    auto& intake_sim = m_intakeSpinMotor.GetSimState();
    intake_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto motor_voltage = intake_sim.GetMotorVoltage();
    m_IntakeSimModel.SetInputVoltage(motor_voltage);

    // Simulate the 20ms run in the simulation model
    m_IntakeSimModel.Update(20_ms);

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_IntakeSimModel.GetCurrentDraw()}));

    // Update the simulated state for the Intake motor
    intake_sim.SetRawRotorPosition(kGearRatio * m_IntakeSimModel.GetAngle());
    intake_sim.SetRotorVelocity(kGearRatio * m_IntakeSimModel.GetVelocity());
}