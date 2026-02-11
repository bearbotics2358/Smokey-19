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
    m_extenderMotor(kExtenderMotorID)
{
    ctre::phoenix6::configs::MotorOutputConfigs motorConfigs;

    ctre::phoenix6::configs::MotorOutputConfigs eMotorConfigs;

    auto& talonFXConfigurator = m_intakeSpinMotor.GetConfigurator();
    ctre::phoenix6::configs::CurrentLimitsConfigs limitConfigs{};
    limitConfigs.SupplyCurrentLimit = units::current::ampere_t(1);
    limitConfigs.SupplyCurrentLimitEnable = false;
    talonFXConfigurator.Apply(limitConfigs);

    motorConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        .WithInverted(false);

    eMotorConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        .WithInverted(false);

    m_intakeSpinMotor.SetPosition(0_tr);

    m_extenderMotor.SetPosition(0_tr);

    m_intakeSpinMotor.GetConfigurator().Apply(motorConfigs);
    m_extenderMotor.GetConfigurator().Apply(motorConfigs);

    m_extenderPID.SetIntegratorRange(-0.4, 0.4);

    if (frc::RobotBase::IsSimulation()) {
        SimulationInit();
    }

    m_setpointAngle = m_stowAngle;
}

void IntakeSubsystem::Periodic() {
    BearLog::Log("IntakeExtender/Angle", CurrentAngle());

    GoToAngle();
}

frc2::CommandPtr IntakeSubsystem::SpinMotor(units::volt_t volts) {
    return frc2::cmd::RunOnce([this, volts] {
        BearLog::Log("Intake/Volts", volts);
        m_intakeSpinMotor.SetVoltage(volts);
    });
    CurrentAngle();
}

units::degree_t IntakeSubsystem::CurrentAngle() {
    units::degree_t angle = GetAngleFromTurns(m_extenderMotor.GetPosition().GetValue()) * kGearRatio;
    return angle;
}

frc2::CommandPtr IntakeSubsystem::SetGoalAngle() {
    return frc2::cmd::RunOnce([this] {
        units::degree_t angle;
        if (isExtended == true) {
            isExtended = false;
            angle = 0_deg;
        } else {
            isExtended = true;
            angle = 90_deg;
        }
        BearLog::Log("Is Extended?", isExtended);
        m_setpointAngle = angle;
    });
}

units::degree_t IntakeSubsystem::GetAngleFromTurns(units::turn_t rotations) {
    units::degree_t angle = units::degree_t(rotations);
    return angle;
}

units::turn_t IntakeSubsystem::GetTurnsFromAngle(units::degree_t angle) {
    units::turn_t rotations = units::turn_t(angle);
    return rotations;
}

void IntakeSubsystem::GoToAngle() {
    double value = m_extenderPID.Calculate(CurrentAngle(), m_setpointAngle);
    BearLog::Log("IntakeExtender/PIDValue", value);
    BearLog::Log("IntakeExtender/Setpoint", m_setpointAngle);
    m_extenderMotor.SetVoltage(units::volt_t(value));
}

// Runs in Simulation only!
void IntakeSubsystem::SimulationInit() {
    const double kSimIntakeLineWidth = 6;
    m_IntakeMech = m_MechRoot->Append<frc::MechanismLigament2d>("Intake", kIntakeRadius.value(), 0_deg, kSimIntakeLineWidth, frc::Color8Bit{frc::Color::kPurple});
    frc::SmartDashboard::PutData("Intake Sim", &m_Mech);

    auto& intake_sim = m_intakeSpinMotor.GetSimState();
    intake_sim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    intake_sim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);

    m_EIntakeMech = m_EMechRoot->Append<frc::MechanismLigament2d>("Turret", kEIntakeRadius.value(), 0_deg, kSimIntakeLineWidth, frc::Color8Bit{frc::Color::kPurple});
    frc::SmartDashboard::PutData("Extender Sim", &m_EMech);

    auto& extender_sim = m_extenderMotor.GetSimState();
    extender_sim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    extender_sim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);
}

// Runs in Simulation only!
void IntakeSubsystem::SimulationPeriodic() {
    auto& intake_sim = m_intakeSpinMotor.GetSimState();
    intake_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto& extender_sim = m_extenderMotor.GetSimState();
    extender_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto motor_voltage = intake_sim.GetMotorVoltage();
    m_IntakeSimModel.SetInputVoltage(motor_voltage);

    auto eMotor_voltage = extender_sim.GetMotorVoltage();
    m_EIntakeSimModel.SetInputVoltage(eMotor_voltage);

    // Simulate the 20ms run in the simulation model
    m_IntakeSimModel.Update(20_ms);
    m_EIntakeSimModel.Update(20_ms);

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_IntakeSimModel.GetCurrentDraw()}));

    // Update the simulated state for the Intake motor
    intake_sim.SetRawRotorPosition(kGearRatio * m_IntakeSimModel.GetAngle());
    intake_sim.SetRotorVelocity(kGearRatio * m_IntakeSimModel.GetVelocity());

    extender_sim.SetRawRotorPosition(kEGearRatio * m_EIntakeSimModel.GetAngle());
    extender_sim.SetRotorVelocity(kEGearRatio * m_EIntakeSimModel.GetVelocity());

    // Update the simulated UI mechanism to the new angle based on the motor
    m_IntakeMech->SetAngle(0_deg);
    m_EIntakeMech->SetAngle(CurrentAngle());
}