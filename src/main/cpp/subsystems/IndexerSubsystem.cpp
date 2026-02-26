#include "subsystems/IndexerSubsystem.h"

#include "bearlog/bearlog.h"
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

IndexerSubsystem::IndexerSubsystem():
    m_indexerSpinMotor{kIndexerSpinMotorID}
{
    ctre::phoenix6::configs::TalonFXConfiguration configs{};

    static constexpr units::ampere_t kPeakTorqueCurrent = 100_A;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = kPeakTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -kPeakTorqueCurrent;

    configs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    configs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

    configs.Slot0.kP = 0.4;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kV = 0.8;

    m_indexerSpinMotor.GetConfigurator().Apply(configs);
}

frc2::CommandPtr IndexerSubsystem::SpinMotorGoal(units::angular_velocity::turns_per_second_t tps) {
    return frc2::cmd::RunOnce([this, tps] {
        m_setpointSpeed = tps;
    });
}

void IndexerSubsystem::GoToSpeed() {
    BearLog::Log("Indexer/SetpointMotorVelocity", m_setpointSpeed);
    BearLog::Log("Indexer/MotorVelocity", GetMotorVelocity());
}

void IndexerSubsystem::Periodic() {
    GoToSpeed();
}

units::revolutions_per_minute_t IndexerSubsystem::GetMotorVelocity() {
    units::revolutions_per_minute_t velocity = m_indexerSpinMotor.GetVelocity().GetValue();
    return velocity;
}

// Runs in Simulation only!
void IndexerSubsystem::SimulationPeriodic() {
    auto& indexer_sim = m_indexerSpinMotor.GetSimState();
    indexer_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto motor_voltage = indexer_sim.GetMotorVoltage();
    m_IndexerSimModel.SetInputVoltage(motor_voltage);

    // Simulate the 20ms run in the simulation model
    m_IndexerSimModel.Update(20_ms);

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_IndexerSimModel.GetCurrentDraw()}));

    indexer_sim.SetRotorVelocity(m_IndexerSimModel.GetAngularVelocity());
    indexer_sim.SetRotorAcceleration(m_IndexerSimModel.GetAngularAcceleration());
}