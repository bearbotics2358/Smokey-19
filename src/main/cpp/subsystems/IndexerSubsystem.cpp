#include "subsystems/IndexerSubsystem.h"

#include "bearlog/bearlog.h"
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

IndexerSubsystem::IndexerSubsystem():
    m_indexerSpinMotor{kIndexerSpinMotorID}
{
    // @todo Update these with real PID values for the motor
    ctre::phoenix6::configs::Slot0Configs slot0Config;
    slot0Config
        .WithKP(30)
        .WithKI(0)
        .WithKD(0)
        .WithKS(0)
        .WithKV(20);

    // Define the overall configuration with the new hardware limit switch config
    // In this config object, we can also apply other things such as current limits,
    // brake mode, which direction is positive rotation, etc.
    ctre::phoenix6::configs::TalonFXConfiguration rotation_config;
    rotation_config.WithSlot0(slot0Config);

    // Must apply the config to the motor during construction of this object and NOT within functions that
    // run in the normal Periodic loop
    m_indexerSpinMotor.GetConfigurator().Apply(rotation_config);
    if (frc::RobotBase::IsSimulation()) {
        SimulationInit();
    }
}

frc2::CommandPtr IndexerSubsystem::SpinMotor(units::angular_velocity::turns_per_second_t tps) {
    return frc2::cmd::RunOnce([this, tps] {
        ctre::phoenix6::controls::VelocityVoltage m_tpsRequest{0_tps};
        BearLog::Log("Indexer/SetpointMotorVelocity", tps);
        BearLog::Log("Indexer/MotorVelocity", GetMotorVelocity());
        m_indexerSpinMotor.SetControl(m_tpsRequest.WithVelocity(tps));
    });
}

void IndexerSubsystem::Periodic() {
}

units::angular_velocity::turns_per_second_t IndexerSubsystem::GetMotorVelocity() {
    units::angular_velocity::turns_per_second_t velocity = m_indexerSpinMotor.GetVelocity().GetValue();
    return velocity;
}

void IndexerSubsystem::SimulationInit() {
    auto& indexer_sim = m_indexerSpinMotor.GetSimState();
    indexer_sim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    indexer_sim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);
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

    // Update the simulated state for the Indexer motor
    indexer_sim.SetRotorVelocity(m_IndexerSimModel.GetAngularVelocity());
    indexer_sim.SetRotorAcceleration(m_IndexerSimModel.GetAngularAcceleration());
}