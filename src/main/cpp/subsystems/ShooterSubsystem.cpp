#include "subsystems/ShooterSubsystem.h"

#include "bearlog/bearlog.h"

#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

ShooterSubsystem::ShooterSubsystem(std::function<frc::Pose2d()> getBotPose)
    : m_GetCurrentBotPose(getBotPose)
{}

void ShooterSubsystem::Periodic() {
    BearLog::Log("Flywheel/Speed", units::revolutions_per_minute_t(m_FlywheelMotor.GetVelocity().GetValue()));
}

units::meter_t ShooterSubsystem::DistanceToHub() {
    double hubx = 4.335;
    double huby = 4.615;
    frc::Pose2d botPose = m_GetCurrentBotPose();
    units::meter_t distance = units::meter_t(sqrt(pow(botPose.X().value() - hubx, 2) + pow(botPose.Y().value() - hubx, 2)));
    
    return distance;
}

void ShooterSubsystem::SimulationPeriodic() {
    auto& flywheel_sim = m_FlywheelMotor.GetSimState();
    flywheel_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto motor_voltage = flywheel_sim.GetMotorVoltage();
    m_FlywheelSimModel.SetInputVoltage(motor_voltage);

    // Simulate the 20ms run in the simulation model
    m_FlywheelSimModel.Update(20_ms);

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_FlywheelSimModel.GetCurrentDraw()}));

    // Update the simulated state for the flywheel motor
    flywheel_sim.SetRotorVelocity(m_FlywheelSimModel.GetAngularVelocity());
    flywheel_sim.SetRotorAcceleration(m_FlywheelSimModel.GetAngularAcceleration());
}