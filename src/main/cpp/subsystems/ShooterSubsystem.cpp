#include "subsystems/ShooterSubsystem.h"
#include "bearlog/bearlog.h"
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotBase.h>

using namespace ctre::phoenix6;
ShooterSubsystem::ShooterSubsystem() {
    ctre::phoenix6::configs::MotorOutputConfigs motorConfigsLead;
    auto& talonFXConfiguratorLead = m_FlywheelMotor.GetConfigurator();
    motorConfigsLead.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    talonFXConfiguratorLead.Apply(motorConfigsLead);

    ctre::phoenix6::configs::MotorOutputConfigs motorConfigsFollower;
    auto& talonFXConfiguratorFollower = m_FlywheelFollowerMotor.GetConfigurator();
    motorConfigsFollower.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    talonFXConfiguratorFollower.Apply(motorConfigsFollower);

    ctre::phoenix6::configs::CurrentLimitsConfigs limitConfigs;
    // Reset the current limit back to the default of 70 amps
    limitConfigs.SupplyCurrentLimit = 70_A;
    talonFXConfiguratorLead.Apply(limitConfigs);
    talonFXConfiguratorFollower.Apply(limitConfigs);
}

void ShooterSubsystem::Periodic() {
    BearLog::Log("Flywheel/SetPointSpeed", m_setSpeed);
    BearLog::Log("Flywheel/Speed", CurrentSpeed());

    GoToSpeed();
}

void ShooterSubsystem::SetGoalSpeed(units::revolutions_per_minute_t speed) {
        m_setSpeed = speed;
}

units::revolutions_per_minute_t ShooterSubsystem::CurrentSpeed() {
    units::revolutions_per_minute_t speed = m_FlywheelMotor.GetVelocity().GetValue();
    return speed;
};

void ShooterSubsystem::GoToSpeed() {
    double value = m_shooterBangBang.Calculate(CurrentSpeed().value(), m_setSpeed.value());

    units::volt_t voltageToApply = units::volt_t(value * kMaxVolts);
    BearLog::Log("Flywheel/BangBang", voltageToApply);

    // The follower motor needs the opposite voltage applied as it spins the other direction from the lead motor
    m_FlywheelMotor.SetVoltage(voltageToApply);
    m_FlywheelFollowerMotor.SetVoltage(-voltageToApply);
}

frc2::CommandPtr ShooterSubsystem::EnableShooter(){
    return frc2::cmd::RunOnce([this] {
        SetGoalSpeed(3600_rpm);
    });
}

frc2::CommandPtr ShooterSubsystem::StopShooter(){
    return frc2::cmd::RunOnce([this] {
        SetGoalSpeed(0_rpm);
    });
}

void ShooterSubsystem::SimulationPeriodic() {
    auto& flywheel_sim = m_FlywheelMotor.GetSimState();
    flywheel_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto motor_voltage = flywheel_sim.GetMotorVoltage();
    m_FlywheelSimModel.SetInputVoltage(motor_voltage);

    auto& flywheel_follower_sim = m_FlywheelFollowerMotor.GetSimState();
    flywheel_follower_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());


    // controls::DutyCycleOut m_FlywheelMotorRequest{0.0};
    // m_FlywheelMotor.SetControl(m_FlywheelMotorRequest.WithOutput(5));

    // Simulate the 20ms run in the simulation model
    m_FlywheelSimModel.Update(20_ms);

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_FlywheelSimModel.GetCurrentDraw()}));

    // Update the simulated state for the flywheel motor
    flywheel_sim.SetRotorVelocity(m_FlywheelSimModel.GetAngularVelocity());
    flywheel_sim.SetRotorAcceleration(m_FlywheelSimModel.GetAngularAcceleration());
}