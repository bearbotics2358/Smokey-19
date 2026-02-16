#include "subsystems/ShooterSubsystem.h"
#include "bearlog/bearlog.h"
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotBase.h>

using namespace ctre::phoenix6;
ShooterSubsystem::ShooterSubsystem() {
    m_FlywheelFollowerMotor.SetControl(controls::Follower(m_FlywheelMotor.GetDeviceID(),signals::MotorAlignmentValue::Opposed));

    ctre::phoenix6::configs::MotorOutputConfigs motorConfigsLead;
    auto& talonFXConfiguratorLead = m_FlywheelMotor.GetConfigurator();
    motorConfigsLead.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast).WithInverted(false);
    talonFXConfiguratorLead.Apply(motorConfigsLead);

    ctre::phoenix6::configs::MotorOutputConfigs motorConfigsFollower;
    auto& talonFXConfiguratorFollower = m_FlywheelFollowerMotor.GetConfigurator();
    motorConfigsFollower.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    talonFXConfiguratorFollower.Apply(motorConfigsFollower);
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
    //m_shooterBangBang.SetTolerance(m_tolerance); 
    double value = m_shooterBangBang.Calculate(CurrentSpeed().value(), m_setSpeed.value());
    frc::SmartDashboard::PutNumber("Flywheel BangBang", value);
    m_FlywheelMotor.SetVoltage(units::volt_t(value*kMaxVolts));
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