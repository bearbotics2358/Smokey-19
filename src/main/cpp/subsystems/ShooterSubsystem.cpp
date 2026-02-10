#include "subsystems/ShooterSubsystem.h"
#include "bearlog/bearlog.h"
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
//including the same as turret.cpp to see
#include <frc/RobotBase.h>
#include <frc/util/Color8Bit.h>

using namespace ctre::phoenix6;
ShooterSubsystem::ShooterSubsystem() {
    m_FlywheelFollowerMotor.SetControl(controls::Follower(m_FlywheelMotor.GetDeviceID(),signals::MotorAlignmentValue::Opposed));   
}

void ShooterSubsystem::Periodic() {
    BearLog::Log("Flywheel/Speed", units::revolutions_per_minute_t(m_FlywheelMotor.GetVelocity().GetValue()));
    GoToSpeed();
    
}

void ShooterSubsystem::SetGoalSpeed(units::revolutions_per_minute_t speed) {
        m_setSpeed = speed;
}

units::revolutions_per_minute_t ShooterSubsystem::CurrentSpeed() {
    units::degree_t speed = GetAngleFromTurns(m_turretSpinMotor.GetPosition().GetValue());
    return speed;
};

void ShooterSubsystem::GoToSpeed() { 
    P = frc::SmartDashboard::GetNumber("PIDTuner/P", 3);
    I = frc::SmartDashboard::GetNumber("PIDTuner/I", 0) / 3;
    D = frc::SmartDashboard::GetNumber("PIDTuner/D", 0) / 3;


    m_shooterPID.SetPID(P, I, D);
    double value = m_shooterPID.Calculate(CurrentSpeed().value(), m_setSpeed.value());
    frc::SmartDashboard::PutNumber("Flywheel PID", value);
    m_FlywheelMotor.SetVoltage(units::volt_t(value));
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