#include "subsystems/ShooterSubsystem.h"
#include "bearlog/bearlog.h"
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotBase.h>

using namespace ctre::phoenix6;
ShooterSubsystem::ShooterSubsystem() {
    configs::TalonFXConfiguration configs{};

    static constexpr units::ampere_t kPeakTorqueCurrent = 40_A;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = kPeakTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -kPeakTorqueCurrent;

    configs.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;

    configs.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

    m_FlywheelMotor.GetConfigurator().Apply(configs);
    m_FlywheelFollowerMotor.GetConfigurator().Apply(configs);

    m_FlywheelFollowerMotor.SetControl(controls::Follower{m_FlywheelMotor.GetDeviceID(), signals::MotorAlignmentValue::Opposed});
}

void ShooterSubsystem::Periodic() {
    BearLog::Log("Flywheel/SetPointSpeed", m_setSpeed);
    BearLog::Log("Flywheel/Speed", CurrentSpeed());
    BearLog::Log("Flywheel Follower/Speed", units::revolutions_per_minute_t(m_FlywheelFollowerMotor.GetVelocity().GetValue()));
    BearLog::Log("Flywheel/Voltage", m_FlywheelMotor.GetMotorVoltage().GetValue());
    BearLog::Log("Flywheel Follower/Voltage", m_FlywheelFollowerMotor.GetMotorVoltage().GetValue());

    BearLog::Log("Flywheel/Supply Current", m_FlywheelMotor.GetSupplyCurrent().GetValue());
    BearLog::Log("Flywheel/Stator Current", m_FlywheelMotor.GetStatorCurrent().GetValue());
    BearLog::Log("Flywheel/Torque Current", m_FlywheelMotor.GetTorqueCurrent().GetValue());

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

    // The motors move at around 5000 RPMs at 10 V
    static constexpr double kMaxVolts = 10.0;
    units::volt_t voltageToApply = units::volt_t(value * kMaxVolts);
    BearLog::Log("Flywheel/BangBang", voltageToApply);

    // The other motor is configured as an inverted follower and will follow the control of this lead motor
    m_FlywheelMotor.SetVoltage(voltageToApply);
}

frc2::CommandPtr ShooterSubsystem::EnableShooter(){
    return frc2::cmd::RunOnce([this] {
        m_setSpeed = 5000_rpm;
    });
}

frc2::CommandPtr ShooterSubsystem::StopShooter(){
    return frc2::cmd::RunOnce([this] {
        m_setSpeed = 0_rpm;
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