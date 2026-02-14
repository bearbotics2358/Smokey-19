#include "subsystems/ShooterSubsystem.h"
#include "bearlog/bearlog.h"
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotBase.h>

using namespace ctre::phoenix6;
ShooterSubsystem::ShooterSubsystem()
    : m_shooterElevationSpinMotor{kShooterElevationMotorID}
{
    m_FlywheelFollowerMotor.SetControl(controls::Follower(m_FlywheelMotor.GetDeviceID(),signals::MotorAlignmentValue::Opposed));

    ctre::phoenix6::configs::MotorOutputConfigs motorConfigsLead;
    auto& talonFXConfiguratorLead = m_FlywheelMotor.GetConfigurator();
    motorConfigsLead.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast).WithInverted(false);
    talonFXConfiguratorLead.Apply(motorConfigsLead);

    ctre::phoenix6::configs::MotorOutputConfigs motorConfigsFollower;
    auto& talonFXConfiguratorFollower = m_FlywheelFollowerMotor.GetConfigurator();
    motorConfigsFollower.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    talonFXConfiguratorFollower.Apply(motorConfigsFollower);

    ctre::phoenix6::configs::Slot0Configs slot0Config;
    slot0Config
        .WithKP(0.6)
        .WithKI(0)
        .WithKD(0.2)
        .WithKS(0)
        .WithKV(0.2);

    ctre::phoenix6::configs::TalonFXConfiguration rotation_config;
    rotation_config
        .WithSlot0(slot0Config);

    m_shooterElevationSpinMotor.GetConfigurator().Apply(rotation_config);

    if (frc::RobotBase::IsSimulation()) {
        SimulationInit();
    }
}

void ShooterSubsystem::Periodic() {
    BearLog::Log("Flywheel/SetPointSpeed", m_setSpeed);
    BearLog::Log("Flywheel/Speed", CurrentSpeed());

    GoToAngle();

    
    BearLog::Log("ShooterElevation/Setpoint", m_setpointAngle);
    BearLog::Log("ShooterElevation/Angle", CurrentAngle());

    GoToSpeed();
}

frc2::CommandPtr ShooterSubsystem::SetGoalAngle(units::degree_t angle) {
    return frc2::cmd::RunOnce([this, angle] {
        m_setpointAngle = angle;
    });
}

units::degree_t ShooterSubsystem::CurrentAngle() {
    units::degree_t angle = GetAngleFromTurns(m_shooterElevationSpinMotor.GetPosition().GetValue());
    return angle;
};

units::degree_t ShooterSubsystem::GetAngleFromTurns(units::turn_t rotations) {
    units::degree_t angle = units::degree_t(rotations);
    return angle;
}

units::turn_t ShooterSubsystem::GetTurnsFromAngle(units::degree_t angle) {
    units::turn_t rotations = units::turn_t(angle);
    return rotations;
}

void ShooterSubsystem::GoToAngle() {
  units::turn_t position_in_motor_turns = GetTurnsFromAngle(m_setpointAngle);

  m_shooterElevationSpinMotor.SetControl(
    m_RotationVoltage.WithPosition(position_in_motor_turns)
      .WithSlot(0));
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

void ShooterSubsystem::SimulationInit() {
    const double kSimShooterElevationLineWidth = 6;
    m_ShooterElevationMech = m_MechRoot->Append<frc::MechanismLigament2d>("ShooterElevation", kShooterElevationRadius.value(), 0_deg, kSimShooterElevationLineWidth, frc::Color8Bit{frc::Color::kPurple});
    frc::SmartDashboard::PutData("ShooterElevation Sim", &m_Mech);

    auto& shooterElevation_sim = m_shooterElevationSpinMotor.GetSimState();
    shooterElevation_sim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    shooterElevation_sim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);
}

void ShooterSubsystem::SimulationPeriodic() {
    auto& flywheel_sim = m_FlywheelMotor.GetSimState();
    flywheel_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto motor_voltage = flywheel_sim.GetMotorVoltage();
    m_FlywheelSimModel.SetInputVoltage(motor_voltage);

    auto& flywheel_follower_sim = m_FlywheelFollowerMotor.GetSimState();
    flywheel_follower_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    // Simulate the 20ms run in the simulation model
    m_FlywheelSimModel.Update(20_ms);

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_FlywheelSimModel.GetCurrentDraw()}));

    // Update the simulated state for the flywheel motor
    flywheel_sim.SetRotorVelocity(m_FlywheelSimModel.GetAngularVelocity());
    flywheel_sim.SetRotorAcceleration(m_FlywheelSimModel.GetAngularAcceleration());


    //Shooter Elevation Sim
    auto& shooterElevation_sim = m_shooterElevationSpinMotor.GetSimState();
    shooterElevation_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto sMotor_voltage = shooterElevation_sim.GetMotorVoltage();
    m_ShooterElevationSimModel.SetInputVoltage(sMotor_voltage);

    // Simulate the 20ms run in the simulation model
    m_ShooterElevationSimModel.Update(20_ms);

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_ShooterElevationSimModel.GetCurrentDraw()}));

    // Update the simulated state for the shooterElevation motor
    shooterElevation_sim.SetRawRotorPosition(kGearRatio * m_ShooterElevationSimModel.GetAngle());
    shooterElevation_sim.SetRotorVelocity(kGearRatio * m_ShooterElevationSimModel.GetVelocity());

    // Update the simulated UI mechanism to the new angle based on the motor
    m_ShooterElevationMech->SetAngle(CurrentAngle());
}