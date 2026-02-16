#include "subsystems/TurretSubsystem.h"

#include "bearlog/bearlog.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/util/Color8Bit.h>
#include <frc/smartdashboard/SmartDashboard.h>

TurretSubsystem::TurretSubsystem(std::function<frc::Pose2d()> getBotPose)
    : m_turretSpinMotor(kTurretMotorID),
      m_GetCurrentBotPose(getBotPose)
{
    ctre::phoenix6::configs::Slot0Configs slot0Config;
    slot0Config
        .WithKP(3)
        .WithKI(0)
        .WithKD(0.2)
        .WithKS(0)
        .WithKV(0.2);

    // Define the overall configuration with the new hardware limit switch config
    // In this config object, we can also apply other things such as current limits,
    // brake mode, which direction is positive rotation, etc.
    ctre::phoenix6::configs::TalonFXConfiguration rotation_config;
    rotation_config
        .WithSlot0(slot0Config);

    // Must apply the config to the motor during construction of this object and NOT within functions that
    // run in the normal Periodic loop
    m_turretSpinMotor.GetConfigurator().Apply(rotation_config);

    if (frc::RobotBase::IsSimulation()) {
        SimulationInit();
    }
}

void TurretSubsystem::Periodic() {
    frc::SmartDashboard::PutBoolean("Get Limit Switch", getLimitSwitch());

    BearLog::Log("Turret/Setpoint", m_setpointAngle);
    BearLog::Log("Turret/Angle", CurrentAngle());

    GoToAngle();
}

units::degree_t TurretSubsystem::AngleToHub() {
    frc::Pose2d robotPose = m_GetCurrentBotPose();

    double strafe = robotPose.Y().value() - 4.135;
    double forward = robotPose.X().value() - 4.615;
    units::degree_t robotAngle = robotPose.Rotation().Degrees();
    units::degree_t angleToHub = units::degree_t(units::radian_t(atan(strafe/forward))) - robotAngle;

    return angleToHub;
}

frc2::CommandPtr TurretSubsystem::PointAtHub() {
    return frc2::cmd::RunOnce([this] {
        if (pointAtHubToggle == true) {
            pointAtHubToggle = false;
        } else {
            pointAtHubToggle = true;
        }
        BearLog::Log("Turret/PointToggle", pointAtHubToggle);
    });
}

void TurretSubsystem::SetGoalAngle() {
    if (pointAtHubToggle == false) {
        m_setpointAngle = m_stowAngle;
    } else {
        m_setpointAngle = AngleToHub();
    }
}

units::degree_t TurretSubsystem::CurrentAngle() {
    units::degree_t angle = GetAngleFromTurns(m_turretSpinMotor.GetPosition().GetValue());
    return angle;
};

units::degree_t TurretSubsystem::GetAngleFromTurns(units::turn_t rotations) {
    units::degree_t angle = units::degree_t(rotations);
    return angle;
}

units::turn_t TurretSubsystem::GetTurnsFromAngle(units::degree_t angle) {
    units::turn_t rotations = units::turn_t(angle);
    return rotations;
}

bool TurretSubsystem::getLimitSwitch() {
    return !m_limitSwitch.Get();
}

void TurretSubsystem::GoToAngle() {
    SetGoalAngle();
  units::turn_t position_in_motor_turns = GetTurnsFromAngle(m_setpointAngle);

  m_turretSpinMotor.SetControl(
    m_RotationVoltage.WithPosition(position_in_motor_turns)
      .WithSlot(0));
}

// Runs in Simulation only!
void TurretSubsystem::SimulationInit() {
    const double kSimTurretLineWidth = 6;
    m_TurretMech = m_MechRoot->Append<frc::MechanismLigament2d>("Turret", kTurretRadius.value(), 0_deg, kSimTurretLineWidth, frc::Color8Bit{frc::Color::kPurple});
    frc::SmartDashboard::PutData("Turret Sim", &m_Mech);

    auto& turret_sim = m_turretSpinMotor.GetSimState();
    turret_sim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    turret_sim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);
}

// Runs in Simulation only!
void TurretSubsystem::SimulationPeriodic() {
    auto& turret_sim = m_turretSpinMotor.GetSimState();
    turret_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto motor_voltage = turret_sim.GetMotorVoltage();
    m_TurretSimModel.SetInputVoltage(motor_voltage);

    // Simulate the 20ms run in the simulation model
    m_TurretSimModel.Update(20_ms);

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_TurretSimModel.GetCurrentDraw()}));

    // Update the simulated state for the turret motor
    turret_sim.SetRawRotorPosition(kGearRatio * m_TurretSimModel.GetAngle());
    turret_sim.SetRotorVelocity(kGearRatio * m_TurretSimModel.GetVelocity());

    // Update the simulated UI mechanism to the new angle based on the motor
    m_TurretMech->SetAngle(CurrentAngle());

    // AdvantageScope does not yet support a mechanism that is in the XY plane. To visualize that in 3d mode,
    // we construct a Pose3d object and use a Cone object in AdvantageScope.
    frc::Pose2d bot_pose = m_GetCurrentBotPose();

    // Incorporate the rotation of the robot to simulate the turret being attached
    frc::Pose3d turret_sim_3d_pose = {bot_pose.X(), bot_pose.Y(), 12_in, frc::Rotation3d{0_rad, 0_rad, bot_pose.Rotation().Degrees() + CurrentAngle()}};
    BearLog::Log("Turret/Pose", turret_sim_3d_pose);
}