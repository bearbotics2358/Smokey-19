#include "subsystems/ShooterSubsystem.h"
#include "bearlog/bearlog.h"
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotBase.h>

using namespace ctre::phoenix6;
ShooterSubsystem::ShooterSubsystem(std::function<frc::Pose2d()> getBotPose, TurretSubsystem* turretSubsystem)
    : m_shooterElevationSpinMotor{kShooterElevationMotorID},
    m_GetCurrentBotPose(getBotPose),
    m_turretSubsystem(turretSubsystem)
{
    m_trajectoryCalc.init();

    m_FlywheelFollowerMotor.SetControl(controls::Follower(m_FlywheelMotor.GetDeviceID(),signals::MotorAlignmentValue::Opposed));


    ctre::phoenix6::configs::Slot0Configs slot0Config;
    slot0Config
        .WithKP(3)
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

    units::meter_t distance = DistanceToHub();

    struct TrajectoryInfo shooterInfo;
    bzero(&shooterInfo, sizeof(shooterInfo));
    shooterInfo.elevation_angle = CurrentAngle();
    shooterInfo.distance = DistanceToHub();
    shooterInfo.wheel_rpm = m_setSpeed;

    //20_ft, 52_deg, 3200_rpm, 55_deg, 3275_rpm

    struct TrajectoryInfo shooterGoal = m_trajectoryCalc.compute_trajectory(shooterInfo);

    int retvalue = shooterGoal.return_value;

    BearLog::Log("shooterInfo/elevation_angle", shooterInfo.elevation_angle);
    BearLog::Log("shooterInfo/goalelevation_angle", shooterGoal.elevation_angle);
    BearLog::Log("shooterInfo/wheel_rpm", shooterInfo.wheel_rpm);
    BearLog::Log("shooterInfo/goalwheel_rpm", shooterGoal.wheel_rpm);
    BearLog::Log("shooterInfo/return_value", retvalue);

    m_setpointAngle = shooterGoal.elevation_angle;
    m_setSpeed = shooterGoal.wheel_rpm;

    DrawTrajectory(RPMToVelocity(m_setSpeed), CurrentAngle());
}

void ShooterSubsystem::DrawTrajectory(
    units::meters_per_second_t velocity,
    units::degree_t angle)
{
    units::meters_per_second_squared_t gravity = 9.81_mps_sq;
    std::vector<frc::Pose3d> poses;
    double timeBetweenPoses = 0.05;

    units::meter_t shooterHeight = 18.2817_in;
    frc::Pose2d botPose2d = m_GetCurrentBotPose();
    frc::Pose3d robotPose3d{
        botPose2d.X(),
        botPose2d.Y(),
        0_m,
        frc::Rotation3d{0_deg, 0_deg, botPose2d.Rotation().Degrees()}
    };

    units::radian_t degRad = units::radian_t(angle);

    BearLog::Log("velocity", velocity);

    // units::second_t timeOfFlight = units::second_t(((velocity.value() * sin(degRad.value())) +
    //     std::sqrt(std::pow(velocity.value() * sin(degRad.value()), 2) +
    //         (2.0 * gravity.value() * shooterHeight.value())
    //     )
    // ) / gravity.value());

    double time = 0;
    double hDist = 0;
    double vDist = shooterHeight.value();

    double airDensity = 1.225;
    double dragCoefficient = 0.47;
    double crossSectionArea = M_PI * pow((0.150114 / 2.0), 2);
    double mass = 0.227;

    double dragConstant = 0.5 * airDensity * dragCoefficient * crossSectionArea;

    double horizontalVelocity = velocity.value() * cos(degRad.value());
    double verticalVelocity = velocity.value() * sin(degRad.value());
    while ((vDist >= 0)) {
        // hDist = velocity.value() * cos(degRad.value()) * time;
        // vDist = shooterHeight.value() + (velocity.value() * sin(degRad.value()) * time) - (0.5 * gravity.value() * pow(time, 2));

        double totalSpeed = sqrt(horizontalVelocity * horizontalVelocity + verticalVelocity * verticalVelocity);

        double horizontalDrag = -(dragConstant * totalSpeed * horizontalVelocity) / mass;
        double verticalDrag = -gravity.value() - (dragConstant * totalSpeed * verticalVelocity) / mass;

        horizontalVelocity += horizontalDrag * timeBetweenPoses;
        verticalVelocity += verticalDrag * timeBetweenPoses;

        hDist += horizontalVelocity * timeBetweenPoses;
        vDist += verticalVelocity * timeBetweenPoses;

        frc::Translation3d localPose{
            units::meter_t(hDist), 
            0_m, 
            units::meter_t(vDist)
        };

        frc::Translation3d rotated = localPose.RotateBy(
        frc::Rotation3d{0_deg, 0_deg, botPose2d.Rotation().Degrees() + m_turretSubsystem->CurrentAngle()});

        frc::Pose3d worldPose{
            robotPose3d.X() + rotated.X(),
            robotPose3d.Y() + rotated.Y(),
            robotPose3d.Z() + rotated.Z(),
            frc::Rotation3d{}
        };

        poses.push_back(worldPose);
        
        time += timeBetweenPoses;
    }

    BearLog::Log("trajectory/trajectory", poses);
}


units::meters_per_second_t ShooterSubsystem::RPMToVelocity(units::revolutions_per_minute_t rpm) {
    units::turns_per_second_t revPerSec = rpm;
    units::meter_t flywheelCircumfrence = 2 * M_PI * kFlywheelRadius;
    return units::meters_per_second_t(revPerSec.value() * flywheelCircumfrence.value() / 2);
}

units::meter_t ShooterSubsystem::DistanceToHub() {
    double hubx = 4.535;
    double huby = 4.615;
    frc::Pose2d botPose = m_GetCurrentBotPose();
    units::meter_t distance = units::meter_t(sqrt(pow(botPose.X().value() - hubx, 2) + pow(botPose.Y().value() - huby, 2)));
    
    BearLog::Log("Distance from the Hub", distance);
    return distance;
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