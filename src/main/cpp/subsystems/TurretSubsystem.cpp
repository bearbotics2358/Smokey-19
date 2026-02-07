#include "subsystems/TurretSubsystem.h"

#include "bearlog/bearlog.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/util/Color8Bit.h>
#include <frc/smartdashboard/SmartDashboard.h>

TurretSubsystem::TurretSubsystem(subsystems::CommandSwerveDrivetrain* drivetrain)
    : m_turretSpinMotor(kTurretMotorID), m_drivetrain(drivetrain)
{
    ctre::phoenix6::configs::MotorOutputConfigs motorConfigs;

    auto& talonFXConfigurator = m_turretSpinMotor.GetConfigurator();
    // ctre::phoenix6::configs::CurrentLimitsConfigs limitConfigs{};
    // limitConfigs.SupplyCurrentLimit = units::current::ampere_t(1);
    // limitConfigs.SupplyCurrentLimitEnable = true;
    // talonFXConfigurator.Apply(limitConfigs);

    motorConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        .WithInverted(false);

    m_turretSpinMotor.GetConfigurator().Apply(motorConfigs);

    m_turretSpinMotor.SetPosition(0_tr);

    m_turretPID.SetIntegratorRange(-0.4, 0.4);

    if (frc::RobotBase::IsSimulation()) {
        SimulationInit();
    }
}

void TurretSubsystem::Periodic() {
    frc::SmartDashboard::PutBoolean("Get Limit Switch", getLimitSwitch());

    BearLog::Log("Turret/Setpoint", m_setpointAngle);
    BearLog::Log("Turret/Angle", CurrentAngle());

    AngleToHub();
    GoToAngle();
}

units::degree_t TurretSubsystem::AngleToHub() {
    double strafe = m_drivetrain->GetState().Pose.Y().value() - 4.02;
    double forward = m_drivetrain->GetState().Pose.X().value() - 4.02;
    units::degree_t robotangle = m_drivetrain->GetState().Pose.Rotation().Degrees();
    units::degree_t angle = units::degree_t(units::radian_t(atan(strafe/forward))) - robotangle - 90_deg;
    SetAngleGoal(angle);

    BearLog::Log("Vision/DriveTrain/RobotPoseDrive/X", strafe);
    BearLog::Log("Vision/DriveTrain/RobotPoseDrive/Y", forward);
    BearLog::Log("Vision/DriveTrain/RobotPoseDrive/anglegoal", angle);

    return angle;
}

frc2::CommandPtr TurretSubsystem::SetGoalAngle(units::degree_t angle) {
    return frc2::cmd::Run([this, angle] {
        m_setpointAngle = angle;
    });
}

void TurretSubsystem::SetAngleGoal(units::degree_t angle) {
        m_setpointAngle = angle;
}

bool TurretSubsystem::motorLimit() {
    if ((CurrentAngle().value() >= 180) || (CurrentAngle().value() < -180)) {
        return true;
    } else {
        return false;
    };
}

units::degree_t TurretSubsystem::CurrentAngle() {
    units::degree_t angle = GetAngleFromTurns(m_turretSpinMotor.GetPosition().GetValue());
    return angle;
};

units::degree_t TurretSubsystem::GetAngleFromTurns(units::turn_t rotations) {
    units::degree_t angle = units::degree_t(rotations.value() * kGearRatio);
    return angle;
}

units::turn_t TurretSubsystem::GetTurnsFromAngle(units::degree_t angle) {
    units::turn_t rotations = units::turn_t(angle.value() / kGearRatio);
    return rotations;
}

bool TurretSubsystem::getLimitSwitch() {
    return !m_limitSwitch.Get();
}

void TurretSubsystem::GoToAngle() {
    P = frc::SmartDashboard::GetNumber("PIDTuner/P", 3);
    I = frc::SmartDashboard::GetNumber("PIDTuner/I", 0) / 3;
    D = frc::SmartDashboard::GetNumber("PIDTuner/D", 0) / 3;
    m_turretPID.SetPID(P, I, D);
    double value = m_turretPID.Calculate(CurrentAngle(), m_setpointAngle);
    frc::SmartDashboard::PutNumber("Turret PID", value);
    m_turretSpinMotor.SetVoltage(units::volt_t(value));
}

void TurretSubsystem::turretInit() {
    //not tested yet
    if (getLimitSwitch() == true) {
        m_turretSpinMotor.SetVoltage(units::volt_t(0));
        m_turretSpinMotor.SetPosition(0_tr);
        turretInitialized = true;
    } else {
        m_turretSpinMotor.SetVoltage(units::volt_t(0.2));
    };
};

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
}