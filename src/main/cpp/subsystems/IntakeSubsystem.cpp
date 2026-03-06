#include "subsystems/IntakeSubsystem.h"

#include "bearlog/bearlog.h"
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/util/Color8Bit.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;

IntakeSubsystem::IntakeSubsystem()
{
    ConfigureExtenderMotor();
    ConfigureIntakeMotor();

    if (frc::RobotBase::IsSimulation()) {
        SimulationInit();
    }

    m_setpointAngle = kStowAngle;

    m_ExtenderHardStop = frc2::Trigger([this] {
        return (units::math::abs(m_extenderMotor.GetVelocity().GetValue()) < 1_tps &&
            units::math::abs(m_extenderMotor.GetTorqueCurrent().GetValue()) > 35_A);
    }).Debounce(0.1_s);
}

void IntakeSubsystem::ConfigureExtenderMotor() {
    configs::TalonFXConfiguration extender_config{};

    static constexpr units::ampere_t kPeakTorqueCurrent = 70_A;
    extender_config.TorqueCurrent.PeakForwardTorqueCurrent = kPeakTorqueCurrent;
    extender_config.TorqueCurrent.PeakReverseTorqueCurrent = -kPeakTorqueCurrent;
    extender_config.CurrentLimits.StatorCurrentLimit = 50_A;
    extender_config.CurrentLimits.StatorCurrentLimitEnable = true;

    extender_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    extender_config.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

    extender_config.Slot0.kP = 0.5;
    extender_config.Slot0.kI = 0.0;
    extender_config.Slot0.kD = 0.0;
    extender_config.Slot0.kV = 0.12;

    extender_config.Slot1.kP = 2.0;
    extender_config.Slot1.kI = 0.0;
    extender_config.Slot1.kD = 0.0;
    extender_config.Slot1.kV = 0.12;

    m_extenderMotor.GetConfigurator().Apply(extender_config);

    m_extenderMotor.SetPosition(0_tr);
}

void IntakeSubsystem::ConfigureExtenderCANCoder() {
    configs::CANcoderConfiguration config{};

    // @todo Find the center of rotation of the CANcoder and set this correctly!
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;

    // @todo Find out if this is correct!
    config.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;

    // @todo Find out the real offset!
    config.MagnetSensor.MagnetOffset = 0.4_tr;

    m_ExtenderCANCoder.GetConfigurator().Apply(config);
}

void IntakeSubsystem::ConfigureIntakeMotor() {
    configs::TalonFXConfiguration configs{};

    static constexpr units::ampere_t kPeakTorqueCurrent = 70_A;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = kPeakTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -kPeakTorqueCurrent;

    configs.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    configs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

    configs.CurrentLimits.StatorCurrentLimit = 60_A;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    configs.Slot0.kP = 0.5;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kV = 0.12;

    m_intakeSpinMotor.GetConfigurator().Apply(configs);
}

void IntakeSubsystem::Periodic() {
    BearLog::Log("Intake/Extender/Angle", CurrentAngle());
    BearLog::Log("Intake/Extender/Turns", m_extenderMotor.GetPosition().GetValue());
    BearLog::Log("Intake/Extender/Setpoint", GetAngleFromTurns(m_ExtenderPositionVoltage.Position));
    BearLog::Log("Intake/Velocity", units::revolutions_per_minute_t(m_intakeSpinMotor.GetVelocity().GetValue()));
    BearLog::Log("Intake/Extender/Current", m_extenderMotor.GetTorqueCurrent().GetValue());
    BearLog::Log("Intake/Extender/Voltage", m_extenderMotor.GetMotorVoltage().GetValue());

    BearLog::Log("Intake/Extender/CANcoder position", m_ExtenderCANCoder.GetPosition().GetValue());
    BearLog::Log("Intake/Extender/Setpoint", m_ExtenderPositionVoltage.Position);
}

frc2::CommandPtr IntakeSubsystem::RunIntake() {
    return RunOnce([this] {
        m_intakeSpinMotor.SetControl(m_IntakeVelocity.WithVelocity(2000_rpm));
    });
}

frc2::CommandPtr IntakeSubsystem::RunIntakeToHelpIndexer() {
    // Use this command when attempting to push more fuel into the indexer
    return RunOnce([this] {
        m_intakeSpinMotor.SetControl(m_IntakeVelocity.WithVelocity(500_rpm));
    });
}

frc2::CommandPtr IntakeSubsystem::RunIntakeInReverse() {
    return RunOnce([this] {
        m_intakeSpinMotor.SetControl(m_IntakeVelocity.WithVelocity(-2000_rpm));
    });
}

frc2::CommandPtr IntakeSubsystem::AgitateToHelpIndexer() {
    return Run([this] {
        m_intakeSpinMotor.SetControl(m_IntakeVelocity.WithVelocity(500_rpm));
        m_extenderMotor.SetControl(m_ExtenderPositionVoltage.WithPosition(0_tr).WithSlot(0));
    }).RaceWith(
        frc2::cmd::Wait(1_s)
    ).AndThen(
        frc2::cmd::Parallel(
            StopHopper(),
            StopIntake()
        )
    );
}

frc2::CommandPtr IntakeSubsystem::StopIntake() {
    return RunOnce([this] {
        m_intakeSpinMotor.SetControl(m_Stop);
    });
}

units::degree_t IntakeSubsystem::CurrentAngle() {
    units::degree_t angle = GetAngleFromTurns(m_extenderMotor.GetPosition().GetValue()) * kEGearRatio;
    return angle;
}

frc2::CommandPtr IntakeSubsystem::ExtendHopper() {
    return Run([this] {
        m_extenderMotor.SetControl(m_ExtenderPositionVoltage.WithPosition(11_tr).WithSlot(0));
    }).Until(
        // This could probably be done using WithLimitForwardMotion, but this works for now
        [this] { return m_ExtenderHardStop.Get(); }
    ).AndThen(
        StopHopper()
    );
}

frc2::CommandPtr IntakeSubsystem::StowHopper() {
    return Run([this] {
        // Using Slot 1 here for retracting to give the the hopper more power to retract
        m_extenderMotor.SetControl(m_ExtenderPositionVoltage.WithPosition(0_tr).WithSlot(1));
    }).Until(
        // This could probably be done using WithLimitForwardMotion, but this works for now
        [this] { return m_ExtenderHardStop.Get(); }
    ).AndThen(
        StopHopper()
    );
}

frc2::CommandPtr IntakeSubsystem::StopHopper() {
    return RunOnce([this] {
        m_extenderMotor.SetControl(m_Stop);
    });
}

units::degree_t IntakeSubsystem::GetAngleFromTurns(units::turn_t rotations) {
    units::degree_t angle = units::degree_t(rotations.value() * kEGearRatio);
    return angle;
}

units::turn_t IntakeSubsystem::GetTurnsFromAngle(units::degree_t angle) {
    units::turn_t rotations = units::turn_t(angle.value() / kEGearRatio);
    return rotations;
}

// Runs in Simulation only!
void IntakeSubsystem::SimulationInit() {
    const double kSimIntakeLineWidth = 6;
    auto& intake_sim = m_intakeSpinMotor.GetSimState();
    intake_sim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    intake_sim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);

    m_EIntakeMech = m_EMechRoot->Append<frc::MechanismLigament2d>("Turret", kEIntakeRadius.value(), 0_deg, kSimIntakeLineWidth, frc::Color8Bit{frc::Color::kPurple});
    frc::SmartDashboard::PutData("Extender Sim", &m_EMech);

    auto& extender_sim = m_extenderMotor.GetSimState();
    extender_sim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    extender_sim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);
}

// Runs in Simulation only!
void IntakeSubsystem::SimulationPeriodic() {
    auto& intake_sim = m_intakeSpinMotor.GetSimState();
    intake_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto& extender_sim = m_extenderMotor.GetSimState();
    extender_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto motor_voltage = intake_sim.GetMotorVoltage();
    m_IntakeSimModel.SetInputVoltage(motor_voltage);

    auto eMotor_voltage = extender_sim.GetMotorVoltage();
    m_EIntakeSimModel.SetInputVoltage(eMotor_voltage);

    // Simulate the 20ms run in the simulation model
    m_IntakeSimModel.Update(20_ms);
    m_EIntakeSimModel.Update(20_ms);

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_IntakeSimModel.GetCurrentDraw()}));

    // Update the simulated state for the Intake motor
    intake_sim.SetRawRotorPosition(kGearRatio * m_IntakeSimModel.GetAngle());
    intake_sim.SetRotorVelocity(kGearRatio * m_IntakeSimModel.GetVelocity());

    extender_sim.SetRawRotorPosition(kEGearRatio * m_EIntakeSimModel.GetAngle());
    extender_sim.SetRotorVelocity(kEGearRatio * m_EIntakeSimModel.GetVelocity());

    // Update the simulated UI mechanism to the new angle based on the motor
    m_EIntakeMech->SetAngle(CurrentAngle());
}