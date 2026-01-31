#include "subsystems/TurretSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

TurretSubsystem::TurretSubsystem():
m_turretSpinMotor(kTurretMotorID),
m_cameraSubsystem(&m_drivetrain)
{
    ctre::phoenix6::configs::MotorOutputConfigs motorConfigs;
    
    auto& talonFXConfigurator = m_turretSpinMotor.GetConfigurator();
    configs::CurrentLimitsConfigs limitConfigs{};
    limitConfigs.SupplyCurrentLimit = units::current::ampere_t(1);
    limitConfigs.SupplyCurrentLimitEnable = true;
    talonFXConfigurator.Apply(limitConfigs);

    motorConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        .WithInverted(false);

    m_turretSpinMotor.GetConfigurator().Apply(motorConfigs);

    m_turretSpinMotor.SetPosition(0_tr);

    m_turretSpinMotor.GetPosition().WaitForUpdate(20_ms);

    m_turretPID.SetIntegratorRange(-0.4, 0.4);
}

void TurretSubsystem::Periodic() {
    frc::SmartDashboard::PutBoolean("Get Limit Switch", getLimitSwitch());

    GoToAngle();
}

frc2::CommandPtr TurretSubsystem::SetGoalAngle(units::degree_t angle) {
    m_setpointAngle = angle;
    frc::SmartDashboard::PutNumber("Degree setpoint", m_setpointAngle.value());
    return frc2::cmd::RunOnce([this] {
        m_setpointAngle;
    });
}

bool TurretSubsystem::motorLimit() {
    if ((CurrentAngle().value() >= 180) || (CurrentAngle().value() < -180)) {
        return true;
    } else {
        return false;
    };
}

units::degree_t TurretSubsystem::CurrentAngle() {
    units::degree_t rotation = units::degree_t(m_turretSpinMotor.GetRotorPosition().GetValueAsDouble() * 113.28);
    if (rotation > 360_deg) {
        rotation -= 360_deg;
    }
    if (rotation < 0_deg) {
        rotation += 360_deg;
    }
    frc::SmartDashboard::PutNumber("CurrentAngle", rotation.value());
    frc::SmartDashboard::PutNumber("CurrentAngleRaw", units::degree_t(m_turretSpinMotor.GetRotorPosition().GetValueAsDouble() * 360).value());
    return rotation;
};

bool TurretSubsystem::getLimitSwitch() {
    return !m_limitSwitch.Get();
}

void TurretSubsystem::GoToAngle() {
    P = frc::SmartDashboard::GetNumber("PIDTuner/P", 3);
    I = frc::SmartDashboard::GetNumber("PIDTuner/I", 0) / 3;
    D = frc::SmartDashboard::GetNumber("PIDTuner/D", 0) / 3;
    m_turretPID.SetPID(P, I, D);
    if (turretInitialized = true) {
        double value = m_turretPID.Calculate(CurrentAngle(), m_setpointAngle);
        frc::SmartDashboard::PutNumber("Turret PID", value);
        m_turretSpinMotor.SetVoltage(units::volt_t(value));
    };
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