// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

RobotContainer::RobotContainer()
    : m_cameraSubsystem(&m_drivetrain),
      m_turretSubsystem{[this] { return m_drivetrain.GetState().Pose; }}
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-driverJoystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-driverJoystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(-driverJoystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        })
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        m_drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    driverJoystick.A().WhileTrue(m_drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    driverJoystick.B().WhileTrue(m_drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-driverJoystick.GetLeftY(), -driverJoystick.GetLeftX()});
    }));

    operatorJoystick.A().OnTrue(m_turretSubsystem.PointAtHub());

    driverJoystick.LeftTrigger().OnFalse(m_intakeSubsystem.SpinMotor(0_V));
    driverJoystick.LeftTrigger().OnTrue(m_intakeSubsystem.SpinMotor(5_V));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (driverJoystick.Back() && driverJoystick.Y()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (driverJoystick.Back() && driverJoystick.X()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (driverJoystick.Start() && driverJoystick.Y()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (driverJoystick.Start() && driverJoystick.X()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    driverJoystick.LeftBumper().OnTrue(m_drivetrain.RunOnce([this] { m_drivetrain.SeedFieldCentric(); }));

    m_drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    // Simple drive forward auton
    return frc2::cmd::Sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        m_drivetrain.RunOnce([this] { m_drivetrain.SeedFieldCentric(frc::Rotation2d{0_deg}); }),
        // Then slowly drive forward (away from us) for 5 seconds.
        m_drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(0.5_mps)
                .WithVelocityY(0_mps)
                .WithRotationalRate(0_tps);
        })
        .WithTimeout(5_s),
        // Finally idle for the rest of auton
        m_drivetrain.ApplyRequest([] { return swerve::requests::Idle{}; })
    );
}
