// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "LaunchHelper.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

RobotContainer::RobotContainer()
    : m_turretSubsystem{[this] { return m_drivetrain.GetState().Pose; }}
{
    // The LaunchHelper needs to be initialized when the robot code is booting up before any other calls to
    // LaunchHelper are made
    LaunchHelper::GetInstance().Init(
        // Shooter/hood angle supplier
        [this] { return m_shooterSubsystem.GetCurrentHoodAngle(); },

        // Shooter speed supplier
        [this] { return m_shooterSubsystem.GetCurrentSpeed(); },

        // Turret angle supplier
        [this] { return m_turretSubsystem.CurrentAngle(); },

        // Robot speed supplier
        [this] { return m_drivetrain.GetState().Speeds; },

        // Robot pose supplier
        [this] { return m_drivetrain.GetState().Pose; }
    );

    m_drivetrain.ConfigureAutoBuilder();

    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    frc::SmartDashboard::PutData("Auto Mode", &m_autoChooser);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        })
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        m_drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    joystick.A().WhileTrue(m_drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    joystick.B().WhileTrue(m_drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));

    joystick.LeftTrigger().OnFalse(m_intakeSubsystem.SpinMotor(0_V));
    joystick.LeftTrigger().OnTrue(m_intakeSubsystem.SpinMotor(5_V));

    joystick.RightTrigger().OnFalse(m_indexerSubsystem.SpinMotorGoal(0_tps));
    joystick.RightTrigger().OnTrue(m_indexerSubsystem.SpinMotorGoal(2_tps));

    joystick.A().WhileTrue(m_turretSubsystem.SetGoalAngle(135_deg));
    joystick.B().WhileTrue(m_turretSubsystem.SetGoalAngle(180_deg));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Back() && joystick.Y()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(m_drivetrain.RunOnce([this] { m_drivetrain.SeedFieldCentric(); }));

    m_drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
    return m_autoChooser.GetSelected();
}
