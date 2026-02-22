// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/RunCommand.h>

#include "bearlog/bearlog.h"

RobotContainer::RobotContainer()
    : m_turretSubsystem{[this] { return m_drivetrain.GetState().Pose; }}
{
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
                return drive.WithVelocityX(xMovement * MaxSpeed) // Drive forward with negative Y (forward)
                    .WithVelocityY(yMovement * MaxSpeed) // Drive left with negative X (left)
                    .WithRotationalRate(rotMovement * MaxAngularRate); // Drive counterclockwise with negative X (left)
            })
        );


    driverJoystick.B().WhileTrue(frc2::cmd::Run([this] {
        frc::Pose2d botPose = m_drivetrain.GetState().Pose;
        double strafe = m_YAlignmentPID.Calculate(botPose.Y().value(), kSetpointDistance.value());
        strafe = std::clamp(strafe, -1.0, 1.0);

        BearLog::Log("Strafe PID", strafe);

        units::degree_t currentDegrees = botPose.Rotation().Degrees();
        double rotation = m_rotationalPID.Calculate(currentDegrees.value(), (0_deg).value());
        rotation = std::clamp(rotation, -1.0, 1.0);

        BearLog::Log("Rotation PID", rotation);


        xMovement = -driverJoystick.GetLeftY();
        yMovement = -strafe;
        rotMovement = rotation;
    })).OnFalse(
        frc2::cmd::Run([this] {
            xMovement = -driverJoystick.GetLeftY();
            yMovement = -driverJoystick.GetLeftX();
            rotMovement = driverJoystick.GetRightX();
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
    // driverJoystick.B().WhileTrue(m_drivetrain.ApplyRequest([this]() -> auto&& {
    //     return point.WithModuleDirection(frc::Rotation2d{-driverJoystick.GetLeftY(), -driverJoystick.GetLeftX()});
    // }));

    operatorJoystick.A().OnTrue(m_turretSubsystem.PointAtHub());

    driverJoystick.LeftTrigger().OnFalse(m_intakeSubsystem.SpinMotor(0_V));
    driverJoystick.LeftTrigger().OnTrue(m_intakeSubsystem.SpinMotor(5_V));
    driverJoystick.RightTrigger().OnFalse(m_indexerSubsystem.SpinMotorGoal(0_tps));
    driverJoystick.RightTrigger().OnTrue(m_indexerSubsystem.SpinMotorGoal(2_tps));

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

frc2::Command* RobotContainer::GetAutonomousCommand()
{
    return m_autoChooser.GetSelected();
}
