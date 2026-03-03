// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "LaunchHelper.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/RunCommand.h>

#include "bearlog/bearlog.h"
#include "subsystems/RobotZoneHelper.h"

RobotContainer::RobotContainer()
    : m_turretSubsystem{[this] { return m_drivetrain.GetState().Pose; }},
      m_driveManager{[this] { return m_drivetrain.GetState().Pose; }}
{
    // The LaunchHelper needs to be initialized when the robot code is booting up before any other calls to
    // LaunchHelper are made
    LaunchHelper::GetInstance().Init(
        // Shooter/hood angle supplier
        [this] { return m_shooterSubsystem.GetCurrentHoodAngle(); },

        // Shooter speed supplier
        [this] { return m_shooterSubsystem.GetCurrentShooterSpeed(); },

        // Turret angle supplier
        [this] { return m_turretSubsystem.CurrentAngle(); },

        // Robot speed supplier
        [this] { return m_drivetrain.GetState().Speeds; },

        // Robot pose supplier
        [this] { return m_drivetrain.GetState().Pose; }
    );

    AddPathPlannerCommands();

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
                return drive.WithVelocityX(-driverJoystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .WithVelocityY(-driverJoystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .WithRotationalRate(driverJoystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
            })
        );

    driverJoystick.A().WhileTrue(
        frc2::cmd::Run([this] {
            if (m_driveManager.AssistManagerA() == true) {
                m_drivetrain.SetControl(
                    drive.WithVelocityX(m_driveManager.xMovement * MaxSpeed) // Drive forward with negative Y (forward)
                        .WithVelocityY(m_driveManager.yMovement * MaxSpeed) // Drive left with negative X (left)
                        .WithRotationalRate(m_driveManager.rotMovement * MaxAngularRate) // Drive counterclockwise with negative X (left)
                );
            }
        }));
    
    driverJoystick.LeftBumper().WhileTrue(
        frc2::cmd::Run([this] {
            if (m_driveManager.TurnToHub() == true) {
                m_drivetrain.SetControl(
                    drive.WithVelocityX(m_driveManager.xMovement * MaxSpeed) // Drive forward with negative Y (forward)
                        .WithVelocityY(m_driveManager.yMovement * MaxSpeed) // Drive left with negative X (left)
                        .WithRotationalRate(m_driveManager.rotMovement * MaxAngularRate) // Drive counterclockwise with negative X (left)
                );
            }
        }));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        m_drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    driverJoystick.X().WhileTrue(m_drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));

    driverJoystick.B().OnTrue(m_intakeSubsystem.ExtendHopper());
    driverJoystick.Y().OnTrue(m_intakeSubsystem.StowHopper());

    driverJoystick.LeftTrigger().OnFalse(m_intakeSubsystem.StopIntake());
    driverJoystick.LeftTrigger().OnTrue(m_intakeSubsystem.RunIntake());



    driverJoystick.RightBumper().OnTrue(
        frc2::cmd::Sequence(
            m_shooterSubsystem.EnableShooterWithFixedHoodAngle().WithTimeout(0.05_s),
            m_indexerSubsystem.RunIndexerForLaunching().WithTimeout(0.05_s)
        ).AndThen(
            frc2::cmd::Parallel(
                m_shooterSubsystem.EnableShooterWithFixedHoodAngle(),
                m_indexerSubsystem.RunIndexerForLaunching())
        )).OnFalse(
        frc2::cmd::Parallel(
            m_shooterSubsystem.StopShooter(),
            m_indexerSubsystem.Stop()
        )
    );

    driverJoystick.RightTrigger().WhileTrue(
        frc2::cmd::Sequence(
            m_shooterSubsystem.EnableShooterWithFixedHoodAngle().WithTimeout(0.05_s),
            m_indexerSubsystem.RunIndexerForLaunching().WithTimeout(0.05_s)
        ).AndThen(
            frc2::cmd::Parallel(
                m_shooterSubsystem.EnableShooterWithFixedHoodAngle(),
                m_indexerSubsystem.RunIndexerForLaunching())
        ).Until( [this] { 
            if (((m_FMSSubsystem.MyAllianceShift() == false) || (RobotZoneHelper::isRobotInMyAllianceZone(m_drivetrain.GetState().Pose) == false))) {
                return true;
            }}).AndThen(
                frc2::cmd::Parallel(
                    m_shooterSubsystem.StopShooter(),
                    m_indexerSubsystem.Stop()
                )
            )
    ).OnFalse(
        frc2::cmd::Parallel(
            m_shooterSubsystem.StopShooter(),
            m_indexerSubsystem.Stop()
        )
    );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (driverJoystick.Back() && driverJoystick.Y()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (driverJoystick.Back() && driverJoystick.X()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (driverJoystick.Start() && driverJoystick.Y()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (driverJoystick.Start() && driverJoystick.X()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    driverJoystick.POVDown().OnTrue(m_drivetrain.RunOnce([this] { m_drivetrain.SeedFieldCentric(); }));

    m_drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });


    operatorJoystick.A().OnTrue(m_turretSubsystem.PointAtHub());
<<<<<<< shooterBindingsFix
    operatorJoystick.POVLeft().WhileTrue(m_FMSSubsystem.ManualShift("Red"));
    operatorJoystick.POVRight().WhileTrue(m_FMSSubsystem.ManualShift("Blue"));
=======
    operatorJoystick.POVUp().WhileTrue(m_FMSSubsystem.ManualShift("Red"));
    operatorJoystick.POVDown().WhileTrue(m_FMSSubsystem.ManualShift("Blue"));

    //Don't use until tested
    //operatorJoystick.B().OnTrue(m_shooterSubsystem.CalibrateHoodMotor());
>>>>>>> main
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
    return m_autoChooser.GetSelected();
}

void RobotContainer::AddPathPlannerCommands() {
    using namespace pathplanner;
    NamedCommands::registerCommand(
        "Extend Hopper",
        std::move(m_intakeSubsystem.ExtendHopper())
    );
    NamedCommands::registerCommand(
        "Run Intake",
        std::move(m_intakeSubsystem.RunIntake())
    );
    NamedCommands::registerCommand(
        "Stop Intake",
        std::move(m_intakeSubsystem.StopIntake())
    );
    NamedCommands::registerCommand(
        "Point Turret at Hub",
        std::move(m_turretSubsystem.PointAtHub())
    );
    NamedCommands::registerCommand(
        "Launch Fuel at Hub",
        std::move(
            frc2::cmd::Sequence(
                m_shooterSubsystem.EnableShooterWithFixedHoodAngle(),
                frc2::cmd::Wait(40_ms),
                m_indexerSubsystem.RunIndexerForLaunching()
            )
        )
    );
}
