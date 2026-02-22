// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Telemetry.h"
#include "Config.h"
#include "subsystems/CameraSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/IndexerSubsystem.h"
#include "vision/VisionConstants.h"
#include "vision/VisionSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

class RobotContainer {
private:
    RobotType m_RobotType{config::GetRobotType()};

    frc::SendableChooser<frc2::Command *> m_autoChooser;

    double speedlimit = 1.0;
    units::meters_per_second_t MaxSpeed = speedlimit * TunerConstants::GetSpeedAt12Volts(m_RobotType); // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = speedlimit * 0.75_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.1).WithRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{MaxSpeed};

    frc2::CommandXboxController driverJoystick{0};
    frc2::CommandXboxController operatorJoystick{1};

    TurretSubsystem m_turretSubsystem;
    ShooterSubsystem m_shooterSubsystem;
    IntakeSubsystem m_intakeSubsystem;
    IndexerSubsystem m_indexerSubsystem;

public:
    subsystems::CommandSwerveDrivetrain m_drivetrain{TunerConstants::CreateDrivetrain(m_RobotType)};
    VisionSubsystem m_VisionSubsystem{&m_drivetrain, VisionConstants::GetLocalizationCameras(&m_drivetrain)};

    RobotContainer();

    frc2::Command* GetAutonomousCommand();

    double xMovement = -driverJoystick.GetLeftY();
    double yMovement = -driverJoystick.GetLeftX();
    double rotMovement = driverJoystick.GetRightX();
private:
    void ConfigureBindings();

    static constexpr double kP = 1.75;
    static constexpr double kI = 0.0;
    static constexpr double kD = 0.0;
    
    frc::PIDController m_YAlignmentPID {kP, kI, kD};

    static constexpr double kRotationP = 0.05;
    static constexpr double kRotationI = 0.0;
    static constexpr double kRotationD = 0.001;
    frc::PIDController m_rotationalPID {kRotationP, kRotationI, kRotationD};

    static constexpr units::meters_per_second_t kMaxVelocity = 1.5_mps;
    static constexpr units::radians_per_second_t kMaxAngularVelocity = 1_rad_per_s;

    const units::meter_t kStrafeTolerance = units::meter_t(0.5_in);
    const units::degree_t kRotationTolerance = 2_deg;

    const units::meter_t kSetpointDistance = units::meter_t(20_in);
};
