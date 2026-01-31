#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/DigitalInput.h>
#include <frc/Encoder.h>

#include <units/length.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc2/command/button/Trigger.h>

#include <frc/controller/SimpleMotorFeedforward.h>

#include <subsystems/CameraSubsystem.h>

#include <subsystems/CommandSwerveDrivetrain.h>

constexpr int kTurretMotorID = 60;

class TurretSubsystem : public frc2::SubsystemBase {
    public:
        TurretSubsystem();

        frc2::CommandPtr SetGoalAngle(units::degree_t angle);
        units::degree_t CurrentAngle();
        void goToAngle();

        void Periodic() override;

        bool motorLimit();

        bool getLimitSwitch();
        
        void turretInit();

        bool turretInitialized = false;

        subsystems::CommandSwerveDrivetrain m_drivetrain{TunerConstants::CreateDrivetrain()};
    private:
        void GoToAngle();

        frc::DigitalInput m_limitSwitch{1};

        ctre::phoenix6::hardware::TalonFX m_turretSpinMotor;

        static constexpr units::turns_per_second_t kMaxVelocity = 1.5_tps;
        static constexpr units::turns_per_second_squared_t kMaxAcceleration = 0.75_tr_per_s_sq;
        double P = 3;
        double I = 0.3;
        double D = 0.0;


        frc::TrapezoidProfile<units::turns>::Constraints m_constraints {
            kMaxVelocity, kMaxAcceleration
        };

        frc::ProfiledPIDController<units::turns> m_turretPID {
            P, I, D, m_constraints
        };

        //frc::SimpleMotorFeedforward m_turretFeedForward{kS, kV, kA};

        units::degree_t m_setpointAngle = 90_deg;

        CameraSubsystem m_cameraSubsystem;

};