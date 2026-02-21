#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/controller/BangBangController.h>
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/button/Trigger.h>
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <units/length.h>

#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>

using namespace ctre::phoenix6;

class ShooterSubsystem : public frc2::SubsystemBase {
public:
    ShooterSubsystem();

    units::revolutions_per_minute_t CurrentSpeed();
    void SetGoalSpeed(units::revolutions_per_minute_t speed);
    void Periodic() override;
    void SimulationPeriodic() override;
    frc2::CommandPtr EnableShooter();
    frc2::CommandPtr StopShooter();

    units::degree_t CurrentAngle();

    frc2::CommandPtr SetGoalAngle(units::degree_t angle);

private:
    void GoToSpeed();

    static constexpr int kFlywheelMotorId = 37;
    ctre::phoenix6::hardware::TalonFX m_FlywheelMotor{kFlywheelMotorId};

    static constexpr int kFlywheelFollowerMotorId = 36;
    ctre::phoenix6::hardware::TalonFX m_FlywheelFollowerMotor{kFlywheelFollowerMotorId};

    frc::BangBangController m_shooterBangBang {};

    units::revolutions_per_minute_t m_setSpeed = 0_rpm;


    ////////////////////////////
    // Simulation related values
    //
    // Reduction between motors and encoder, as output over input. If the flywheel
    // spins slower than the motors, this number should be greater than one.
    static constexpr double kFlywheelGearing = 1.0;

    // @todo Figure out a real moment of inertia for the flywheel for a better simulation
    static constexpr units::kilogram_square_meter_t kFlywheelMomentOfInertia = 0.5 * 1.5_lb * 4_in * 4_in;

    frc::DCMotor m_ShooterGearbox{frc::DCMotor::KrakenX60(1)};
    frc::LinearSystem<1, 1, 1> m_Plant{frc::LinearSystemId::FlywheelSystem(
      m_ShooterGearbox, kFlywheelMomentOfInertia, kFlywheelGearing)};
    frc::sim::FlywheelSim m_FlywheelSimModel{m_Plant, m_ShooterGearbox};


    //Elevation Motor Inits
    void GoToAngle();
    units::degree_t GetAngleFromTurns(units::turn_t rotations);
    units::turn_t GetTurnsFromAngle(units::degree_t angle);

    ctre::phoenix6::hardware::TalonFX m_shooterElevationSpinMotor;
    static constexpr int kShooterElevationMotorID = 43;

    static constexpr double kGearRatio = 1;

    units::degree_t m_setpointAngle = 65_deg;

    ctre::phoenix6::controls::PositionVoltage m_RotationVoltage{2.2_tr};

        void SimulationInit();
        frc::Mechanism2d m_Mech{1, 1};
        frc::MechanismRoot2d* m_MechRoot{m_Mech.GetRoot("shooterElevationRoot", 0.5, 0.5)};
        frc::MechanismLigament2d* m_ShooterElevationMech;
        const units::meter_t kShooterElevationRadius = 12_in;
        frc::DCMotor m_ShooterElevationGearbox{frc::DCMotor::KrakenX60(1)};
        frc::sim::SingleJointedArmSim m_ShooterElevationSimModel{
          m_ShooterElevationGearbox,
          kGearRatio,
          frc::sim::SingleJointedArmSim::EstimateMOI(kShooterElevationRadius, 0.2_kg),
          kShooterElevationRadius,
          -360_deg,
          360_deg,
          false,
          10_deg,
        };
};