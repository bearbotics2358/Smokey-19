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
//simulation stuff i think
#include <frc/simulation/FlywheelSim.h>

using namespace ctre::phoenix6;

class ShooterSubsystem : public frc2::SubsystemBase {
public:
    ShooterSubsystem();

    units::revolutions_per_minute_t GetCurrentSpeed();
    units::degree_t GetCurrentHoodAngle();
    void SetGoals(units::revolutions_per_minute_t speed, units::degree_t hoodAngle);
    void Periodic() override;
    void SimulationPeriodic() override;
    frc2::CommandPtr EnableShooter();
    frc2::CommandPtr StopShooter();
    frc2::CommandPtr RunHubTracking();
    frc2::CommandPtr RunFixedPosition();

private:
    void GoToSpeed();

    static constexpr units::revolutions_per_minute_t kFixedPositionSpeed = 3600_rpm;
    static constexpr units::degree_t kFixedPositionHoodAngle = 55_deg;

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
};