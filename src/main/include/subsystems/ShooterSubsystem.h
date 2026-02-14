#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <ctre/phoenix6/TalonFX.hpp>

class ShooterSubsystem : public frc2::SubsystemBase {
public:
    ShooterSubsystem(std::function<frc::Pose2d()> getBotPose);
    void Periodic() override;
    void SimulationPeriodic() override;
    units::meter_t DistanceToHub();

private:
    static constexpr int kFlywheelMotorId = 2;
    ctre::phoenix6::hardware::TalonFX m_FlywheelMotor{kFlywheelMotorId};
    
    std::function<frc::Pose2d()> m_GetCurrentBotPose;

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