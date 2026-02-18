#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/Commands.h>

#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <units/length.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

constexpr int kIndexerSpinMotorID = 55;

class IndexerSubsystem : public frc2::SubsystemBase {
public:
    IndexerSubsystem();
    frc2::CommandPtr SpinMotorGoal(units::angular_velocity::turns_per_second_t tps);

    units::angular_velocity::turns_per_second_t GetMotorVelocity();

    void Periodic() override;
    void SimulationPeriodic() override;

private:
    void GoToSpeed();
    ctre::phoenix6::hardware::TalonFX m_indexerSpinMotor;

    units::turns_per_second_t m_setpointTPS;

    ctre::phoenix6::controls::VelocityVoltage m_tpsRequest{0_tps};

    static constexpr double kGearRatio = 7.5;

    static constexpr units::turns_per_second_t kMaxVelocity = 1.5_tps;
    static constexpr units::turns_per_second_squared_t kMaxAcceleration = 0.75_tr_per_s_sq;

    void SimulationInit();
    static constexpr units::kilogram_square_meter_t kIndexerMomentOfInertia = 0.5 * 1.5_lb * 4_in * 4_in;
    frc::DCMotor m_IndexerGearbox{frc::DCMotor::KrakenX60(1)};
    frc::LinearSystem<1, 1, 1> m_Plant{frc::LinearSystemId::FlywheelSystem(
    m_IndexerGearbox, kIndexerMomentOfInertia, kGearRatio)};
    frc::sim::FlywheelSim m_IndexerSimModel{
        m_Plant,
        m_IndexerGearbox
    };
};