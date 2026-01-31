// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot():
m_cameraSubsystem(&m_drivetrain),
m_turretSubsystem(),
m_container(&m_turretSubsystem)
{}

void Robot::RobotPeriodic() {
    m_timeAndJoystickReplay.Update();
    m_cameraSubsystem.Periodic();
    m_container.Periodic();
    m_turretSubsystem.SetGoalAngle(units::angle::degree_t(fabs(frc::SmartDashboard::GetNumber("PIDTuner/x", 0)) * 100));
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand.value());
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    //not tested yet
    m_turretSubsystem.turretInit();
    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand.value());
    }
}

void Robot::TeleopPeriodic() {
    m_turretSubsystem.Periodic();
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
