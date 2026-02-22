#include <subsystems/FMSSubsystem.h>
#include <bearlog/bearlog.h>
#include <frc/smartdashboard/SmartDashboard.h>

FMSSubsystem::FMSSubsystem() {

}

void FMSSubsystem::Periodic() {
    BearLog::Log("FMS/MatchTime", GetMatchTime());

    std::string gameData = frc::DriverStation::GetGameSpecificMessage();

    BearLog::Log("FMS/AutoWinner", AutoWinner());
}

std::string FMSSubsystem::AutoWinner() {
    std::string gameData = frc::DriverStation::GetGameSpecificMessage();
    if (!gameData.empty()) {
        if (gameData[0] == 'R') {
            return "Red";
        } else if (gameData[0] == 'B') {
            return "Blue";
        } else {
            return "Invalid FMS Data";
        }
    } else {
        return "No FMS Info";
    }
}

units::second_t FMSSubsystem::GetMatchTime() {
    return frc::DriverStation::GetMatchTime();
}