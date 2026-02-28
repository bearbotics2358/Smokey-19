#include <subsystems/FMSSubsystem.h>
#include <bearlog/bearlog.h>
#include <frc/smartdashboard/SmartDashboard.h>

FMSSubsystem::FMSSubsystem() {

}

void FMSSubsystem::Periodic() {
    BearLog::Log("FMS/MatchTime", GetMatchTime());
    CurrentShift();
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
    return units::second_t(std::round(frc::DriverStation::GetMatchTime().value()));
}

enum FMSSubsystem::allianceShift FMSSubsystem::CurrentShift() {
    if (GetMatchTime() > 130_s){
        BearLog::Log("Next Shift", GetMatchTime() - 130_s);
        BearLog::Log("AllianceColor", std::string("#ffffffff"));
        return allianceShift::kBothShift;
    } else if ((GetMatchTime() <= 130_s) && (GetMatchTime() > 30_s)) {
        if (AutoWinner() == "Red") {
            m_autoWinner = allianceShift::kRedShift;
            m_autoLoser = allianceShift::kBlueShift;
        } else if (AutoWinner() == "Blue") {
            m_autoWinner = allianceShift::kBlueShift;
            m_autoLoser = allianceShift::kRedShift;
        } else {
            m_autoWinner = allianceShift::kNoShift;
            m_autoLoser = allianceShift::kNoShift;
        }

        int currentShift;
        if ((GetMatchTime() <= 130_s) && (GetMatchTime() > 105_s)) {
            BearLog::Log("Next Shift", GetMatchTime() - 105_s);
            currentShift = 4;
        } else if ((GetMatchTime() <= 105_s) && (GetMatchTime() > 80_s)) {
            BearLog::Log("Next Shift", GetMatchTime() - 80_s);
            currentShift = 3;
        } else if ((GetMatchTime() <= 80_s) && (GetMatchTime() > 55_s)) {
            BearLog::Log("Next Shift", GetMatchTime() - 55_s);
            currentShift = 2;
        } else if ((GetMatchTime() <= 55_s) && (GetMatchTime() > 30_s)) {
            BearLog::Log("Next Shift", GetMatchTime() - 30_s);
            currentShift = 1;
        } else {
            BearLog::Log("Next Shift", GetMatchTime());
            currentShift = 0;
        }

        if (!(currentShift == 0)) {
            if (currentShift % 2 == 0) {
                if (m_autoLoser == allianceShift::kRedShift) {
                    BearLog::Log("AllianceColor", std::string("#FF0000"));
                } else {
                    BearLog::Log("AllianceColor", std::string("#0000FF"));
                }
                return m_autoLoser;
            } else {
                if (m_autoWinner == allianceShift::kRedShift) {
                    BearLog::Log("AllianceColor", std::string("#FF0000"));
                } else {
                    BearLog::Log("AllianceColor", std::string("#0000FF"));
                }
                return m_autoWinner;
            }
        }
    } else if ((GetMatchTime() <= 30_s) && (GetMatchTime() >= 0_s)) {
        BearLog::Log("AllianceColor", std::string("#ffffffff"));
        return allianceShift::kBothShift;
    } else {
        BearLog::Log("AllianceColor", std::string("#000000ff"));
        return allianceShift::kNoShift;
    }
}