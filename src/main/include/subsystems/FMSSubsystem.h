#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DriverStation.h>


class FMSSubsystem : public frc2::SubsystemBase {
public:
    FMSSubsystem();
    
    units::second_t GetMatchTime();
    std::string AutoWinner();

    void Periodic() override;
    
    enum class allianceShift{
        kNoShift,
        kBlueShift,
        kRedShift,
        kBothShift
    };

    allianceShift CurrentShift();
    
    allianceShift m_autoWinner;
    allianceShift m_autoLoser;
};