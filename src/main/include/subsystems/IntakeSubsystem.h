#pragma once

#include <frc2/command/SubsystemBase.h>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem();
    void Periodic() override;
};