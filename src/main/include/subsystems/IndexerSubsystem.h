#pragma once

#include <frc2/command/SubsystemBase.h>

class IndexerSubsystem : public frc2::SubsystemBase {
public:
    IndexerSubsystem();
    void Periodic() override;
};