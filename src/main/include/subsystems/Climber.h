#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

class Climber : public frc2::SubsystemBase {
public:
    Climber();
    void Periodic() override;

    frc2::CommandPtr Climb();
    frc2::CommandPtr CancelClimb();
};