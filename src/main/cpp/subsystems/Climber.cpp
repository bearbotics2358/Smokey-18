#include "subsystems/Climber.h"
#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber()
{
}

void Climber::Periodic() {
}

frc2::CommandPtr Climber::Climb() {
    return frc2::cmd::Run([this] {
        // @todo Implement the ability to climb
    });
}

frc2::CommandPtr Climber::CancelClimb() {
    return frc2::cmd::Run([this] {
        // @todo Implement the ability to cancel a climb safely
    });
}