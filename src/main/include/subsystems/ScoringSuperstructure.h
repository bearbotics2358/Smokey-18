#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include "subsystems/CoralSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include <subsystems/AlgaeSubsystem.h>

namespace subsystems {

class ScoringSuperstructure : public frc2::SubsystemBase {
    public:
        // @todo This will also need to include the CoralSubsystem and AlgaeSubsystem when they are ready
        ScoringSuperstructure(ElevatorSubsystem& elevator, CoralSubsystem& coralMech, AlgaeSubsystem& algae);

        // Command to return all components to their idle positions
        frc2::CommandPtr StowCommand();
        frc2::CommandPtr ScoreCoralL1Command();
        frc2::CommandPtr ScoreCoralL2Command();
        frc2::CommandPtr ScoreCoralL3Command();
        frc2::CommandPtr ScoreCoralL4Command();
        frc2::CommandPtr IntakePosCommand();
        frc2::CommandPtr MoveAlgaeCommand();

    private:
        ElevatorSubsystem& m_elevator;
        CoralSubsystem& m_coralMech;
        AlgaeSubsystem& m_algae;
};

}