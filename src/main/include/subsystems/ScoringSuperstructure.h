#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include "subsystems/CoralSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/AlgaeSubsystem.h"

#include <map>
#include <tuple>

namespace subsystems {

class ScoringSuperstructure : public frc2::SubsystemBase {
    public:
        // @todo This will also need to include the CoralSubsystem and AlgaeSubsystem when they are ready
        ScoringSuperstructure(ElevatorSubsystem& elevator, CoralSubsystem& coralMech, AlgaeSubsystem& algaeMech);

        frc2::CommandPtr PrepareElevator(units::inch_t desiredPosition, bool removeAlgae);

        frc2::CommandPtr ScoreIntoReef();
        frc2::CommandPtr ScoreIntoProcessor();
        frc2::CommandPtr PrepareAndScoreIntoReef(units::inch_t desiredPosition, bool removeAlgae);

        frc2::CommandPtr ToCollectPosition();
        frc2::CommandPtr ToStowPosition();
    private:
        ElevatorSubsystem& m_elevator;
        CoralSubsystem& m_coral;
        AlgaeSubsystem& m_algae;

        units::inch_t m_elevatorSetpointHeight;
        bool m_collectAlgae;

        // Values for the tuple go from coral to algae angles to the name of the command.
        std::map<units::inch_t, std::tuple<units::degree_t, units::degree_t>> m_elevatorMap = {
            {kElevatorStowPosition, std::make_tuple(kCoralStow, kAlgaeStowAngle)},
            {kElevatorCollectPosition, std::make_tuple(kCoralStow, kAlgaeStowAngle)},
            {kElevatorL1Position, std::make_tuple(kCoralL1, kAlgaeStowAngle)},
            {kElevatorL2Position, std::make_tuple(kCoralL2, kAlgaeStowAngle)},
            {kElevatorL3Position, std::make_tuple(kCoralL3, kAlgaeStowAngle)},
            {kElevatorL4Position, std::make_tuple(kCoralL4, kAlgaeStowAngle)},
        };
};

}