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

        frc2::CommandPtr PrepareElevator(units::inch_t desiredPosition);

        frc2::CommandPtr Score(bool removeAlgae);
    private:
        ElevatorSubsystem& m_elevator;
        CoralSubsystem& m_coral;
        AlgaeSubsystem& m_algae;

        units::inch_t m_elevatorSetpointHeight;

        // Values for the tuple go from coral to algae angles.
        std::map<units::inch_t, std::tuple<double, double>> m_elevatorMap = {
            {kElevatorStowPosition, std::make_tuple(kCoralStow, kAlgaeStow)},
            {kElevatorCollectPosition, std::make_tuple(kCoralStow, kAlgaeStow)},
            {kElevatorL1Position, std::make_tuple(kCoralL1, kAlgaeStow)},
            {kElevatorL2Position, std::make_tuple(kCoralL2, kAlgaeStow)},
            {kElevatorL3Position, std::make_tuple(kCoralL3, kAlgaeStow)},
            {kElevatorL4Position, std::make_tuple(kCoralL4 kAlgaeStow)},
        };
};

}