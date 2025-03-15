#include "subsystems/ScoringSuperstructure.h"

using namespace subsystems;

ScoringSuperstructure::ScoringSuperstructure(
    ElevatorSubsystem& elevator, 
    CoralSubsystem& coralMech, 
    AlgaeSubsystem& algaeMech
):
m_elevator(elevator),
m_coral(coralMech),
m_algae(algaeMech)
{}

frc2::CommandPtr ScoringSuperstructure::PrepareElevator(units::inch_t desiredPosition) {
    return frc2::cmd::RunOnce([this, desiredPosition] {
        m_elevatorSetpointHeight = desiredPosition;
    });
}

frc2::CommandPtr ScoringSuperstructure::ScoreIntoReef(bool removeAlgae) {
    auto [coralAngle, algaeAngle, commandName] = m_elevatorMap[m_elevatorSetpointHeight];

    if (removeAlgae) {
        return frc2::cmd::Parallel(
            m_elevator.GoToHeight(kElevatorL3Position),
            frc2::cmd::Sequence(
                m_algae.SetGoalAngle(kAlgaeCollect),
                m_algae.SetSpeed(kAlgaeCollectSpeed)
            )
        ).WithName(commandName);
    }

    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(m_elevatorSetpointHeight),
        m_coral.GoToAngle(coralAngle),
        m_algae.SetGoalAngle(algaeAngle)
    ).WithName(commandName);
}