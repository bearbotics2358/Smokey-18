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

frc2::CommandPtr ScoringSuperstructure::StowCommand() {
    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorStowPosition)
        // @todo Add other parallel commands to stow the coral and algae subsystems
    ).WithName("Stow");
}

frc2::CommandPtr ScoringSuperstructure::ScoreCoralL1Command() {
    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL1Position)

        // @todo Add other commands to position the coral and algae subsystems
    ).WithName("ScoreL1");
}

frc2::CommandPtr ScoringSuperstructure::ScoreCoralL2Command() {
    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL2Position)

        // @todo Add other commands to position the coral and algae subsystems
    ).WithName("ScoreL2");
}

frc2::CommandPtr ScoringSuperstructure::ScoreCoralL3Command() {
    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL3Position)

        // @todo Add other commands to position the coral and algae subsystems
    ).WithName("ScoreL3");
}

frc2::CommandPtr ScoringSuperstructure::ScoreCoralL4Command() {
    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL4Position)

        // @todo Add other commands to position the coral and algae subsystems
    ).WithName("ScoreL4");
}

frc2::CommandPtr ScoringSuperstructure::PrepareElevator(units::inch_t desiredPosition) {
    return frc2::cmd::RunOnce([this, desiredPosition] {
        m_elevatorSetpointHeight = desiredPosition;
    })
}

frc2::CommandPtr Score(bool removeAlgae) {
    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(m_elevatorSetpointHeight),
        m_coral.GoToAngle(std::get<0>(m_elevatorMap[m_elevatorSetpointHeight])),
        m_algae.SetGoalAngle(std::get<1>(m_elevatorMap[m_elevatorSetpointHeight]))
    )
}