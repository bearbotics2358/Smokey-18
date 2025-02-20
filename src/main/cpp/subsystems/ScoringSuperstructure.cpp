#include "subsystems/ScoringSuperstructure.h"

using namespace subsystems;

ScoringSuperstructure::ScoringSuperstructure(ElevatorSubsystem& elevator, CoralSubsystem& coralMech):
    m_elevator(elevator),
    m_coralMech(coralMech)
{
}

frc2::CommandPtr ScoringSuperstructure::StowCommand() {
    return frc2::cmd::Parallel(
        m_elevator.SetPositionCommand(kElevatorStowPosition)

        // @todo Add other parallel commands to stow the coral and algae subsystems
    ).WithName("Stow");
}

frc2::CommandPtr ScoringSuperstructure::ScoreCoralL1Command() {
    return frc2::cmd::Parallel(
        m_elevator.SetPositionCommand(kElevatorL1Position)

        // @todo Add other commands to position the coral and algae subsystems
    ).WithName("ScoreL1");
}

frc2::CommandPtr ScoringSuperstructure::ScoreCoralL2Command() {
    return frc2::cmd::Parallel(
        m_elevator.SetPositionCommand(kElevatorL2Position)

        // @todo Add other commands to position the coral and algae subsystems
    ).WithName("ScoreL2");
}

frc2::CommandPtr ScoringSuperstructure::ScoreCoralL3Command() {
    return frc2::cmd::Parallel(
        m_elevator.SetPositionCommand(kElevatorL3Position)

        // @todo Add other commands to position the coral and algae subsystems
    ).WithName("ScoreL3");
}

frc2::CommandPtr ScoringSuperstructure::ScoreCoralL4Command() {
    return frc2::cmd::Parallel(
        m_elevator.SetPositionCommand(kElevatorL4Position)

        // @todo Add other commands to position the coral and algae subsystems
    ).WithName("ScoreL4");
}