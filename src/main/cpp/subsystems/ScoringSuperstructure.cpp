#include "subsystems/ScoringSuperstructure.h"

using namespace subsystems;

ScoringSuperstructure::ScoringSuperstructure(ElevatorSubsystem& elevator, CoralSubsystem& coralMech, AlgaeSubsystem& algae):
    m_elevator(elevator),
    m_coral(coralMech),
    m_algae(algae)
{
}

frc2::CommandPtr ScoringSuperstructure::StowCommand() {
    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorStowPosition),
        frc2::cmd::RunOnce([this] {
            m_coral.GoToAngle(160.0);
        })
    ).WithName("Stow");
}

frc2::CommandPtr ScoringSuperstructure::ScoreCoralL1() {
    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL1Position),
        frc2::cmd::RunOnce([this] {
            m_coral.GoToAngle(65.0);
        })
    ).WithName("ScoreL1");
}

frc2::CommandPtr ScoringSuperstructure::ScoreCoralL2() {
    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL2Position),
        frc2::cmd::RunOnce([this] {
            m_coral.GoToAngle(55.0);
        })
    ).WithName("ScoreL2");
}

frc2::CommandPtr ScoringSuperstructure::ScoreCoralL3() {
    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL3Position),
        frc2::cmd::RunOnce([this] {
            m_coral.GoToAngle(55.0);
        })
    ).WithName("ScoreL3");
}

frc2::CommandPtr ScoringSuperstructure::ScoreCoralL4() {
    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL4Position),
        frc2::cmd::RunOnce([this] {
            m_coral.GoToAngle(50.0);
        })
    ).WithName("ScoreL4");
}

frc2::CommandPtr ScoringSuperstructure::SetCoralAngle() {
    return frc2::cmd::RunOnce([this] {
        m_algae.SetGoalAngle(90.0);}
    ).WithName("Intake");
}

frc2::CommandPtr ScoringSuperstructure::SetAlgaeAngle() {
    return frc2::cmd::Parallel(
        m_algae.SetGoalAngle(90.0),
        frc2::cmd::RunOnce([this] {
            m_coral.GoToAngle(125.0);
        })
    ).WithName("Intake");
}