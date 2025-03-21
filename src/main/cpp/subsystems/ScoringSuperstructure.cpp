#include "subsystems/ScoringSuperstructure.h"
#include "commands/DriveForwardToScore.h"
#include "commands/DriveBackAfterScore.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/auto/NamedCommands.h>

using namespace subsystems;

ScoringSuperstructure::ScoringSuperstructure(
    ElevatorSubsystem& elevator,
    CoralSubsystem& coralMech,
    AlgaeSubsystem& algaeMech,
    CommandSwerveDrivetrain& drivetrain
):
m_elevator(elevator),
m_coral(coralMech),
m_algae(algaeMech),
m_drivetrain(drivetrain)
{
    AddPathPlannerCommands();
}

frc2::CommandPtr ScoringSuperstructure::PrepareScoring(ScoringSelector selectedScore) {
    return frc2::cmd::RunOnce([this, selectedScore] {
        m_selectedScore = selectedScore;
    });
}

frc2::CommandPtr ScoringSuperstructure::DispenseCoralAndMoveBack() {
    return frc2::cmd::Sequence(
        m_elevator.WaitUntilElevatorAtHeight(),
        DriveForwardToScore(&m_drivetrain).WithTimeout(kForwardTimeout),
        m_coral.dispenseCoral(),
        DriveBackAfterScore(&m_drivetrain).WithTimeout(kBackupTimeout),
        ToStowPosition()
    );
}

frc2::CommandPtr ScoringSuperstructure::ScoreIntoReef() {
    return frc2::cmd::Select<ScoringSelector>([this] {
            return m_selectedScore;
        },
        std::pair{L1, ScoreReefL1()},
        std::pair{L2, ScoreReefL2()},
        std::pair{L3AlgaeAndCoral, ScoreReefL3(false)},
        std::pair{L3AlgaeOnly, ScoreReefL3(true)},
        std::pair{L4, ScoreReefL4()});
}

frc2::CommandPtr ScoringSuperstructure::ScoreReefL1() {
    auto [coralAngle, algaeAngle] = m_elevatorMap[kElevatorL1Position];

    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL1Position),
        m_coral.GoToAngle(coralAngle),
        DispenseCoralAndMoveBack()
    );
}

frc2::CommandPtr ScoringSuperstructure::ScoreReefL2() {
    auto [coralAngle, algaeAngle] = m_elevatorMap[kElevatorL2Position];

    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL2Position),
        m_coral.GoToAngle(coralAngle),
        DispenseCoralAndMoveBack()
    );
}

frc2::CommandPtr ScoringSuperstructure::ScoreReefL3(bool algaeOnly) {
    auto [coralAngle, algaeAngle] = m_elevatorMap[kElevatorL3Position];

    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL3Position),
        m_coral.GoToAngle(coralAngle),
        frc2::cmd::Either(
            frc2::cmd::Sequence(
                m_algae.SetGoalAngle(kAlgaeExtendedAngle),
                m_algae.Intake()
            ),
            m_algae.SetGoalAngle(algaeAngle),
            [this, algaeOnly] { return algaeOnly; }
        ),
        DispenseCoralAndMoveBack()
    );
}

frc2::CommandPtr ScoringSuperstructure::ScoreReefL4() {
    auto [coralAngle, algaeAngle] = m_elevatorMap[kElevatorL4Position];

    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL4Position),
        m_coral.GoToAngle(coralAngle),
        DispenseCoralAndMoveBack()
    );
}

frc2::CommandPtr ScoringSuperstructure::ScoreIntoProcessor() {
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            m_elevator.GoToHeight(kElevatorProcessorPosition),
            m_algae.SetGoalAngle(kAlgaeExtendedAngle)
        ),
        m_elevator.WaitUntilElevatorAtHeight(),
        m_algae.Dispense()
    ).OnlyIf([this] { return m_algae.IsAlgaeStored(); });
}

frc2::CommandPtr ScoringSuperstructure::PrepareAndScoreIntoReef(ScoringSelector selectedScore) {
    return PrepareScoring(selectedScore).AndThen(ScoreIntoReef());
}

frc2::CommandPtr ScoringSuperstructure::ToCollectPosition() {
    return frc2::cmd::Either(
        // If algae is detected, don't let the elevator go all the way down because
        // it hits the bumper
        frc2::cmd::Parallel(
            m_elevator.GoToHeight(kElevatorProcessorPosition),
            m_coral.GoToAngle(kCoralCollect)
        ),
        frc2::cmd::Parallel(
            m_elevator.GoToHeight(kElevatorCollectPosition),
            m_coral.GoToAngle(kCoralCollect)
        ),
        [this] { return m_algae.IsAlgaeStored(); }
    );
}

frc2::CommandPtr ScoringSuperstructure::ToStowPosition() {
    return frc2::cmd::Either(
        // If algae is detected, don't let the elevator go all the way down because
        // it hits the bumper
        frc2::cmd::Parallel(
            m_elevator.GoToHeight(kElevatorProcessorPosition),
            m_coral.GoToAngle(kCoralStow)
        ),
        frc2::cmd::Parallel(
            m_elevator.GoToHeight(kElevatorStowPosition),
            m_coral.GoToAngle(kCoralStow),
            m_algae.SetGoalAngle(kAlgaeStowAngle)
        ),
        [this] { return m_algae.IsAlgaeStored(); }
    );
}

void ScoringSuperstructure::AddPathPlannerCommands() {
    using namespace pathplanner;
    NamedCommands::registerCommand(
        "Stow",
        std::move(ToStowPosition())
    );
    NamedCommands::registerCommand(
        "Collect",
        std::move(ToCollectPosition())
    );
    NamedCommands::registerCommand(
        "ScoreL1",
        std::move(PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L1))
    );
    NamedCommands::registerCommand(
        "ScoreL2",
        std::move(PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L2))
    );
    NamedCommands::registerCommand(
        "ScoreL3",
        std::move(PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L3AlgaeAndCoral))
    );
    NamedCommands::registerCommand(
        "ScoreL4",
        std::move(PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L4))
    );
    NamedCommands::registerCommand(
        "ScoreL3AndRemoveAlgae",
        std::move(PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L3AlgaeAndCoral))
    );
    NamedCommands::registerCommand(
        "ScoreAlgae",
        std::move(ScoreIntoProcessor())
    );
}

