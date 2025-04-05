#include "subsystems/ScoringSuperstructure.h"
#include "commands/DriveForwardToScore.h"
#include "commands/DriveBackAfterScore.h"

#include <frc/smartdashboard/SmartDashboard.h>

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
{}

frc2::CommandPtr ScoringSuperstructure::PrepareScoring(ScoringSelector selectedScore) {
    return frc2::cmd::RunOnce([this, selectedScore] {
        m_selectedScore = selectedScore;
    });
}

frc2::CommandPtr ScoringSuperstructure::DispenseCoralAndMoveBack() {
    return frc2::cmd::Sequence(
        m_elevator.WaitUntilElevatorIsCloseEnoughToMove(),
        DriveForwardToScore(&m_drivetrain).WithTimeout(2.0_s),
        m_coral.Dispense(),
        DriveBackAfterScore(&m_drivetrain).WithTimeout(kBackupTimeout),
        ToStowPosition()
    );
}

frc2::CommandPtr ScoringSuperstructure::StopDriving() {
    return frc2::cmd::RunOnce(
        [this] {m_drivetrain.SetControl(stopDriving);}
    );
}

frc2::CommandPtr ScoringSuperstructure::CancelScore() {
    return frc2::cmd::Sequence(
        DriveBackAfterScore(&m_drivetrain).WithTimeout(0.5_s),
        ToStowPosition()
    );
}

frc2::CommandPtr ScoringSuperstructure::ScoreIntoReef() {
    return frc2::cmd::Select<ScoringSelector>([this] {
            return m_selectedScore;
        },
        std::pair{L1, ScoreReefL1()},
        std::pair{L2, ScoreReefL2()},
        std::pair{L3AlgaeAndCoral, ScoreReefL3(true)},
        std::pair{L3AlgaeOnly, RemoveAlgaeL3()},
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

frc2::CommandPtr ScoringSuperstructure::ScoreReefL3(bool removeAlgae) {
    auto [coralAngle, algaeAngle] = m_elevatorMap[kElevatorL3Position];

    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL3Position),
        m_coral.GoToAngle(coralAngle),

        frc2::cmd::Either(
            frc2::cmd::Sequence(
                m_algae.SetGoalAngle(kAlgaeExtendedAngle),
                m_algae.Dispense()
            ),
            m_algae.SetGoalAngle(kAlgaeStowAngle),
            [this, removeAlgae] { return removeAlgae; }
        ),

        DispenseCoralAndMoveBack()
    );
}

frc2::CommandPtr ScoringSuperstructure::RemoveAlgaeL3() {
    auto [_, algaeAngle] = m_elevatorMap[kElevatorAlgaeOnlyL3Position];

    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorAlgaeOnlyL3Position),
        frc2::cmd::Sequence(
            m_algae.SetGoalAngle(kAlgaeExtendedAngle),
            m_algae.Dispense()
        ),
        DriveForwardToScore(&m_drivetrain, 9_in).WithTimeout(2.0_s)
    ).AndThen(DriveBackAfterScore(&m_drivetrain).WithTimeout(2.0_s));
}

frc2::CommandPtr ScoringSuperstructure::ScoreReefL4() {
    auto [coralAngle, algaeAngle] = m_elevatorMap[kElevatorL4Position];

    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorL4Position),
        m_coral.GoToAngle(coralAngle),
        DispenseCoralAndMoveBack()
    ).AndThen(m_elevator.WaitUntilElevatorIsCloseEnoughToMove().WithTimeout(2.0_s));
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
            frc2::cmd::Sequence(
                m_coral.GoToAngle(kCoralCollect),
                m_coral.Collect(),
                ToStowPosition()
            )
        ),
        [this] { return m_algae.IsAlgaeStored(); }
    );
}

frc2::CommandPtr ScoringSuperstructure::ToStowPosition() {
    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(kElevatorStowPosition),
        m_coral.GoToAngle(kCoralStow),
        m_algae.SetGoalAngle(kAlgaeStowAngle)
    );
}

frc2::CommandPtr ScoringSuperstructure::WaitTillElevatorAtHeight() {
    return m_elevator.WaitUntilElevatorIsCloseEnoughToMove();
}
