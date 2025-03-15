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

    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(m_elevatorSetpointHeight),
        m_coral.GoToAngle(coralAngle),
        frc2::cmd::Either(
            frc2::cmd::Sequence(
                m_algae.SetGoalAngle(kAlgaeCollect),
                m_algae.Intake()
            ),
            m_algae.SetGoalAngle(algaeAngle),
            [removeAlgae] { return removeAlgae; }
        ),
        m_elevator.WaitUntilElevatorAtHeight().AndThen(m_coral.dispenseCoral())
    ).WithName(commandName);
}

frc2::CommandPtr ScoringSuperstructure::ScoreIntoProcessor() {
    return frc2::cmd::Either(
        frc2::cmd::Sequence(
            frc2::cmd::Parallel(
                m_elevator.GoToHeight(kElevatorProcessorPosition),
                m_algae.SetGoalAngle(kAlgaeDispense)
            ),
            m_elevator.WaitUntilElevatorAtHeight(),
            m_algae.Dispense()
        ),
        frc2::cmd::None(),
        [this] { return m_algae.IsAlgaeStored(); }
    );
}

frc2::CommandPtr ScoringSuperstructure::PrepareAndScoreIntoReef(units::inch_t desiredPosition, bool removeAlgae) {
    return PrepareElevator(desiredPosition).AndThen(ScoreIntoReef(removeAlgae));
}