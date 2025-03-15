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

frc2::CommandPtr ScoringSuperstructure::PrepareElevator(units::inch_t desiredPosition, bool removeAlgae) {
    return frc2::cmd::RunOnce([this, desiredPosition, removeAlgae] {
        m_elevatorSetpointHeight = desiredPosition;
        m_collectAlgae = removeAlgae;
    });
}

frc2::CommandPtr ScoringSuperstructure::ScoreIntoReef() {
    auto [coralAngle, algaeAngle] = m_elevatorMap[m_elevatorSetpointHeight];

    return frc2::cmd::Parallel(
        m_elevator.GoToHeight(m_elevatorSetpointHeight),
        m_coral.GoToAngle(coralAngle),
        frc2::cmd::Either(
            frc2::cmd::Sequence(
                m_algae.SetGoalAngle(kAlgaeCollect),
                m_algae.Intake()
            ),
            m_algae.SetGoalAngle(algaeAngle),
            [this] { return m_collectAlgae; }
        ),
        m_elevator.WaitUntilElevatorAtHeight().AndThen(m_coral.dispenseCoral())
    );
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
    return PrepareElevator(desiredPosition, removeAlgae).AndThen(ScoreIntoReef());
}

frc2::CommandPtr ScoringSuperstructure::ToCollectPosition() {
    return m_elevator.GoToHeight(m_algae.IsAlgaeStored() ? kElevatorProcessorPosition : kElevatorCollectPosition);
}

frc2::CommandPtr ScoringSuperstructure::ToStowPosition() {
    return m_elevator.GoToHeight(m_algae.IsAlgaeStored() ? kElevatorProcessorPosition : kElevatorStowPosition);
}
