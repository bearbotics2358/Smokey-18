#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include "subsystems/CoralSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/AlgaeSubsystem.h"
#include "subsystems/CommandSwerveDrivetrain.h"

#include <map>
#include <tuple>

namespace subsystems {

class ScoringSuperstructure : public frc2::SubsystemBase {
    public:
        enum ScoringSelector {
            L1,
            L2,
            L3AlgaeAndCoral,
            L3AlgaeOnly,
            L4
        };

        ScoringSuperstructure(ElevatorSubsystem& elevator, CoralSubsystem& coralMech, AlgaeSubsystem& algaeMech,
                              CommandSwerveDrivetrain& drivetrain);

        frc2::CommandPtr PrepareScoring(ScoringSelector selectedScore);
        frc2::CommandPtr PrepareAndScoreIntoReef(ScoringSelector selectedScore);

        frc2::CommandPtr ScoreIntoReef();
        frc2::CommandPtr ScoreIntoProcessor();

        frc2::CommandPtr ToCollectPosition();
        frc2::CommandPtr ToStowPosition();

        frc2::CommandPtr DriveToReefForScoring();
        frc2::CommandPtr BackUpAfterScoring();
    private:
        ElevatorSubsystem& m_elevator;
        CoralSubsystem& m_coral;
        AlgaeSubsystem& m_algae;
        CommandSwerveDrivetrain& m_drivetrain;

        ScoringSelector m_selectedScore = L1;

        static constexpr units::second_t kForwardTimeout = 2.2_s;
        static constexpr units::second_t kBackupTimeout = 1_s;

        // Values for the tuple are coral and algae angles.
        std::map<units::inch_t, std::tuple<units::degree_t, units::degree_t>> m_elevatorMap = {
            {kElevatorStowPosition, std::make_tuple(kCoralStow, kAlgaeStowAngle)},
            {kElevatorCollectPosition, std::make_tuple(kCoralStow, kAlgaeStowAngle)},
            {kElevatorL1Position, std::make_tuple(kCoralL1, kAlgaeStowAngle)},
            {kElevatorL2Position, std::make_tuple(kCoralL2, kAlgaeStowAngle)},
            {kElevatorL3Position, std::make_tuple(kCoralL3, kAlgaeStowAngle)},
            {kElevatorL4Position, std::make_tuple(kCoralL4, kAlgaeStowAngle)},
        };

        frc2::CommandPtr ScoreReefL1();
        frc2::CommandPtr ScoreReefL2();
        frc2::CommandPtr ScoreReefL3(bool algaeOnly);
        frc2::CommandPtr ScoreReefL4();

        frc2::CommandPtr DispenseCoralAndMoveBack();
};

}