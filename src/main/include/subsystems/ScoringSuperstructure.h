/**
 * @file ScoringSuperstructure.h
 */

#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include "subsystems/CoralSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/AlgaeSubsystem.h"
#include "subsystems/CommandSwerveDrivetrain.h"

#include <map>
#include <tuple>

/**
 * @note This namespace solely contains the ScoringSuperstructure class.
 */
namespace subsystems {

/**
 * @brief This subsystem aggregates multiple subsystems to easily create command compositions
 * that perform robot actions like scoring a coral.
 * @details This subsystem includes the @ref Elevator, 
 * @ref Coral, @ref Algae, and @ref CommandSwerveDrivetrain subsystems.
 */
class ScoringSuperstructure : public frc2::SubsystemBase {
    public:
        /**
         * @brief This enum details all the possible reef scoring states.
         */
        enum ScoringSelector {
            L1,
            L2,
            L3AlgaeAndCoral,
            L3AlgaeOnly,
            L4
        };

        ScoringSuperstructure(ElevatorSubsystem& elevator, CoralSubsystem& coralMech, AlgaeSubsystem& algaeMech, 
                              CommandSwerveDrivetrain& drivetrain);

        /**
         * @brief This function is intended to be called by the operator controller to 
         * save the desired scoring setpoint, which is **not** immediately executed.
         * @note Only the driver controller is allowed to execute the scoring sequence,
         * and the driver calls the saved scoring sequence.
         */
        frc2::CommandPtr PrepareScoring(ScoringSelector selectedScore);

        /**
         * @brief This function saves and immediately executes the scoring sequence.
         * @attention This function is only meant to be called during **autonomous**. 
         */
        frc2::CommandPtr PrepareAndScoreIntoReef(ScoringSelector selectedScore);

        /**
         * @brief This function executes the saved scoring sequence and automatically
         * selects the appropriate function.
         */
        frc2::CommandPtr ScoreIntoReef();

        /**
         * @brief This function will be called when
         * @ref ScoringSelector::L3AlgaeOnly is inputted into
         * @ref ScoringSuperstructure::PrepareScoring.
         */
        frc2::CommandPtr RemoveAlgaeL3();
        
        /**
         * @brief This function moves the robot to its collect coral position.
         */
        frc2::CommandPtr ToCollectPosition();

        /**
         * @brief This function moves the robot to its stow position.
         */
        frc2::CommandPtr ToStowPosition();

        /**
         * @deprecated This function is unused and will be removed.
         */
        frc2::CommandPtr DriveToReefForScoring();

        /**
         * @deprecated This function is unused and will be removed.
         */
        frc2::CommandPtr BackUpAfterScoring();

        /**
         * @brief This function stops the swerves.
         */
        frc2::CommandPtr StopDriving();

        /**
         * @brief This function will be called when the driver wants to stop the 
         * current scoring sequence.
         */
        frc2::CommandPtr CancelScore();

        /**
         * @brief This function returns the Elevator's `WaitTillElevatorAtHeight()` 
         * function.
         */
        frc2::CommandPtr WaitTillElevatorAtHeight();
        
    private:
        ElevatorSubsystem& m_elevator;
        CoralSubsystem& m_coral;
        AlgaeSubsystem& m_algae;
        CommandSwerveDrivetrain& m_drivetrain;

        ScoringSelector m_selectedScore = L1;

        static constexpr units::second_t kForwardTimeout = 0.5_s;
        static constexpr units::second_t kBackupTimeout = 1_s;

        swerve::requests::RobotCentric stopDriving = swerve::requests::RobotCentric{}
            .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
            .WithVelocityX(0_mps)
            .WithVelocityY(0_mps)
            .WithRotationalRate(0_rad_per_s);

        // Values for the tuple are coral and algae angles.
        std::map<units::inch_t, std::tuple<units::degree_t, units::degree_t>> m_elevatorMap = {
            {kElevatorStowPosition, std::make_tuple(kCoralStow, kAlgaeStowAngle)},
            {kElevatorCollectPosition, std::make_tuple(kCoralStow, kAlgaeStowAngle)},
            {kElevatorL1Position, std::make_tuple(kCoralL1, kAlgaeStowAngle)},
            {kElevatorL2Position, std::make_tuple(kCoralL2, kAlgaeStowAngle)},
            {kElevatorL3Position, std::make_tuple(kCoralL3, kAlgaeStowAngle)},
            {kElevatorAlgaeOnlyL3Position, std::make_tuple(kCoralStow, kAlgaeExtendedAngle)},
            {kElevatorL4Position, std::make_tuple(kCoralL4, kAlgaeStowAngle)},
        };

        frc2::CommandPtr ScoreReefL1();
        frc2::CommandPtr ScoreReefL2();
        frc2::CommandPtr ScoreReefL3(bool algaeOnly);
        frc2::CommandPtr ScoreReefL4();

        frc2::CommandPtr DispenseCoralAndMoveBack();
};

}