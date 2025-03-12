#include "subsystems/CommandSwerveDrivetrain.h"
#include <frc/RobotController.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace subsystems;

void CommandSwerveDrivetrain::ConfigureAutoBuilder()
{
    auto config = pathplanner::RobotConfig::fromGUISettings();
    pathplanner::AutoBuilder::configure(
        // Supplier of current robot pose
        [this] { return GetState().Pose; },
        // Consumer for seeding pose against auto
        [this](frc::Pose2d const &pose) { return ResetPose(pose); },
        // Supplier of current robot speeds
        [this] { return GetState().Speeds; },
        // Consumer of ChassisSpeeds and feedforwards to drive the robot
        [this](frc::ChassisSpeeds const &speeds, pathplanner::DriveFeedforwards const &feedforwards) {
            return SetControl(
                m_pathApplyRobotSpeeds.WithSpeeds(speeds)
                    .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                    .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY)
            );
        },
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            // PID constants for translation
            pathplanner::PIDConstants{10.0, 0.0, 0.0},
            // PID constants for rotation
            pathplanner::PIDConstants{7.0, 0.0, 0.0}
        ),
        std::move(config),
        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        [] {
            auto const alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
            return alliance == frc::DriverStation::Alliance::kRed;
        },
        this // Subsystem for requirements
    );
}

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
        auto const allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor) {
            SetOperatorPerspectiveForward(
                *allianceColor == frc::DriverStation::Alliance::kRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        }
    }
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

frc2::CommandPtr CommandSwerveDrivetrain::GoToAutoStart(std::string autoName, frc::Pose2d currentPose) {
    using namespace pathplanner;

    // Setting the origin point.
    AutoBuilder::resetOdom(currentPose);

    frc::SmartDashboard::PutString("Auto name", autoName);
    
    frc::Pose2d autoStartingPose = PathPlannerAuto(autoName).getStartingPose();

    // Poses should apparently be in field centric, which autoStartingPose is in.
    std::vector<frc::Pose2d> poses {
        autoStartingPose
    };
    std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

    frc::SmartDashboard::PutNumber("Auto X", waypoints.at(0).anchor.X().value());
    frc::SmartDashboard::PutNumber("Auto Y", waypoints.at(0).anchor.Y().value());

    // // TODO: implement the correct constraints
    PathConstraints constraints(
        1_mps, // Max Velocity
        1_mps_sq, // Max Acceleration
        1_rad_per_s, // Max Angular Velocity
        1_rad_per_s_sq, // Max Angular Acceleration
        12_V, // OPTIONAL PARAMETER: Nominal Voltage
        false // OPTIONAL PARAMETER: whether the constraints be unlimited
    );

    std::shared_ptr<PathPlannerPath> path = std::make_shared<PathPlannerPath>(
        waypoints,
        constraints,
        std::nullopt, // To indicate that we're generating a path on the play
        // TODO: implement the correct GoalEndState
        GoalEndState(0.0_mps, frc::Rotation2d(0_deg))
    );

    // /* This following line of code is for preventing the path from being flipped if the coordinates 
    //  * are already correct; I believe that the path should be flipped based on whether you are in the red alliance
    //  * or not.
    //  * TODO: Is this correct?
    //  */
    path->preventFlipping = true;

    return AutoBuilder::followPath(path);
}