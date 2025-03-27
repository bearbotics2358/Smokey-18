#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CommandSwerveDrivetrain.h"

#include <frc/controller/PIDController.h>

enum DriveDirection {
    Backward = -1,
    Forward  =  1
};

class DriveRobot
    : public frc2::CommandHelper<frc2::Command, DriveRobot> {
public:
    static constexpr units::inch_t kDefaultDistance = 8_in;

    explicit DriveRobot(
        subsystems::CommandSwerveDrivetrain* drivetrain, 
        DriveDirection direction,
        units::inch_t distance = kDefaultDistance
    );
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

    units::inch_t GetTraveledDistance();

private:
    subsystems::CommandSwerveDrivetrain* m_drivetrain;

    swerve::requests::RobotCentric robotOriented = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
        .WithVelocityY(0_mps)
        .WithRotationalRate(0_rad_per_s);
    
    static constexpr double kP = 1.0;
    static constexpr double kI = 0.0;
    static constexpr double kD = 0.0;
    static constexpr units::meters_per_second_t kMaxVelocity = 0.75_mps;

    frc::PIDController m_XAlignmentPID {kP, kI, kD};

    const units::inch_t kTolerance = 1_in;
    units::inch_t m_distance;
    DriveDirection m_direction;
    frc::Pose2d m_initialPose;
};