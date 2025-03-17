#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CommandSwerveDrivetrain.h"

#include <frc/controller/PIDController.h>

class DriveForwardToScore
    : public frc2::CommandHelper<frc2::Command, DriveForwardToScore> {
public:
    explicit DriveForwardToScore(subsystems::CommandSwerveDrivetrain* drivetrain);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

    swerve::requests::RobotCentric robotOriented = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
        .WithVelocityY(0_mps)
        .WithRotationalRate(0_rad_per_s);

private:
    subsystems::CommandSwerveDrivetrain* m_drivetrain;

    static constexpr double kP = 1.0;
    static constexpr double kI = 0.0;
    static constexpr double kD = 0.0;
    static constexpr units::meters_per_second_t kMaxVelocity = 0.25_mps;

    frc::PIDController m_XAlignmentPID {kP, kI, kD};

    const units::inch_t kTolerance = 2_in;
    const units::inch_t kForwardDistance = 4_in;
    units::inch_t m_targetX;
};