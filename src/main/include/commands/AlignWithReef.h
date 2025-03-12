// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CameraSubsystem.h"
#include "subsystems/CommandSwerveDrivetrain.h"

#include <frc/controller/HolonomicDriveController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Transform2d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>
#include <units/acceleration.h>

class AlignWithReef
    : public frc2::CommandHelper<frc2::Command, AlignWithReef> {
public:
    /**
     * Creates a new ExampleCommand.
     *
     * @param camera The subsystem used by this command.
     * @param drivetrain
     */
    explicit AlignWithReef(CameraSubsystem* camera, subsystems::CommandSwerveDrivetrain* drivetrain, bool useOffsetAlignment);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

    swerve::requests::RobotCentric robotOriented = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);
    swerve::requests::ApplyFieldSpeeds m_fieldSpeedRequest = swerve::requests::ApplyFieldSpeeds{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

private:

    CameraSubsystem* m_camera;
    subsystems::CommandSwerveDrivetrain* m_drivetrain;
    frc::Transform2d m_alignmentTransform;
    std::optional<frc::Pose2d> m_targetPose;

    static constexpr double kP = 1.0;
    static constexpr double kI = 0.1;
    static constexpr double kD = 0.0;

    frc::PIDController m_XAlignmentPID {kP, kI, kD};
    frc::PIDController m_YAlignmentPID {kP, kI, kD};

    static constexpr units::radians_per_second_t kMaxAngularVelocity = 1.0_rad_per_s;
    static constexpr units::turns_per_second_squared_t kMaxAngularAcceleration = 1.0_rad_per_s_sq;
    static constexpr double kRotationP = 1.0;
    static constexpr double kRotationI = 0.1;
    static constexpr double kRotationD = 0.0;

    frc::TrapezoidProfile<units::radians>::Constraints m_angularConstraints {
        kMaxAngularVelocity, kMaxAngularAcceleration
    };
    frc::ProfiledPIDController<units::radians> m_angularAlignmentPID {
        kRotationP, kRotationI, kRotationD, m_angularConstraints
    };

    const units::meter_t kForwardTolerance = units::meter_t(2_in);
    const units::meter_t kStrafeTolerance = units::meter_t(2_in);
    const units::degree_t kRotationTolerance = 3_deg;

    frc::HolonomicDriveController m_holonomicPID {
        frc::PIDController {1, 0, 0},
        frc::PIDController {1, 0, 0},
        frc::ProfiledPIDController<units::radian>{
            1, 0, 0, frc::TrapezoidProfile<units::radian>::Constraints{
            6.28_rad_per_s, 3.14_rad_per_s / 1_s}}};

    const units::meters_per_second_t kMaxLinearVelocity = 0.5_mps;

    // Transforms relative to any AprilTag. These are used to generate the target robot pose based
    // on the AprilTag we see on the reef.
    const units::meter_t kDistanceFromReefSetpoint = units::meter_t(48_in);
    const units::meter_t kStrafeSetpoint = units::meter_t(0_in);
    const units::radian_t kRotationalSetpoint = units::radian_t(180_deg);
    const frc::Transform2d kCenterAlignedTransform{kDistanceFromReefSetpoint, kStrafeSetpoint, kRotationalSetpoint};
    const frc::Transform2d kOffsetAlignedTransform{kDistanceFromReefSetpoint, kStrafeSetpoint + 13_in, kRotationalSetpoint};
};
