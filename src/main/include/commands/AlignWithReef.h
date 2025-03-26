// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CameraSubsystem.h"
#include "subsystems/CommandSwerveDrivetrain.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>
#include <units/acceleration.h>

 enum ReefSide {
    Left,
    Right
};

class AlignWithReef
    : public frc2::CommandHelper<frc2::Command, AlignWithReef> {
public:
    /**
     * Creates a new ExampleCommand.
     *
     * @param camera The subsystem used by this command.
     * @param drivetrain
     */
    explicit AlignWithReef(CameraSubsystem* camera, subsystems::CommandSwerveDrivetrain* drivetrain, ReefSide reefSide);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

    swerve::requests::RobotCentric robotOriented = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);
    
private:
    CameraSubsystem* m_camera;
    subsystems::CommandSwerveDrivetrain* m_drivetrain;

    std::optional<int> m_targetTagId;
    units::degree_t m_targetDegrees;

    static constexpr double kP = 2.0;
    static constexpr double kI = 0.0;
    static constexpr double kD = 0.0;

    frc::PIDController m_XAlignmentPID {kP, kI, kD};
    frc::PIDController m_YAlignmentPID {kP, kI, kD};

    static constexpr double kRotationP = 0.05;
    static constexpr double kRotationI = 0.0;
    static constexpr double kRotationD = 0.0;
    frc::PIDController m_rotationalPID {kRotationP, kRotationI, kRotationD};

    static constexpr units::meters_per_second_t kMaxVelocity = 1.25_mps;
    static constexpr units::radians_per_second_t kMaxAngularVelocity = 0.75_rad_per_s;

    const units::meter_t kForwardTolerance = units::meter_t(2_in);
    const units::meter_t kStrafeTolerance = units::meter_t(0.5_in);
    const units::degree_t kRotationTolerance = 2_deg;

    const units::meter_t kDistanceFromReefSetpoint = units::meter_t(30_in);
    const units::meter_t kStrafeLeftReefSetpoint = units::meter_t(0_in);
    const units::meter_t kStrafeRightReefSetpoint = units::meter_t(kStrafeLeftReefSetpoint + 13_in);
    units::meter_t m_strafeSetpoint = kStrafeLeftReefSetpoint;

    const std::map<int, units::degree_t> kTagAngleMap = {
        {6, 120_deg},
        {7, 180_deg},
        {8, 240_deg},
        {9, 300_deg},
        {10, 0_deg},
        {11, 60_deg},
        {17, 60_deg},
        {18, 0_deg},
        {19, 300_deg},
        {20, 240_deg},
        {21, 180_deg},
        {22, 120_deg},
    };

    ReefSide m_reefSide = ReefSide::Left;
};
