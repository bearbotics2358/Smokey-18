// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @file AlignWithReef.h
 */

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

/**
 * @var ReefSide
 * @brief An enum representing the two sides of the reef, left or right.
 */
enum ReefSide {
    Left,
    Right
};

/**
 * @brief A class to align the robot to the best april tag found by the camera.
 */
class AlignWithReef
    : public frc2::CommandHelper<frc2::Command, AlignWithReef> {
public:
    /**
     * @brief The constructor that takes in a pointer to the camera and drive subsystems and the side of the reef to align to.
     * @param camera The camera subsystem.
     * @param drivetrain The drivetrain.
     * @param reefSide The side of the reef to align to.
     */
    explicit AlignWithReef(CameraSubsystem* camera, subsystems::CommandSwerveDrivetrain* drivetrain, ReefSide reefSide);

    void Initialize() override;
    /**
     * @details Runs until IsFinished returns true. Doesn't run at all if no april tags are present.
     */
    void Execute() override;
    /**
     * @details Stops the command when there are no april tags present or 
     * when the robot is within the forward, strafe, and rotation tolerances.
     */
    bool IsFinished() override;

    /**
     * @details This swerve request ensures the robot only moves in a robot oriented fashion,
     * where X is forward and Y is side to side.
     */
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

    static constexpr units::meters_per_second_t kMaxVelocity = 1.75_mps;
    static constexpr units::radians_per_second_t kMaxAngularVelocity = 1_rad_per_s;

    const units::meter_t kForwardTolerance = units::meter_t(3_in);
    const units::meter_t kStrafeTolerance = units::meter_t(0.5_in);
    const units::degree_t kRotationTolerance = 2_deg;

    const units::meter_t kDistanceFromReefSetpoint = units::meter_t(30_in);
    const units::meter_t kStrafeLeftReefSetpoint = units::meter_t(-1_in);
    const units::meter_t kStrafeRightReefSetpoint = units::meter_t(kStrafeLeftReefSetpoint + 13_in);
    units::meter_t m_strafeSetpoint = kStrafeLeftReefSetpoint;

    /**
     * @brief Used for desired robot orientation at the end of alignment (in degrees).
     */
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
};
