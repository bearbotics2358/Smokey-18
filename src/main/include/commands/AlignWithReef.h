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
    explicit AlignWithReef(CameraSubsystem* camera, subsystems::CommandSwerveDrivetrain* drivetrain);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

    swerve::requests::RobotCentric robotOriented = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

private:

    CameraSubsystem* m_camera;
    subsystems::CommandSwerveDrivetrain* m_drivetrain;

    static constexpr units::meters_per_second_t kMaxVelocity = 0.25_mps;
    static constexpr units::meters_per_second_squared_t kMaxAcceleration = 0.25_mps_sq;
    static constexpr units::radians_per_second_t kMaxAngularVelocity = 1.0_rad_per_s;
    static constexpr units::turns_per_second_squared_t kMaxAngularAcceleration = 1.0_rad_per_s_sq;
    static constexpr double kP = 1.0;
    static constexpr double kI = 0.0;
    static constexpr double kD = 0.0;


    frc::TrapezoidProfile<units::meters>::Constraints m_constraints {
        kMaxVelocity, kMaxAcceleration
    };

    frc::TrapezoidProfile<units::radians>::Constraints m_angularConstraints {
        kMaxAngularVelocity, kMaxAngularAcceleration
    };

    frc::PIDController m_XAlignmentPID{kP, kI, kD};
    frc::PIDController m_YAlignmentPID {kP, kI, kD};

    frc::HolonomicDriveController controller{
        frc::PIDController{1, 0, 0}, frc::PIDController{1, 0, 0},
        frc::ProfiledPIDController<units::radian>{
            1, 0, 0, frc::TrapezoidProfile<units::radian>::Constraints{
            6.28_rad_per_s, 3.14_rad_per_s / 1_s}}};

    frc::ProfiledPIDController<units::radians> m_angularAlignmentPID {
        kP, kI, kD, m_angularConstraints
    };

    const units::meter_t kLateralTolerance = 2_in;
    const units::degree_t kRotationTolerance = 3_deg;

    units::meter_t m_distanceFromReefSetpoint = units::meter_t(48_in);
    units::meter_t m_lateralSetpoint = 0_in;
    units::radian_t m_rotationalSetpoint = 0_rad;
};
