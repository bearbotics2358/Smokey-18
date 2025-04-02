/**
 * @file RobotContainer.h
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandGenericHID.h>
#include "io/FeatherCanDecoder.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/CameraSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/CoralSubsystem.h"
#include "subsystems/Climber.h"
#include "subsystems/ScoringSuperstructure.h"
#include "subsystems/LED.h"
#include "Telemetry.h"
#include <frc2/command/RunCommand.h>
#include "subsystems/AlgaeSubsystem.h"
#include "commands/AlignWithReef.h"

/**
 * @brief This class is where all of the subsystems are included and commands are 
 * binded to triggers.
 */
class RobotContainer {
private:
    /**
     * @brief Max robot speed.
     */
    units::meters_per_second_t m_maxSpeed = TunerConstants::kSpeedAt12Volts;

    /**
     * @brief This multiplier is intended to change the robot's speed during teleop if 
     * certain conditions are met, like the driver pressing slow mode or the elevator
     * above a certain height threshold.
     */
    double m_speedMultiplier = 1.0;

    /**
     * @brief Max robot angular speed.
     */
    units::radians_per_second_t m_maxAngularRate = 0.75_tps;

    /**
     * @brief Setting up bindings for necessary control of the swerve drive platform 
     * @details This variable moves the robot using the drivetrain's default command.
     * @note A 10% joystick deadband is added, along with open loop control.
     */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(m_maxSpeed * 0.1).WithRotationalDeadband(m_maxAngularRate * 0.1)
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);
    
    /**
     * @brief This variable will immediately place the swerve drives in a X direction to rapidly slow
     * down the robot.
     */
    swerve::requests::SwerveDriveBrake brake{};

    /**
     * @brief This variable will not drive the swerves forward; instead only the angle of the
     * swerve drives will change.
     */
    swerve::requests::PointWheelsAt point{};

    /**
     * @brief This variable will move the robot side to side.
     */
    swerve::requests::RobotCentric strafe = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
        .WithVelocityX(0_mps);

    /**
     * @brief Logs the swerve drive data, allowing for robot simulation.
     * @note This must be constructed before the drivetrain, otherwise we need to
     * define a destructor to un-register the telemetry from the drivetrain 
     */
    Telemetry logger{m_maxSpeed};

    /**
     * @brief The Driver Xbox Controller.
     */
    frc2::CommandXboxController m_driverJoystick{0};

    /**
     * @brief The Operator Xbox Controller.
     */
    frc2::CommandXboxController m_operatorJoystick{1};
    // frc2::CommandGenericHID m_gamepad{4};

    /**
     * @note Robot.cpp owns the FeatherCanDecoder object
     */ 
    FeatherCanDecoder* m_featherCanDecoder;
    LED m_LED;
    CameraSubsystem m_cameraSubsystem;
    ElevatorSubsystem m_elevatorSubsystem;
    CoralSubsystem m_coralSubsystem;
    AlgaeSubsystem m_algaeSubsystem;

    Climber m_climberSubsystem;
    subsystems::ScoringSuperstructure m_scoringSuperstructure;

public:
    subsystems::CommandSwerveDrivetrain m_drivetrain{TunerConstants::CreateDrivetrain()};

private:
    /* Path follower */
    frc::SendableChooser<frc2::Command *> m_autoChooser;

public:
    RobotContainer(FeatherCanDecoder* featherCanDecoder);

    frc2::Command *GetAutonomousCommand();

    frc2::CommandPtr AddControllerRumble(frc::GenericHID::RumbleType rumbleType, double rumble);

    void ResetRobotForAutonomous();
    frc2::CommandPtr ResetRobotForTeleOp();
private:
    void ConfigureBindings();
    void AddPathPlannerCommands();
    
    ReefSide m_reefSide = ReefSide::Left;
};
