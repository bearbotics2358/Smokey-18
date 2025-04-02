// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @file Robot.h
 */
#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include "RobotContainer.h"

#include <io/FeatherCanDecoder.h>

/**
 * @brief Runs the robot code.
 * @attention All of the subsystem declaration and trigger bindings should happen in
 * Robot Container.
 * @see RobotContainer.h
 */
class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

  /**
   * @brief Represents the device that decodes CAN messages to get various sensor values.
   * @note This variable will be passed into various subsystems as a pointer.
   */
  FeatherCanDecoder m_featherCanDecoder;
 private:
  frc2::Command *m_autonomousCommand;

  RobotContainer m_container;
};
