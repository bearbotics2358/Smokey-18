
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>

class AlgaeSubsystem : public frc2::SubsystemBase {
 public:
  AlgaeSubsystem();

  frc2::CommandPtr SetSpeed(double speed);

  frc2::CommandPtr GoToAngle(double angle);

 private:
    // TODO: change ids later
    const int MOTOR_LEFT_PORT = 0;
    const int MOTOR_RIGHT_PORT = 1;

    rev::spark::SparkMax m_algaeLeftMotor;
    rev::spark::SparkMax m_algaeRightMotor;
};