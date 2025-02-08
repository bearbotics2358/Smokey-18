
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>

class AlgaeSubsystem : public frc2::SubsystemBase {
 public:
  AlgaeSubsystem();

  void SetSpeed(double speed);

  double GetAngle();
  void GoToAngle(double angle);

 private:
    rev::spark::SparkMax m_algaeMotor;
    rev::spark::SparkMax m_coralMotor;
};