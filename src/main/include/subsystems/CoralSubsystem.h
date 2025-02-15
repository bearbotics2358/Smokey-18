
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include "subsystems/ICoralIntakeDataProvider.h"

class CoralSubsystem : public frc2::SubsystemBase {
 public:
  CoralSubsystem(ICoralIntakeDataProvider* coralDataProvider);

  void SetIntakeSpeed(double speed);
  void SetPivotSpeed(double speed);

  double GetAngle();
  bool CoralPresent();
  double CalculatePID(double goal);
  bool GoToAngle(double angle);

 private:

    //TODO - properly define the motor ports
    const int kCoralPivotMotorPort = 0;
    const int kCoralIntakeMotorPort = 0;

    const double kCoralP = 0.5;
    const double kCoralI = 0.0;
    const double kCoralD = 0.0;

    rev::spark::SparkMax m_pivotMotor;
    rev::spark::SparkMax m_intakeMotor;

    ICoralIntakeDataProvider* m_coralDataProvider;

    frc::PIDController coralPID{kCoralP, kCoralI, kCoralD};
};