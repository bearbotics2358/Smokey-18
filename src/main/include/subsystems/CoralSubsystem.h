
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include "subsystems/ICoralIntakeDataProvider.h"

class CoralSubsystem : public frc2::SubsystemBase {
 public:
  CoralSubsystem(ICoralIntakeDataProvider* dataProvider);

  void SetIntakeSpeed(double speed);
  void SetPivotSpeed(double speed);
  
  bool getLRStatus = false;
  void PrepareCoralSide(bool currentSide);

  double GetAngle();
  bool CoralPresent();

  frc2::CommandPtr GoToAngle(double angle);
  frc2::CommandPtr collectCoral();
  frc2::CommandPtr dispenseCoral();

    

 private:
    const int ANGLE_TOLERANCE = 3.0;
    //TODO - properly define the motor ports
    const int kCoralPivotMotorID = 0;
    const int kCoralIntakeMotorID = 0;

    const double kCoralP = 0.5;
    const double kCoralI = 0.0;
    const double kCoralD = 0.0;

    rev::spark::SparkMax m_pivotMotor;
    rev::spark::SparkMax m_intakeMotor;

    ICoralIntakeDataProvider* m_coralDataProvider;

    frc::PIDController m_coralPID{kCoralP, kCoralI, kCoralD};
};