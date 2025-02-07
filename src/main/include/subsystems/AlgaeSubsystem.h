
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>

class AlgaeSubsystem : public frc2::SubsystemBase {
 public:
  AlgaeSubsystem();

  void setSpeed(double speed);

  double getAngle();
  void goToAngle(double angle);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // TODO These need to be declared in constants.h when it gets merged
    #define CORALMOTORPORT = 0
    #define PIVOTMOTORPORT = 1

    rev::spark::SparkMax m_algaeMotor;

};