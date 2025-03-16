
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include "subsystems/ICoralIntakeDataProvider.h"

constexpr units::degree_t kCoralStow = 160.0_deg;
constexpr units::degree_t kCoralL1 = 65.0_deg;
constexpr units::degree_t kCoralL2 = 55.0_deg;
constexpr units::degree_t kCoralL3 = 55.0_deg;
constexpr units::degree_t kCoralL4 = 50.0_deg;

class CoralSubsystem : public frc2::SubsystemBase {
 public:
  CoralSubsystem(ICoralIntakeDataProvider* dataProvider);

  void Periodic() override;

  void SetIntakeSpeed(double speed);
  void SetPivotSpeed(double speed);

  bool CoralPresent();

  frc2::CommandPtr GoToAngle(units::degree_t angle);
  frc2::CommandPtr collectCoral();
  frc2::CommandPtr dispenseCoral();

 private:
    const int kPivotMotorMaxCurrentAmps = 10;
    const int ANGLE_TOLERANCE = 3.0;
    const int kCoralPivotMotorID = 58;
    const int kCoralIntakeMotorID = 61;

    double m_setpointAngle;

    const double kCoralP = 0.01;
    const double kCoralI = 0.0;
    const double kCoralD = 0.0005;

    rev::spark::SparkMax m_intakeMotor;
    rev::spark::SparkMax m_pivotMotor;

    ICoralIntakeDataProvider* m_coralDataProvider;

    frc::PIDController m_coralPID{kCoralP, kCoralI, kCoralD};
};