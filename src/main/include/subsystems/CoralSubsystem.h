
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include "subsystems/ICoralIntakeDataProvider.h"

constexpr units::degree_t kCoralCollect = 135_deg;
constexpr units::degree_t kCoralStow = 157_deg;
constexpr units::degree_t kCoralL1 = 65.0_deg;
constexpr units::degree_t kCoralL2 = 55.0_deg;
constexpr units::degree_t kCoralL3 = 55.0_deg;
constexpr units::degree_t kCoralL4 = 30_deg;

class CoralSubsystem : public frc2::SubsystemBase {
 public:
  CoralSubsystem(ICoralIntakeDataProvider* dataProvider);

  void Periodic() override;

  void SetIntakeSpeed(double speed);
  void SetPivotSpeed(double speed);

  bool CoralPresent();

  frc2::CommandPtr GoToAngle(units::degree_t angle);
  frc2::CommandPtr Collect();
  frc2::CommandPtr Dispense();
  frc2::CommandPtr StopIntake();

 private:
    // Per the REV documentation, the Neo 550 has 42 counts per revolution
    static constexpr double kNeoCountsPerRev = 42;

    // @todo Get the correct gear ratio
    static constexpr double kCoralPivotGearRatio = 125.0;

    const int kPivotMotorMaxCurrentAmps = 10;
    const int ANGLE_TOLERANCE = 3.0;
    const int kCoralPivotMotorID = 58;
    const int kCoralIntakeMotorID = 61;

    double m_setpointAngle;

    const double kCoralP = 0.0085;
    const double kCoralI = 0.0;
    const double kCoralD = 0.0005;

    rev::spark::SparkMax m_intakeMotor;
    rev::spark::SparkMax m_pivotMotor;
    rev::spark::SparkRelativeEncoder m_pivotEncoder;
    double m_relEncoderOffsetDegrees = 0.0;

    ICoralIntakeDataProvider* m_coralDataProvider;

    frc::PIDController m_coralPID{kCoralP, kCoralI, kCoralD};

    units::degree_t GetAngleDegreesFallback();
};