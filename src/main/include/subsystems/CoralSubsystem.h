/**
 * @file CoralSubsystem.h
 */

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include "subsystems/ICoralIntakeDataProvider.h"

constexpr units::degree_t kCoralCollect = 135_deg;
constexpr units::degree_t kCoralStow = 157_deg;
/**
 * @brief The coral arm angle for scoring at the L1 reef level.
 */

constexpr units::degree_t kCoralL1 = 65.0_deg;
/**
 * @brief The coral arm angle for scoring at the L2 reef level.
 */

constexpr units::degree_t kCoralL2 = 55.0_deg;
/**
 * @brief The coral arm angle for scoring at the L3 reef level.
 */
constexpr units::degree_t kCoralL3 = 55.0_deg;

/**
 * @brief The coral arm angle for scoring at the L4 reef level.
 */
constexpr units::degree_t kCoralL4 = 30_deg;

/**
 * @brief The subsystem controls the coral arm.
 */
class CoralSubsystem : public frc2::SubsystemBase {
 public:
  CoralSubsystem(ICoralIntakeDataProvider* dataProvider);

  void Periodic() override;

  /**
   * @brief Sets the speed of the coral arm.
   * @attention This function is **not** meant to be used for taking in and dispensing coral.
   * @ref CoralSubsystem::Collect and @ref CoralSubsystem::Dispense instead.
   * @param speed The speed should range from -1.0 to 1.0.
   */
  void SetIntakeSpeed(double speed);

  /**
   * @brief Sets the speed of the coral arm's pivot motor.
   * @attention This function is **not** meant to be called, set the goal coral arm angle instead.
   * @param speed The speed should range from -1.0 to 1.0.
   */
  void SetPivotSpeed(double speed);

  /**
   * @brief Returns true if a coral is collected and false otherwise.
   */
  bool CoralPresent();

  /**
   * Set the desired setpoint angle for the coral scoring mechanism. This function only
   * changes the saved setpoint. The Periodic function is responsible for tracking the
   * setpoint and holding the motor at the angle.
   *
   * @param targetAngle The desired angle of the mechanism in degrees
   */
  frc2::CommandPtr GoToAngle(units::degree_t angle);

  /**
   * @brief Starts the intake motor to collect coral and stops it either 
   * when the coral has been collected.
   */
  frc2::CommandPtr Collect();

  /**
   * @brief Starts the intake motor to dispense coral and stops it after 
   * one second.
   */
  frc2::CommandPtr Dispense();

  /**
   * @brief Stops the coral intake motor.
   */
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

    const double kCoralP = 0.0087;
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