
#include <subsystems/CoralSubsystem.h>

#include <frc/smartdashboard/SmartDashboard.h>

CoralSubsystem::CoralSubsystem(ICoralIntakeDataProvider* dataProvider) :
m_intakeMotor(kCoralIntakeMotorID, rev::spark::SparkMax::MotorType::kBrushless), 
m_pivotMotor(kCoralPivotMotorID, rev::spark::SparkMax::MotorType::kBrushless), 
m_coralDataProvider(dataProvider)
{
    m_setpointAngle = 160.0;

    rev::spark::SparkBaseConfig pivotConfig;
    pivotConfig.SmartCurrentLimit(kPivotMotorMaxCurrentAmps)
        .SetIdleMode(rev::spark::SparkBaseConfig::kBrake);

    // m_pivotMotor.Configure(pivotConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void CoralSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("Pivot Temp", m_pivotMotor.GetMotorTemperature());
    frc::SmartDashboard::PutBoolean("Pivot Temp Good?", m_pivotMotor.GetMotorTemperature() < 60.0);
    frc::SmartDashboard::PutNumber("Pivot Current", m_pivotMotor.GetOutputCurrent());

    frc::SmartDashboard::PutNumber("Coral Setpoint", m_setpointAngle);

    double pid_calculation = m_coralPID.Calculate(m_coralDataProvider->GetCoralIntakeAngleDegrees(), m_setpointAngle);
    frc::SmartDashboard::PutNumber("Coral PID", pid_calculation);
    SetPivotSpeed(pid_calculation);
}

//Returns true if a coral is collected and false otherwise
bool CoralSubsystem::CoralPresent() {
    return m_coralDataProvider->IsCoralCollected();
}

//Set the speed of the coral collector - parameter should be a value from -1.0 to 1.0
void CoralSubsystem::SetIntakeSpeed(double speed) {
    const double kSlowDown = 0.4;
    m_intakeMotor.Set(speed * kSlowDown);
}

//Set the speed of the coral collector's pivot motor - parameter should be from -1.0 to 1.0
void CoralSubsystem::SetPivotSpeed(double speed) {
    frc::SmartDashboard::PutNumber("Coral Pivot speed", speed);
    m_pivotMotor.Set(-speed);
}

/**
 * Set the desired setpoint angle for the coral scoring mechanism. This function only
 * changes the saved setpoint. The Periodic function is responsible for tracking the
 * setpoint and holding the motor at the angle.
 * 
 * @param targetAngle The desired angle of the mechanism in degrees
 */
//Set the angle of the coral scoring mechanism. Requires the desired angle as a parameter
frc2::CommandPtr CoralSubsystem::GoToAngle(units::degree_t targetAngle) {
    return frc2::cmd::RunOnce([this, targetAngle] {
        m_setpointAngle = targetAngle.value();
    });
}

// Start the intake motor and stop it either when the coral is collected or after three seconds
frc2::CommandPtr CoralSubsystem::collectCoral() {
    return frc2::cmd::StartEnd(
        [this] {
            SetIntakeSpeed(0.5);
        }, 
        [this] {
            m_intakeMotor.StopMotor();
        }, 
        {this}
    ).WithTimeout(3_s).Until(([this] { return CoralPresent(); })).WithName("collectCoral");
}   

// Run the intake motor backwards for one second to dispense held coral
frc2::CommandPtr CoralSubsystem::dispenseCoral() {
    return frc2::cmd::StartEnd(
        [this] {
            SetIntakeSpeed(-0.5);
        }, 
        [this] {
            m_intakeMotor.StopMotor();
        }, 
        {this}
    ).WithTimeout(1_s).WithName("dispenseCoral");
}