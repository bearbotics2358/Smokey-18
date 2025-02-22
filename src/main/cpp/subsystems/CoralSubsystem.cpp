
#include <subsystems/CoralSubsystem.h>

#include <frc/smartdashboard/SmartDashboard.h>

CoralSubsystem::CoralSubsystem(ICoralIntakeDataProvider* dataProvider) :
m_intakeMotor(kCoralIntakeMotorID, rev::spark::SparkMax::MotorType::kBrushless), 
m_pivotMotor(kCoralPivotMotorID, rev::spark::SparkMax::MotorType::kBrushless), 
m_coralDataProvider(dataProvider)
{
    this->SetDefaultCommand(GoToAngle(100));
}

//Returns true if a coral is collected and false otherwise
bool CoralSubsystem::CoralPresent() {
    return m_coralDataProvider->IsCoralCollected();
}

//Returns the angle of the coral manipulator as a double
double CoralSubsystem::GetAngle() {
    return m_coralDataProvider->GetCoralIntakeAngleDegrees();
}

//Set the speed of the coral collector - parameter should be a value from -1.0 to 1.0
void CoralSubsystem::SetIntakeSpeed(double speed) {
    const double kSlowDown = 0.2;
    m_intakeMotor.Set(speed * kSlowDown);
}

//Set the speed of the coral collector's pivot motor - parameter should be from -1.0 to 1.0
void CoralSubsystem::SetPivotSpeed(double speed) {
    frc::SmartDashboard::PutNumber("Coral Pivot speed", speed);
    const double kSlowDown = 0.2;
    m_pivotMotor.Set(speed * kSlowDown);
}

//Set the angle of the coral scoring mechanism. Requires the desired angle as a parameter
frc2::CommandPtr CoralSubsystem::GoToAngle(double targetAngle) {
    targetAngle -= 262.6;
    return frc2::cmd::Run(
        [this, targetAngle] {
            SetPivotSpeed(-0.025 * m_coralPID.Calculate(m_coralDataProvider->GetCoralIntakeAngleDegrees(), targetAngle));
        },
        {this}
    );
}

//Start the intake motor and stop it when the coral is collected
frc2::CommandPtr CoralSubsystem::collectCoral() {
    return frc2::cmd::RunEnd(
        [this] {
            SetIntakeSpeed(0.5);
            SetPivotSpeed(-0.025 * m_coralPID.Calculate(m_coralDataProvider->GetCoralIntakeAngleDegrees(), m_coralDataProvider->GetCoralIntakeAngleDegrees()));
        }, 
        [this] {SetIntakeSpeed(0.0);}, 
        {this}
    ).WithTimeout(3_s).WithName("collectCoral");
}   

//Run the intake motor backwards for 1 second to dispense held coral
frc2::CommandPtr CoralSubsystem::dispenseCoral() {
    return frc2::cmd::StartEnd(
        [this] {
            SetIntakeSpeed(-0.5);
        }, 
        [this] {
            m_intakeMotor.StopMotor();
            SetPivotSpeed(-0.0025 * m_coralPID.Calculate(m_coralDataProvider->GetCoralIntakeAngleDegrees(), -262.6));
        }, 
        {this}
    ).WithTimeout(1_s).WithName("dispenseCoral");
}