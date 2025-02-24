
#include <subsystems/CoralSubsystem.h>

CoralSubsystem::CoralSubsystem(ICoralIntakeDataProvider* dataProvider) :
m_intakeMotor(kCoralIntakeMotorID, rev::spark::SparkMax::MotorType::kBrushless), 
m_pivotMotor(kCoralPivotMotorID, rev::spark::SparkMax::MotorType::kBrushless), 
m_coralDataProvider(dataProvider)
{
  
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
    m_intakeMotor.Set(speed);
}

//Set the speed of the coral collector's pivot motor - parameter should be from -1.0 to 1.0
void CoralSubsystem::SetPivotSpeed(double speed) {
    m_pivotMotor.Set(speed);
}

//determine if scoring coral on left or right
void CoralSubsystem::PrepareCoralSide(bool currentSide) {
if (currentSide != getLRStatus);
    getLRStatus = currentSide;
}

//Set the angle of the coral scoring mechanism. Requires the desired angle as a parameter
frc2::CommandPtr CoralSubsystem::GoToAngle(double targetAngle) {
    return frc2::cmd::RunEnd(
        [this, targetAngle] { 
            SetPivotSpeed(m_coralPID.Calculate(m_coralDataProvider->GetCoralIntakeAngleDegrees(), targetAngle));
        },
        [this] { SetPivotSpeed(0.0); },
        {this}
    ).Until([this, targetAngle] {
        return fabs(m_coralDataProvider->GetCoralIntakeAngleDegrees() - targetAngle) <= ANGLE_TOLERANCE;
    });
}

//Start the intake motor and stop it when the coral is collected
frc2::CommandPtr CoralSubsystem::collectCoral() {
    return frc2::cmd::StartEnd(
        [this] {SetIntakeSpeed(0.5);}, 
        [this] {SetIntakeSpeed(0.0);}, 
        {this}
    ).Until(([this] {return CoralPresent();})).WithName("collectCoral");
}

//Run the intake motor backwards for 1 second to dispense held coral
frc2::CommandPtr CoralSubsystem::dispenseCoral() {
    return frc2::cmd::StartEnd(
        [this] {SetIntakeSpeed(-0.5); }, 
        [this] {SetIntakeSpeed(0.0); }, 
        {this}
    ).WithTimeout(1_s).WithName("dispenseCoral");
}