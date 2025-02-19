
#include <subsystems/CoralSubsystem.h>

CoralSubsystem::CoralSubsystem(FeatherCanDecoder* featherPointer) :
m_intakeMotor(kCoralIntakeMotorPort, rev::spark::SparkMax::MotorType::kBrushless), 
m_pivotMotor(kCoralPivotMotorPort, rev::spark::SparkMax::MotorType::kBrushless), 
m_coralDataProvider(featherPointer)
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

bool CoralSubsystem::GoToAngle(double angle) {
    if(fabs(m_coralDataProvider->GetCoralIntakeAngleDegrees() - angle) > 3.0) {
        CoralSubsystem::SetPivotSpeed(m_coralPID.Calculate(m_coralDataProvider->GetCoralIntakeAngleDegrees(), angle));
        return false;
    } else {
        CoralSubsystem::SetPivotSpeed(0.0);
        return true;
    }
}