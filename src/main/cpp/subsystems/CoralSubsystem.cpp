
#include <subsystems/CoralSubsystem.h>

CoralSubsystem::CoralSubsystem(ICoralIntakeDataProvider* coralDataProvider):
m_intakeMotor{kCoralIntakeMotorPort, rev::spark::SparkMax::MotorType::kBrushless}, 
m_pivotMotor{kCoralPivotMotorPort, rev::spark::SparkMax::MotorType::kBrushless}, 
m_coralDataProvider{coralDataProvider}
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

//Finds the speed necessary to get to the goal angle efficiently - parameter is the goal angle
double CoralSubsystem::CalculatePID(double goal) {
    return coralPID.Calculate(m_coralDataProvider->GetCoralIntakeAngleDegrees(), goal);
}

bool CoralSubsystem::GoToAngle(double angle) {
    while(fabs(m_coralDataProvider->GetCoralIntakeAngleDegrees() - angle) > 3.0) {
        CoralSubsystem::SetPivotSpeed(CoralSubsystem::CalculatePID(angle));
        return false;
    }
    CoralSubsystem::SetPivotSpeed(0.0);
    return true;
}