
#include <subsystems/CoralSubsystem.h>

#include <frc/smartdashboard/SmartDashboard.h>


CoralSubsystem::CoralSubsystem(ICoralIntakeDataProvider* dataProvider) :
m_intakeMotor(kCoralIntakeMotorID, rev::spark::SparkMax::MotorType::kBrushless), 
m_pivotMotor(kCoralPivotMotorID, rev::spark::SparkMax::MotorType::kBrushless), 
m_coralDataProvider(dataProvider)
{
    m_setpointAngle = 0.0;
}

void CoralSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("Coral Setpoint", m_setpointAngle);
    SetPivotSpeed(-0.025 * m_coralPID.Calculate(m_coralDataProvider->GetCoralIntakeAngleDegrees(), m_setpointAngle));
}

//Returns true if a coral is collected and false otherwise
bool CoralSubsystem::CoralPresent() {
    return m_coralDataProvider->IsCoralCollected();
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
    m_pivotMotor.Set(speed * kSlowDown * kSlowDown);
}

//determine if scoring coral on left or right
// getLRstatus is true when the robot needs to slide 13 in to the right to score coral
void CoralSubsystem::PrepareCoralSide(bool currentSide) {
if (currentSide != getLRStatus){
    getLRStatus = currentSide;
    };
}

/**
 * Set the desired setpoint angle for the coral scoring mechanism. This function only
 * changes the saved setpoint. The Periodic function is responsible for tracking the
 * setpoint and holding the motor at the angle.
 * 
 * @param targetAngle The desired angle of the mechanism in degrees
 */
//Set the angle of the coral scoring mechanism. Requires the desired angle as a parameter  
void CoralSubsystem::GoToAngle(double targetAngle) {
    m_setpointAngle = targetAngle;
}

//Start the intake motor and stop it when the coral is collected
frc2::CommandPtr CoralSubsystem::collectCoral() {
    return frc2::cmd::StartEnd(
        [this] {
            SetIntakeSpeed(0.5);
        }, 
        [this] {
            m_intakeMotor.StopMotor();
        }, 
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
            SetPivotSpeed(-0.0025 * m_coralPID.Calculate(m_coralDataProvider->GetCoralIntakeRawAngleDegrees(), -262.6));
        }, 
        {this}
    ).WithTimeout(1_s).WithName("dispenseCoral");
}