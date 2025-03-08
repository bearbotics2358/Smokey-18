#include "subsystems/Climber.h"
#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber(IClimberDataProvider* dataProvider):
m_climberMotor(kClimberMotor1Id),
m_climberDataProvider(dataProvider)
{
    ctre::phoenix6::configs::MotorOutputConfigs motorConfigs;
    motorConfigs
        .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        .WithInverted(true);  // We'll need to test this

    m_climberMotor.GetConfigurator().Apply(motorConfigs);
    
    m_climberMotor.SetPosition(0_tr);
    /*
     * This method blocks the current robot loop until the signal is retrieved or the timeout is activated.
     * The CTRE docs state that this API can ensure that set operations are completed before continuing control flow.
     * This method reports an error to the DriverStation.
     * The link: https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html
     */ 
    m_climberMotor.GetPosition().WaitForUpdate(20_ms);
}

void Climber::Periodic() {
    frc::SmartDashboard::PutNumber("Climber Angle", CurrentAngle());

    frc::SmartDashboard::PutNumber("Climber Setpoint", m_setpointAngle.value());

    SetMotorVoltage();
}

// Returns the current angle of the climber in degrees
double Climber::CurrentAngle() {
    return m_climberDataProvider->GetClimberAngleDegrees();
}

frc2::CommandPtr Climber::GoToAngle(units::degree_t desiredAngle){
    return frc2::cmd::RunOnce([this, desiredAngle] {
        m_setpointAngle = desiredAngle;
    });
}

// Sets the motor voltage based on profiled PID calculations
void Climber::SetMotorVoltage() {
    double value = m_climberPID.Calculate(units::degree_t(CurrentAngle()), m_setpointAngle);
    frc::SmartDashboard::PutNumber("Climber PID", value);

    //m_climberMotor.SetVoltage(goalVolts);
}

// Stops the climb motor completely
frc2::CommandPtr Climber::StopClimber() {
    return frc2::cmd::RunOnce([this] {
        m_climberMotor.StopMotor();
    });
}