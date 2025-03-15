#include "subsystems/Climber.h"
#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber(IClimberDataProvider* dataProvider):
m_climberMotor(kClimberMotor1Id),
m_climberDataProvider(dataProvider)
{
    ctre::phoenix6::configs::MotorOutputConfigs motorConfigs;
    motorConfigs
        .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        .WithInverted(true);  //We'll need to tests this

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
    frc::SmartDashboard::PutNumber("Climber Angle", CurrentAngle().value());

    frc::SmartDashboard::PutNumber("Climber Setpoint", m_setpointAngle.value());

    if (CurrentAngle() == 368.500_deg) {
        frc::SmartDashboard::PutBoolean("Climber Angle Good?", false);
    } else {
        frc::SmartDashboard::PutBoolean("Climber Angle Good?", true);
    }

    SetMotorVoltage();
}

//Returns the current angle of the climber in degrees
units::degree_t Climber::CurrentAngle() {
   return units::degree_t(
        m_climberDataProvider->GetClimberAngleDegrees()
   );
}

//Starts the climb
frc2::CommandPtr Climber::Climb() {
    return frc2::cmd::RunOnce([this] {
        m_setpointAngle = kClimberEndAngle;
    });
}

//Cancels the climb
frc2::CommandPtr Climber::Extend() {
    return frc2::cmd::RunOnce([this] {
        m_setpointAngle = kClimberStartAngle;
    });
}

//Sets the climber to the stow position if we happen to start the climb when we don't want to
frc2::CommandPtr Climber::Stow() {
    return frc2::cmd::RunOnce([this] {
        m_setpointAngle = kClimberStowAngle;
    });
}

//Sets the motor voltage based on profiled PID calculations
void Climber::SetMotorVoltage() {
    double value = -m_climberPID.Calculate(CurrentAngle(), m_setpointAngle);
    frc::SmartDashboard::PutNumber("Climber PID", value);

    m_climberMotor.SetVoltage(units::volt_t(value));
}

//Stops the climb motor completely
frc2::CommandPtr Climber::StopClimber() {
    return frc2::cmd::RunOnce([this] {
        m_climberMotor.StopMotor();
    });
}