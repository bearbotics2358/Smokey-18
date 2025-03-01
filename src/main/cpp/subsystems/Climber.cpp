#include "subsystems/Climber.h"
#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber():
    m_climberMotor(kClimberMotor1Id)
{
    ctre::phoenix6::configs::MotorOutputConfigs motorConfigs;
    motorConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        .WithInverted(true)
        // Limit the forward and reverse duty cycle while getting the climber working.
        .WithPeakForwardDutyCycle(kSlowClimber)
        .WithPeakReverseDutyCycle(kSlowClimber);

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
    frc::SmartDashboard::PutNumber("Raw Climber Turns", CurrentTurns().value());

    frc::SmartDashboard::PutNumber("Climber Setpoint Degrees", m_setpointAngle.value());

    SetMotorVoltage();
}

units::turn_t Climber::CurrentTurns() {
    return units::turn_t(m_climberMotor.GetPosition().GetValueAsDouble());
}

frc2::CommandPtr Climber::Climb() {
    return frc2::cmd::RunOnce([this] {
        m_setpointAngle = kClimberEndAngle;
    });
}

frc2::CommandPtr Climber::CancelClimb() {
    return frc2::cmd::RunOnce([this] {
        m_setpointAngle = kClimberStartAngle;
    });
}

frc2::CommandPtr Climber::Stow() {
    return frc2::cmd::RunOnce([this] {
        m_setpointAngle = kClimberStowAngle;
    });
}

void Climber::SetMotorVoltage() {
    units::turn_t targetTurns = m_setpointAngle * (kClimberGearRatio * 1_tr / 1_deg);
    frc::SmartDashboard::PutNumber("Climber Setpoint Turns", targetTurns.value());

    double value = m_elevatorPID.Calculate(CurrentTurns(), targetTurns);
    frc::SmartDashboard::PutNumber("Climber PID", value);

    units::volt_t goalVolts = units::volt_t(value);
    frc::SmartDashboard::PutNumber("Climber PID in Volts", goalVolts.value());

    //m_climberMotor.SetVoltage(goalVolts);
}

frc2::CommandPtr Climber::StopClimbMotor() {
    return frc2::cmd::RunOnce([this] {m_climberMotor.StopMotor();});
}