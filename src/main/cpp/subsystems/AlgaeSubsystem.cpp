
#include <subsystems/AlgaeSubsystem.h>
#include <rev/config/SparkBaseConfig.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/Commands.h>

AlgaeSubsystem::AlgaeSubsystem():
m_algaeLeftMotor{kAlgaeMotorLeft, rev::spark::SparkLowLevel::MotorType::kBrushless},
m_algaeRightMotor{kAlgaeMotorRight, rev::spark::SparkLowLevel::MotorType::kBrushless},
m_algaePivotMotor(kAlgaePivot)
{
    rev::spark::SparkBaseConfig followerConfig;
    followerConfig.Follow(kAlgaeMotorRight, true);

    // TODO: Configure these variables later.
    rev::spark::SparkBase::ResetMode resetMode;
    rev::spark::SparkBase::PersistMode persistMode;

    m_algaeLeftMotor.Configure(followerConfig, resetMode, persistMode);
}

frc2::CommandPtr AlgaeSubsystem::SetSpeed(double speed) {
    return frc2::cmd::RunOnce([this, speed] {
        
    });
}

frc2::CommandPtr AlgaeSubsystem::GoToAngle(double angle) {
    return frc2::cmd::RunOnce([this, angle] {

    });
}

units::turn_t AlgaeSubsystem::CurrentAngle() {
    return m_algaePivotMotor.GetPosition().GetValue();
};

void AlgaeSubsystem::SetAlgaeVoltage () {
    double value = m_algaePID.Calculate(CurrentAngle(), m_setpointAngle);
    frc::SmartDashboard::PutNumber("Algae PID", value);

    units::volt_t goalVolts = units::volt_t(value);
    frc::SmartDashboard::PutNumber("Algae PID with feedforward", goalVolts.value());

    double current_difference = fabs(m_setpointAngle.value() - CurrentAngle().value());
    if (current_difference >= TOLERANCE) {
        m_algaePivotMotor.SetVoltage(goalVolts);
    } else {
        m_algaePivotMotor.SetVoltage(0.0_V);
    }

    frc::SmartDashboard::PutNumber("Algae diff", current_difference);
}