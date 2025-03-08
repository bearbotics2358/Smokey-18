
#include <subsystems/AlgaeSubsystem.h>
#include <rev/config/SparkBaseConfig.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/Commands.h>

AlgaeSubsystem::AlgaeSubsystem(IAlgaeDataProvider* dataProvider):
m_algaePivotMotor(kAlgaePivot),
m_algaeLeftMotor{kAlgaeMotorLeft, rev::spark::SparkLowLevel::MotorType::kBrushless},
m_algaeRightMotor{kAlgaeMotorRight, rev::spark::SparkLowLevel::MotorType::kBrushless},
m_algaeDataProvider(dataProvider)
{
    rev::spark::SparkBaseConfig config;
    config.Follow(kAlgaeMotorRight, true);
    config.Inverted(true);

    m_algaeLeftMotor.Configure(config,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters
    );
}

void AlgaeSubsystem::Periodic() {
    GoToAngle();
    frc::SmartDashboard::PutNumber("Algae Angle", CurrentAngle().value());
}

frc2::CommandPtr AlgaeSubsystem::SetSpeed(double speed) {
    return frc2::cmd::RunOnce([this, speed] {
        m_algaeRightMotor.Set(speed);
    });
}

frc2::CommandPtr AlgaeSubsystem::SetGoalAngle(double angle) {
    return frc2::cmd::RunOnce([this, angle] {
        m_setpointAngle = units::turn_t(angle);
    });
}

units::degree_t AlgaeSubsystem::CurrentAngle() {
    // return m_algaePivotMotor.GetPosition().GetValue()
    return units::degree_t(m_algaeDataProvider->GetAlgaeAngleDegrees());
};

void AlgaeSubsystem::GoToAngle() {
    double value = m_algaePID.Calculate(CurrentAngle(), m_setpointAngle);
    frc::SmartDashboard::PutNumber("Algae PID", value);

    units::volt_t goalVolts = units::volt_t(value);
    frc::SmartDashboard::PutNumber("Algae PID in Volts", goalVolts.value());

    double current_difference = fabs(m_setpointAngle.value() - CurrentAngle().value());
    // TODO: Uncomment
    // m_algaePivotMotor.SetVoltage(goalVolts); //current_difference >= TOLERANCE  ? 

    frc::SmartDashboard::PutNumber("Algae diff", current_difference);
}