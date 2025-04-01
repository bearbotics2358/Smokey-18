
#include <subsystems/AlgaeSubsystem.h>
#include <rev/config/SparkBaseConfig.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/Commands.h>

AlgaeSubsystem::AlgaeSubsystem(IAlgaeDataProvider* dataProvider):
m_algaePivotMotor(kAlgaePivot),
m_algaeMotor{kAlgaeMotor, rev::spark::SparkLowLevel::MotorType::kBrushless},
m_algaeDataProvider(dataProvider),
//todo switch m_algaeMotor to m_algaePivotMotor
m_algaePivotEncoder(m_algaeMotor.GetEncoder())
{
}

void AlgaeSubsystem::Periodic() {
    GoToAngle();
    frc::SmartDashboard::PutNumber("Algae Angle", CurrentAngle().value());

    if (m_algaeDataProvider->IsAlgaeAngleValid()) {
        // Only set the encoder offset the first time a value is valid. This needs to be done
        // in Periodic since the angle is not unpacked until the FeatherCanDecoder periodic
        // function runs.
        if (0.0 == m_relEncoderOffsetDegrees) {
            m_relEncoderOffsetDegrees = m_algaeDataProvider->GetAlgaeRawAngleDegrees();
        }

        double pid_calculation = m_algaePID.Calculate(m_algaeDataProvider->GetAlgaeAngleDegrees(), m_setpointAngle);
        frc::SmartDashboard::PutNumber("Algae PID", pid_calculation);
        SetPivotSpeed(pid_calculation);
    } else {
        double fallback_pid_calculation = m_algaePID.Calculate(GetAngleDegreesFallback().value(), m_setpointAngle);
        frc::SmartDashboard::PutNumber("Algae Neo 550 PID", fallback_pid_calculation);
        // @todo Enable this after validating that the calculated angles are correct
        // SetPivotSpeed(fallback_pid_calculation);
    }
}

void AlgaeSubsystem::SetPivotSpeed(double speed) {
    frc::SmartDashboard::PutNumber("Algae Pivot speed", speed);
    m_algaePivotMotor.Set(-speed);
}

units::degree_t AlgaeSubsystem::GetAngleDegreesFallback() {
    double currentPosition = m_algaePivotEncoder.GetPosition();
    double scaledPosition = currentPosition / kAlgaePivotGearRatio;
    units::degree_t angle = units::radian_t((scaledPosition / kNeoCountsPerRev) * 2 * M_PI);

    return angle - units::degree_t(m_relEncoderOffsetDegrees);
}

frc2::CommandPtr AlgaeSubsystem::Intake() {
    return frc2::cmd::StartEnd(
        [this] {
            m_algaeMotor.Set(0.4);
        },
        [this] {
            m_algaeMotor.Set(0.0);
        }
    ).Until(
        [this] {
            return m_algaeDataProvider->IsAlgaeCollected();
        }
    ).WithTimeout(3_s);

}

frc2::CommandPtr AlgaeSubsystem::Dispense() {
    return frc2::cmd::StartEnd(
        [this] {
            m_algaeMotor.Set(-0.4);
        },
        [this] {
            m_algaeMotor.Set(0.0);
        }
    ).WithTimeout(5_s);
}

frc2::CommandPtr AlgaeSubsystem::SetGoalAngle(units::degree_t angle) {
    return frc2::cmd::RunOnce([this, angle] {
        m_setpointAngle = angle;
    });
}

units::degree_t AlgaeSubsystem::CurrentAngle() {
    return units::degree_t(m_algaeDataProvider->GetAlgaeAngleDegrees());
};

void AlgaeSubsystem::GoToAngle() {
    double value = m_algaePID.Calculate(CurrentAngle(), m_setpointAngle);
    frc::SmartDashboard::PutNumber("Algae PID", value);

    units::volt_t goalVolts = units::volt_t(value) + m_algaeFeedForward.Calculate(units::radian_t(m_algaePID.GetSetpoint().position), m_algaePID.GetSetpoint().velocity, 0_rad_per_s_sq);
    frc::SmartDashboard::PutNumber("Algae PID with FeedForward", goalVolts.value());

    double current_difference = fabs(m_setpointAngle.value() - CurrentAngle().value());
    m_algaePivotMotor.SetVoltage(goalVolts);

    frc::SmartDashboard::PutNumber("Algae diff", current_difference);
}

bool AlgaeSubsystem::IsAlgaeStored() {
    return m_algaeDataProvider->IsAlgaeCollected();
}