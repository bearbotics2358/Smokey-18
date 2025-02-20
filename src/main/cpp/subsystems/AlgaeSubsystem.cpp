
#include <subsystems/AlgaeSubsystem.h>
#include <rev/config/SparkBaseConfig.h>

AlgaeSubsystem::AlgaeSubsystem():
m_algaeLeftMotor{MOTOR_LEFT_PORT, rev::spark::SparkLowLevel::MotorType::kBrushless},
m_algaeRightMotor{MOTOR_RIGHT_PORT, rev::spark::SparkLowLevel::MotorType::kBrushless}
{
    rev::spark::SparkBaseConfig followerConfig;
    followerConfig.Follow(MOTOR_RIGHT_PORT, true);

    // TODO: Configure these variables later.
    rev::spark::SparkBase::ResetMode resetMode;
    rev::spark::SparkBase::PersistMode persistMode;

    m_algaeLeftMotor.Configure(followerConfig, resetMode, persistMode);
}

void AlgaeSubsystem::SetSpeed(double speed) {

}

void AlgaeSubsystem::GoToAngle(double angle) {

}