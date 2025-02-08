
#include <subsystems/AlgaeSubsystem.h>
#include <Constants.h>

AlgaeSubsystem::AlgaeSubsystem():
m_algaeMotor{AlgaeConstants::CORAL_MOTOR_PORT, rev::spark::SparkLowLevel::MotorType::kBrushed},
m_coralMotor{AlgaeConstants::PIVOT_MOTOR_PORT, rev::spark::SparkLowLevel::MotorType::kBrushed}
{
  
}

void AlgaeSubsystem::SetSpeed(double speed) {

}

double AlgaeSubsystem::GetAngle() {

}

void AlgaeSubsystem::GoToAngle(double angle) {

}