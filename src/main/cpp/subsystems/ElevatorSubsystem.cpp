#include "subsystems/ElevatorSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"

ElevatorSubsystem::ElevatorSubsystem():
m_elevatorMotor1(kElevatorMotor1Id),
// m_elevatorMotor2(kElevatorMotor2Id),
m_elevatorLimitSwitch(kLimitSwitchId)
{
    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    slot0Configs.kV = .12;
    slot0Configs.kP = 0.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    m_elevatorMotor1.GetConfigurator().Apply(slot0Configs);
    // slot0Configs.kV = 8_V;
    // slot0Configs.Voltage.PeakReverseVoltage = -8_V;

//   configs.TorqueCurrent.PeakForwardTorqueCurrent = 120_A;
//   configs.TorqueCurrent.PeakReverseTorqueCurrent = -120_A;



    // m_elevatorMotor2.GetConfigurator().Apply(slot0Configs);
ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_elevatorMotor1.GetConfigurator().Apply(slot0Configs);
    if (status.IsOK()) break;
    frc::SmartDashboard::PutString("Error", "nuh uh");
  }
  if (!status.IsOK()) {
    frc::SmartDashboard::PutString("Error", "Could not apply configs, error code:");
    std::cout << "Could not apply configs, error code: " << status.GetName() << std::endl;
  }

};