#include "subsystems/ElevatorSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ElevatorSubsystem::ElevatorSubsystem():
m_elevatorMotor1(kElevatorMotor1Id),
m_elevatorMotor2(kElevatorMotor2Id),
m_elevatorLimitSwitch(kLimitSwitchId)
{
    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    slot0Configs.kV = .12;
    slot0Configs.kP = 0.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    m_elevatorMotor1.GetConfigurator().Apply(slot0Configs);
    m_elevatorMotor2.GetConfigurator().Apply(slot0Configs);

    m_elevatorMotor1.SetPosition(0_tr);
};

void ElevatorSubsystem::Periodic() {
    PlotElevatorPosition();
}

/*
 * To see the variable `position` as a graph (located inside the function code), first deploy the robot code to a robot.
 * Then, Click on the WPILIB icon in the top right corner of your screen.
 * Search `WPILIB: Start Tool` and click `Elastic`.
 * Click `Add Widget` at the top of the screen and look for `Smart Dashboard` to find `Elevator Motor Position`.
 * Alternatively, use the search widget to find `Elevator Motor Position`.
 * Drag the Elevator widget onto the screen into a empty position. You should see a green highlight.
 * Right click on the widget -> `Show As` -> `Graph`.
 * To resize the graph, drag on the outlines of the widget. 
 */
void ElevatorSubsystem::PlotElevatorPosition() {
    ctre::phoenix6::StatusSignal<units::turn_t> position = m_elevatorMotor1.GetPosition();
    frc::SmartDashboard::PutNumber("Elevator Motor Position", position.GetValue().value());
}