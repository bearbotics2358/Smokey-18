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
    /*
     * This method blocks the current robot loop until the signal is retrieved or the timeout is activated.
     * The CTRE docs state that this API can ensure that set operations are completed before continuing control flow.
     * This method reports an error to the DriverStation.
     * The link: https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html
     */ 
    m_elevatorMotor1.GetPosition().WaitForUpdate(20_ms);
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