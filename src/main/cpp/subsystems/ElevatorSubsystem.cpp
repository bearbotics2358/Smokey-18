#include "subsystems/ElevatorSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/controls/Follower.hpp>

ElevatorSubsystem::ElevatorSubsystem():
m_elevatorMotor1(kElevatorMotor1Id),
m_elevatorMotor2(kElevatorMotor2Id),
m_elevatorLimitSwitch(kLimitSwitchId)
{
    ctre::phoenix6::configs::MotorOutputConfigs motorConfigs;
    motorConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        .WithInverted(true);

    m_elevatorMotor1.GetConfigurator().Apply(motorConfigs);

    // Motor 2 has the same configuration as Motor 1 except that it runs in the opposite direction
    motorConfigs.WithInverted(false);
    m_elevatorMotor2.GetConfigurator().Apply(motorConfigs);

    m_elevatorMotor1.SetPosition(0_tr);
    m_elevatorMotor2.SetPosition(0_tr);
    /*
     * This method blocks the current robot loop until the signal is retrieved or the timeout is activated.
     * The CTRE docs state that this API can ensure that set operations are completed before continuing control flow.
     * This method reports an error to the DriverStation.
     * The link: https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html
     */
    m_elevatorMotor1.GetPosition().WaitForUpdate(20_ms);
    m_elevatorMotor2.GetPosition().WaitForUpdate(20_ms);

    IsMagneticLimitSwitchActive.OnTrue(frc2::cmd::RunOnce([this] {
        frc::SmartDashboard::PutBoolean("Elevator Limit Switch", true);
        // Avoid calling SetPosition from the Periodic function because the call can take more than 20 milliseconds
        m_elevatorMotor1.SetPosition(0_tr, 13_ms);
    })).OnFalse(frc2::cmd::RunOnce([this] {
        frc::SmartDashboard::PutBoolean("Elevator Limit Switch", false);
    }));
};

void ElevatorSubsystem::Periodic() {
    PlotElevatorPosition();

    frc::SmartDashboard::PutNumber("Elevator Set Point", m_setpointHeight.value());

    SetMotorVoltage();
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
    frc::SmartDashboard::PutNumber("Elevator Motor Position", position.GetValueAsDouble());

    frc::SmartDashboard::PutNumber("Elevator Height", CurrentHeight().value());
};

units::inch_t ElevatorSubsystem::CurrentHeight() {
    return units::inch_t(
        (m_elevatorMotor1.GetPosition().GetValueAsDouble() * 2 * M_PI * WHEEL_RADIUS) / GEAR_RATIO
    );
}

void ElevatorSubsystem::SetMotorVoltage() {
    double value = m_elevatorPID.Calculate(CurrentHeight(), m_setpointHeight);
    frc::SmartDashboard::PutNumber("Elevator PID", value);

    units::volt_t goalVolts = units::volt_t(value) + m_feedforward.Calculate(m_elevatorPID.GetSetpoint().velocity);
    frc::SmartDashboard::PutNumber("Elevator PID with feedforward", goalVolts.value());

    double current_difference = fabs(m_setpointHeight.value() - CurrentHeight().value());
    if (current_difference >= TOLERANCE) {
        m_elevatorMotor1.SetVoltage(goalVolts);
        m_elevatorMotor2.SetVoltage(goalVolts);
        elevatorAtHeight = false;
    } else if (m_setpointHeight <= 0.2_in) {
        m_elevatorMotor1.SetVoltage(0_V);
        m_elevatorMotor2.SetVoltage(0_V);
        elevatorAtHeight = false;
    } else {
        m_elevatorMotor1.SetVoltage(kG);
        m_elevatorMotor2.SetVoltage(kG);
        elevatorAtHeight = true;
    }
}

frc2::CommandPtr ElevatorSubsystem::GoToHeight(units::inch_t height) {
    return frc2::cmd::RunOnce([this, height] {
        m_setpointHeight = height;
    });
}

bool ElevatorSubsystem::GetElevatorHeightAboveThreshold() {
    return CurrentHeight() >= kHeightThreshold;
}

frc2::CommandPtr ElevatorSubsystem::WaitUntilElevatorAtHeight() {
    return frc2::cmd::WaitUntil([this] { return elevatorAtHeight; });
}