#include "subsystems/ElevatorSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/controls/Follower.hpp>

ElevatorSubsystem::ElevatorSubsystem():
m_elevatorMotor1(kElevatorMotor1Id),
m_elevatorMotor2(kElevatorMotor2Id),
m_elevatorLimitSwitch(kLimitSwitchId)
{
    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    // slot0Configs.kV = .12;
    slot0Configs.kP = 7.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    m_elevatorMotor1.GetConfigurator().Apply(slot0Configs);
    m_elevatorMotor2.GetConfigurator().Apply(slot0Configs);

    ctre::phoenix6::configs::MotorOutputConfigs motorConfigs;
    motorConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        // Limit the forward and reverse duty cycle while getting the elevator working.
        .WithPeakForwardDutyCycle(kSlowElevator)
        .WithPeakReverseDutyCycle(kSlowElevator);

    m_elevatorMotor1.GetConfigurator().Apply(motorConfigs);
    m_elevatorMotor2.GetConfigurator().Apply(motorConfigs);

    ctre::phoenix6::controls::Follower follower(kElevatorMotor1Id, true);
    m_elevatorMotor2.SetControl(follower);

    m_elevatorMotor1.SetPosition(0_tr);
    /*
     * This method blocks the current robot loop until the signal is retrieved or the timeout is activated.
     * The CTRE docs state that this API can ensure that set operations are completed before continuing control flow.
     * This method reports an error to the DriverStation.
     * The link: https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html
     */ 
    m_elevatorMotor1.GetPosition().WaitForUpdate(20_ms);
}

void ElevatorSubsystem::Periodic() {
    PlotElevatorPosition();

    frc::SmartDashboard::PutBoolean("Elevator Limit Switch", !m_elevatorLimitSwitch.Get());
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

units::turn_t ElevatorSubsystem::HeightToTurns(units::inch_t height) {
    return units::turn_t(
        -(height.value() * GEAR_RATIO) / (2 * M_PI * WHEEL_RADIUS.value())
    );
}

void ElevatorSubsystem::PrepareElevator(units::inch_t newPosition) {
    m_desiredElevatorPosition = newPosition;
}

frc2::CommandPtr ElevatorSubsystem::GoToSavedPosition() {
    return GoToCoralLevel(m_desiredElevatorPosition);
}

frc2::CommandPtr ElevatorSubsystem::GoToCoralLevel(units::inch_t position) {
    return frc2::cmd::RunOnce([this, position] {
        units::turn_t desiredTurns = HeightToTurns(position);
        frc::SmartDashboard::PutNumber("Desired Turns", desiredTurns.value());
        m_elevatorMotor1.SetControl(
            m_positionVoltage.WithPosition(desiredTurns)
        );
    });
}

frc2::CommandPtr ElevatorSubsystem::Lower() {
    return frc2::cmd::RunOnce([this] {
        // Test command to slowly lower the elevator
        m_elevatorMotor1.SetControl(ctre::phoenix6::controls::DutyCycleOut(kSlowElevator));
    });
}

frc2::CommandPtr ElevatorSubsystem::Raise() {
    return frc2::cmd::RunOnce([this] {
        // Test command to slowly raise the elevator
        m_elevatorMotor1.SetControl(ctre::phoenix6::controls::DutyCycleOut(-kSlowElevator));
    });
}

frc2::CommandPtr ElevatorSubsystem::Stop() {
    return frc2::cmd::RunOnce([this] {
        m_elevatorMotor1.StopMotor();
    });
}

units::inch_t ElevatorSubsystem::CurrentHeight() {
    return units::inch_t(
        -(m_elevatorMotor1.GetPosition().GetValueAsDouble() * 2 * M_PI * WHEEL_RADIUS) / GEAR_RATIO
    );
}

bool ElevatorSubsystem::IsMagneticLimitSwitchActive() {
    // The REV magnetic limit switch is Active-low so a false from the Get() call means the elevator is at the bottom
    return !m_elevatorLimitSwitch.Get();
}