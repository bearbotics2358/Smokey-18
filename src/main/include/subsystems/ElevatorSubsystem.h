/**
 * @file ElevatorSubsystem.h
 */

#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/DigitalInput.h>
#include <frc/Encoder.h>

#include <units/length.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc2/command/button/Trigger.h>

/**
 * @brief The CAN ID of one of the elevator motors.
 */
constexpr int kElevatorMotor1Id = 36;

/**
 * @brief The CAN ID of the other the elevator motors.
 */
constexpr int kElevatorMotor2Id = 37;

/**
 * @brief The CAN ID of the elevator limit switch.
 */
constexpr int kLimitSwitchId = 0;

constexpr units::inch_t kElevatorCollectPosition = 0_in;
constexpr units::inch_t kElevatorStowPosition = 0_in;
constexpr units::inch_t kElevatorProcessorPosition = 10_in;
constexpr units::inch_t kElevatorL1Position = 0_in;
constexpr units::inch_t kElevatorL2Position = 13_in;
constexpr units::inch_t kElevatorL3Position = 28_in;
/**
 * @brief The elevator height in inches for removing algae in between the 
 * L3 and L4 reef bars.
 */
constexpr units::inch_t kElevatorAlgaeOnlyL3Position = 44_in;
constexpr units::inch_t kElevatorL4Position = 61.5_in;

/**
 * @brief Slows the speed of the elevator down. 
 * @attention This variable should only be used for testing purposes.
 */
constexpr float kSlowElevator = 0.6;

/**
 * @brief The robot will automatically drive slower if the the elevator is above this height.
 * @attention This threshold is _only_ applied during teleop.
 */
constexpr units::inch_t kHeightThreshold = 20_in;

/**
 * @brief Represents the subsystem that controls the elevator.
 */
class ElevatorSubsystem : public frc2::SubsystemBase {
    public:
        ElevatorSubsystem();

        void Periodic() override;

        /**
         * @brief Prints out the elevator height to plot in Elastic Dashboard.
         * @details To see the variable `position` as a graph (located inside the function code), 
         * first deploy the robot code to a robot. Then, Click on the WPILIB icon in the top right 
         * corner of your screen. Search `WPILIB: Start Tool` and click `Elastic`.
         * Click `Add Widget` at the top of the screen and look for `Smart Dashboard` to find `Elevator Motor Position`.
         * Alternatively, use the search widget to find `Elevator Motor Position`.
         * Drag the Elevator widget onto the screen into a empty position. You should see a green highlight.
         * Right click on the widget -> `Show As` -> `Graph`. To resize the graph, drag on the outlines of the widget.
         */
        void PlotElevatorPosition();

        units::inch_t CurrentHeight();

        /**
         * @brief This function was intended to save the height the elevator should go to.
         * @deprecated This function is unused and will be removed.
         */
        void PrepareElevator(units::inch_t newPosition);
        frc2::CommandPtr GoToHeight(units::inch_t height);
        
        /**
         * @brief The radius of the spools that spin the wires that help move the elevator.
         * @note Because the spools are curved, this variable represents the approximate
         * average radius of the spools.
         */
        const units::inch_t WHEEL_RADIUS = 1.325_in;
        const double GEAR_RATIO = 9.0;

        /**
         * @brief Sets the voltage of the elevator motors, which is calculated based off of PID.
         * @attention This function is not meant to be called directly and must be called
         * periodically.
         */
        void SetMotorVoltage();

        /**
         * @brief Triggers when the elevator is above the @ref kHeightThreshold.
         */
        frc2::Trigger IsHeightAboveThreshold = frc2::Trigger([this] {
            return GetElevatorHeightAboveThreshold();
        });

        /**
         * @brief This `frc2::CommandPtr` is meant to be used in command compositions that 
         * require the elevator to be at a certain height before executing actions.
         * @details This `frc2::CommandPtr` will wait until the robot can safely move
         * while the elevator is moving.
         */
        frc2::CommandPtr WaitUntilElevatorIsCloseEnoughToMove();
    private:

        bool GetElevatorHeightAboveThreshold();

        bool m_closeEnoughToMove = false;

        ctre::phoenix6::hardware::TalonFX m_elevatorMotor1;
        ctre::phoenix6::hardware::TalonFX m_elevatorMotor2;
        frc::DigitalInput m_elevatorLimitSwitch{kLimitSwitchId};

        frc2::Trigger IsMagneticLimitSwitchActive = frc2::Trigger([this] {
            // The REV magnetic limit switch is Active-low so a false from the Get() call means the
            // elevator is at the bottom
            return !m_elevatorLimitSwitch.Get();
        });

        static constexpr units::inch_t kSetpointTolerance = 0.5_in;
        static constexpr units::inch_t kCloseEnoughToMove = 4_in;

        static constexpr units::meters_per_second_t kMaxVelocity = 5.0_mps;
        static constexpr units::meters_per_second_squared_t kMaxAcceleration = 8.0_mps_sq;
        static constexpr double kP = 20.0;
        static constexpr double kI = 0.5;
        static constexpr double kD = 2.0;
        static constexpr units::volt_t kS = 0.325_V;
        static constexpr units::volt_t kG = 0.35_V;
        static constexpr auto kV = 0.0_V / 1.0_mps;

        frc::TrapezoidProfile<units::meters>::Constraints m_constraints {
            kMaxVelocity, kMaxAcceleration};

        frc::ProfiledPIDController<units::meters> m_elevatorPID{
            kP, kI, kD, m_constraints
        };

        frc::ElevatorFeedforward m_feedforward{kS, kG, kV};

        // Changing m_setpointHeight will send the elevator to that position immediately
        units::inch_t m_setpointHeight = 0_in;

};