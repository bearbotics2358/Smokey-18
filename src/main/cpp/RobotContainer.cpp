// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <subsystems/LED.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

using namespace subsystems;

RobotContainer::RobotContainer(FeatherCanDecoder* featherCanDecoder):
m_featherCanDecoder(featherCanDecoder),
m_cameraSubsystem(&m_drivetrain),
m_coralSubsystem(m_featherCanDecoder),
m_algaeSubsystem(m_featherCanDecoder),
m_climberSubsystem(m_featherCanDecoder),
m_scoringSuperstructure(m_elevatorSubsystem, m_coralSubsystem, m_algaeSubsystem, m_drivetrain)
{
    m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::MSG_IDLE);

    m_drivetrain.ConfigNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    AddPathPlannerCommands();

    m_drivetrain.ConfigureAutoBuilder();

    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &m_autoChooser);

    ConfigureBindings();
}

frc2::CommandPtr RobotContainer::AddControllerRumble(frc::GenericHID::RumbleType rumbleType, double rumble) {
    return frc2::cmd::RunOnce([this, rumble, rumbleType] {
        m_driverJoystick.SetRumble(rumbleType, rumble);
    });
}

void RobotContainer::ConfigureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    // **** Xbox Drivetrain Buttons **** //
    m_drivetrain.SetDefaultCommand(m_drivetrain.ApplyRequest([this]() -> auto&& {
        // Drivetrain will execute this command periodically
        return drive.WithVelocityX(
                -m_driverJoystick.GetLeftY() * m_maxSpeed * m_speedMultiplier
            ) // Drive forward with negative Y (forward)
            .WithVelocityY(
                -m_driverJoystick.GetLeftX() * m_maxSpeed * m_speedMultiplier
            ) // Drive left with negative X (left)
            .WithRotationalRate(
                -m_driverJoystick.GetRightX() * m_maxAngularRate * m_speedMultiplier
            ); // Drive counterclockwise with negative X (left)
    }));

    m_drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

    //   _____       _                   _____            _             _ _             ____        _   _                  
    //  |  __ \     (_)                 / ____|          | |           | | |           |  _ \      | | | |                 
    //  | |  | |_ __ ___   _____ _ __  | |     ___  _ __ | |_ _ __ ___ | | | ___ _ __  | |_) |_   _| |_| |_ ___  _ __  ___ 
    //  | |  | | '__| \ \ / / _ \ '__| | |    / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__| |  _ <| | | | __| __/ _ \| '_ \/ __|
    //  | |__| | |  | |\ V /  __/ |    | |___| (_) | | | | |_| | | (_) | | |  __/ |    | |_) | |_| | |_| || (_) | | | \__ \
    //  |_____/|_|  |_| \_/ \___|_|     \_____\___/|_| |_|\__|_|  \___/|_|_|\___|_|    |____/ \__,_|\__|\__\___/|_| |_|___/

    // **** Xbox A, B, X, & Y Button **** //
    m_driverJoystick.B().WhileTrue(
        frc2::cmd::Either(
            // This checks the state of the L/R reef switch to determine which reef pole to align with
            AlignWithReef(&m_cameraSubsystem, &m_drivetrain, ReefSide::Right)
                .ToPtr()
                .AndThen(AddControllerRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0)),
            AlignWithReef(&m_cameraSubsystem, &m_drivetrain, ReefSide::Left)
                .ToPtr()
                .AndThen(AddControllerRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0)),
            [this] { return m_reefSide == ReefSide::Right; }
        )
    ).ToggleOnFalse(
        AddControllerRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0)
    );

    m_driverJoystick.X().WhileTrue(m_scoringSuperstructure.ScoreIntoProcessor());

    m_driverJoystick.A().OnTrue(m_coralSubsystem.Dispense());
    
    m_driverJoystick.Y().OnTrue(m_coralSubsystem.Collect());

    // **** Xbox Trigger & Bumper Buttons **** //
    m_driverJoystick.RightTrigger()
        .OnTrue(
            m_scoringSuperstructure.ScoreIntoReef().FinallyDo(
                [this] { m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::MSG_IDLE); }
            )
        )
        .OnFalse(
            m_scoringSuperstructure.CancelScore()
        );

    m_driverJoystick.LeftTrigger()
        .OnTrue(
            m_coralSubsystem.Collect()
        )
        .OnFalse(
            frc2::cmd::Parallel(
                m_coralSubsystem.StopIntake(),
                m_scoringSuperstructure.ToStowPosition()
            )
        );

    m_driverJoystick.RightBumper().WhileTrue(m_drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));

    // **** Xbox Dpad Buttons **** //
    m_driverJoystick.POVUp().OnTrue(m_climberSubsystem.Climb());

    m_driverJoystick.POVDown().OnTrue(frc2::cmd::RunOnce([this] { m_drivetrain.SeedFieldCentric(); }));

    m_driverJoystick.POVRight().WhileTrue(
        m_drivetrain.ApplyRequest([this]() -> auto&& {
            return strafe.WithVelocityY(-0.25_mps);
        })
    );

    m_driverJoystick.POVLeft().WhileTrue(
        m_drivetrain.ApplyRequest([this]() -> auto&& {
            return strafe.WithVelocityY(0.25_mps);
        })
    );

    //    ____                       _                _____            _             _ _             ____        _   _                  
    //   / __ \                     | |              / ____|          | |           | | |           |  _ \      | | | |                 
    //  | |  | |_ __   ___ _ __ __ _| |_ ___  _ __  | |     ___  _ __ | |_ _ __ ___ | | | ___ _ __  | |_) |_   _| |_| |_ ___  _ __  ___ 
    //  | |  | | '_ \ / _ \ '__/ _` | __/ _ \| '__| | |    / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__| |  _ <| | | | __| __/ _ \| '_ \/ __|
    //  | |__| | |_) |  __/ | | (_| | || (_) | |    | |___| (_) | | | | |_| | | (_) | | |  __/ |    | |_) | |_| | |_| || (_) | | | \__ \
    //   \____/| .__/ \___|_|  \__,_|\__\___/|_|     \_____\___/|_| |_|\__|_|  \___/|_|_|\___|_|    |____/ \__,_|\__|\__\___/|_| |_|___/
    //         | |                                                                                                                      
    //         |_|

    // **** Xbox A, B, X, & Y Button **** //
    m_operatorJoystick.Y().OnTrue(
        frc2::cmd::Parallel(
            m_scoringSuperstructure.PrepareScoring(ScoringSuperstructure::ScoringSelector::L4),
            frc2::cmd::RunOnce([this] {
                m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::IDK);
            })
        ));

    m_operatorJoystick.X().OnTrue(
        frc2::cmd::Parallel(
            m_scoringSuperstructure.PrepareScoring(ScoringSuperstructure::ScoringSelector::L3AlgaeAndCoral),
            frc2::cmd::RunOnce([this] {
                m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L3);
            })
        ));
                                                                                                                                                                                 
    m_operatorJoystick.POVUp().OnTrue(frc2::cmd::Parallel(
        frc2::cmd::RunOnce([this] {
            m_drivetrain.ConfigNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
        }),
        m_climberSubsystem.Extend()
    ));

    m_operatorJoystick.B().OnTrue(frc2::cmd::Parallel(
        m_scoringSuperstructure.PrepareScoring(ScoringSuperstructure::ScoringSelector::L2),
        frc2::cmd::RunOnce([this] {
            m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L2);
        })
    ));

    m_operatorJoystick.A().OnTrue(frc2::cmd::Parallel(
        m_scoringSuperstructure.ToCollectPosition(),
        frc2::cmd::RunOnce([this] {
            m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L1);
        })
    ));

    // **** Xbox Trigger & Bumper Buttons **** //
    m_operatorJoystick.LeftBumper().OnTrue(frc2::cmd::Parallel(
        m_scoringSuperstructure.ToStowPosition(),
        frc2::cmd::RunOnce([this] {
            m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::MSG_IDLE);
        })
    ));

    // **** Xbox Dpad Buttons **** //
    m_operatorJoystick.POVDown().OnTrue(frc2::cmd::Parallel(
        frc2::cmd::RunOnce([this] {
            m_drivetrain.ConfigNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
        }),
        m_climberSubsystem.Stow()
    ));

    m_operatorJoystick.POVRight().OnTrue(frc2::cmd::RunOnce(
        [this] {
            m_reefSide = ReefSide::Right;
        })
    );

    m_operatorJoystick.POVLeft().OnTrue(frc2::cmd::RunOnce(
        [this] {
            m_reefSide = ReefSide::Left;
        })
    );

    // m_gamepad.Button(9).OnTrue(frc2::cmd::Parallel(
    //     m_scoringSuperstructure.PrepareScoring(ScoringSuperstructure::ScoringSelector::L3AlgaeOnly),
    //     frc2::cmd::RunOnce([this] {
    //         m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L3_ALGAE);
    //     })
    // ));

    // m_gamepad.Button(1).OnTrue(frc2::cmd::Parallel(
    //     m_scoringSuperstructure.PrepareScoring(ScoringSuperstructure::ScoringSelector::L1),
    //     frc2::cmd::RunOnce([this] {
    //         m_LED.SetLEDState(ArduinoConstants::RIO_MESSAGES::ELEVATOR_L1);
    //     })
    // ));

    //    _____          _                    _______   _                         ____  _           _ _                 
    //   / ____|        | |                  |__   __| (_)                       |  _ \(_)         | (_)                
    //  | |    _   _ ___| |_ ___  _ __ ___      | |_ __ _  __ _  __ _  ___ _ __  | |_) |_ _ __   __| |_ _ __   __ _ ___ 
    //  | |   | | | / __| __/ _ \| '_ ` _ \     | | '__| |/ _` |/ _` |/ _ \ '__| |  _ <| | '_ \ / _` | | '_ \ / _` / __|
    //  | |___| |_| \__ \ || (_) | | | | | |    | | |  | | (_| | (_| |  __/ |    | |_) | | | | | (_| | | | | | (_| \__ \
    //   \_____\__,_|___/\__\___/|_| |_| |_|    |_|_|  |_|\__, |\__, |\___|_|    |____/|_|_| |_|\__,_|_|_| |_|\__, |___/
    //                                                     __/ | __/ |                                         __/ |    
    //                                                    |___/ |___/                                         |___/     
    
    (m_elevatorSubsystem.IsHeightAboveThreshold || m_driverJoystick.LeftBumper())
        .OnTrue(
            frc2::cmd::RunOnce([this] {m_speedMultiplier = 0.1;})
        )
        .OnFalse(
            frc2::cmd::RunOnce([this] {m_speedMultiplier = 1.0;})
        );

    (m_climberSubsystem.IsLeftCageHooked)
        .OnTrue(
            AddControllerRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0)
        )
        .OnFalse(
            AddControllerRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0)
        );

    (m_climberSubsystem.IsRightCageHooked)
        .OnTrue(
            AddControllerRumble(frc::GenericHID::RumbleType::kRightRumble, 1.0)
        )
        .OnFalse(
            AddControllerRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0)
        );                                                                                                               
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return m_autoChooser.GetSelected();
}


void RobotContainer::AddPathPlannerCommands() {
    using namespace pathplanner;
    NamedCommands::registerCommand(
        "Stow",
        std::move(m_scoringSuperstructure.ToStowPosition())
    );
    NamedCommands::registerCommand(
        "Collect",
        std::move(m_scoringSuperstructure.ToCollectPosition())
    );
    NamedCommands::registerCommand(
        "ScoreL1",
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L1))
    );
    NamedCommands::registerCommand(
        "ScoreL2",
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L2))
    );
    NamedCommands::registerCommand(
        "ScoreL3",
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L3AlgaeAndCoral))
    );
    NamedCommands::registerCommand(
        "ScoreL4",
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L4))
    );
    NamedCommands::registerCommand(
        "ScoreL3AndRemoveAlgae",
        std::move(m_scoringSuperstructure.PrepareAndScoreIntoReef(ScoringSuperstructure::ScoringSelector::L3AlgaeAndCoral))
    );
    NamedCommands::registerCommand(
        "ScoreAlgae",
        std::move(m_scoringSuperstructure.ScoreIntoProcessor())
    );
    NamedCommands::registerCommand(
        "AlignWithReefLeft",
        std::move(AlignWithReef(&m_cameraSubsystem, &m_drivetrain, ReefSide::Left).ToPtr())
    );
}
