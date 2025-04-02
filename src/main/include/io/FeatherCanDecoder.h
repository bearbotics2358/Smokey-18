/**
 * @file FeatherCanDecoder.h
 */

#pragma once

#include "subsystems/ICoralIntakeDataProvider.h"
#include "subsystems/IAlgaeDataProvider.h"
#include "subsystems/IBellyPanDataProvider.h"
#include "subsystems/IClimberDataProvider.h"

#include <frc/CAN.h>

/**
 * @brief This class is used for parsing sensor values.
 * @details This class parses CAN messages to extract sensor values (proximity sensors, etc.).
 * The CAN messages are created through custom hardware that integrates multiple sensors 
 * and releases their values as CAN messages. A timeout of 100 milliseconds exist,
 * which is used for the Is...Valid() functions.
 */
class FeatherCanDecoder:
public ICoralIntakeDataProvider,
public IClimberDataProvider,
public IBellyPanDataProvider,
public IAlgaeDataProvider
{
public:
    FeatherCanDecoder();

    /**
     * @brief This function parses all the CAN messages; this function must be called periodically.
     */
    void Update();

    // **** ICoralIntakeDataProvider interface functions **** //

    /**
     * @brief Gets the transformed **absolute** angle of the coral arm.
     * @details An offset is applied to "transform" the range of the coral arm.
     */
    float GetCoralIntakeAngleDegrees() override;

    /**
     * @brief Gets the raw **absolute** angle of the coral arm.
     */
    float GetCoralIntakeRawAngleDegrees() override;

    bool IsCoralAngleValid() override;
    
    /**
     * @details Checks whether the value of the proximity sensor is over a tuned threshold.
     */
    bool IsCoralCollected() override;

    // **** IAlgaeDataProvider interface functions **** //

    /**
     * @brief Gets the transformed **absolute** angle of the algae arm.
     * @details An offset is applied to "transform" the range of the algae arm.
     */
    float GetAlgaeAngleDegrees() override;

    /**
     * @brief Gets the raw **absolute** angle of the algae arm.
     */
    float GetAlgaeRawAngleDegrees() override;

    /**
     * @details Checks whether the value of the proximity sensor is over a tuned threshold.
     */
    bool IsAlgaeCollected() override;

    // **** IClimberDataProvider interface functions **** //

    /**
     * @brief Gets the transformed **absolute** angle of the climber mechanism.
     * @details An offset is applied to "transform" the range of the climber mechanism.
     */
    float GetClimberAngleDegrees() override;

    /**
     * @brief Gets the raw **absolute** angle of the climber mechanism.
     */
    float GetClimberRawAngleDegrees() override;

    /**
     * @details Checks whether the value of the proximity sensor is over a tuned threshold
     * for the left side of the cage.
     */
    bool IsLeftCageHooked() override;

    /**
     * @details Checks whether the value of the proximity sensor is over a tuned threshold
     * for the right side of the cage.
     */
    bool IsRightCageHooked() override;

    // **** IBellyPanDataProvider interface functions **** //

    /**
     * @todo Use this function.
     */
    double GetBellyPanLeftDistance() override;

    /**
     * @todo Use this function.
     */
    double GetBellyPanRightDistance() override;

private:
    const int kCANReadTimeoutMs = 100;

    bool IsTOFDataValid(int proximityReading);

    void UnpackCoralCANData();

    const int kCoralDeviceID = 1;
    const int kCoralAPIId = 1;
    const int kCoralProximityThreshold = 1500;
    const double kCoralAngleOffsetDegrees = -258;

    float m_coralIntakeAngleDegrees;
    bool m_coralAngleValid;
    bool m_coralCollected;
    frc::CAN m_coralCAN;

    void UnpackAlgaeCANData();

    const int kAlgaeDeviceID = 2;
    const int kAlgaeAPIId = 2;
    const int kAlgaeProximityThreshold = 1500;
    const double kAlgaeAngleOffsetDegrees = -237.6;

    float m_algaeAngleDegrees;
    bool m_algaeAngleValid;
    bool m_algaeCollected;
    frc::CAN m_algaeCAN;

    void UnpackClimberCANData();

    const int kClimberDeviceID = 3;
    const int kClimberAPIId = 3;
    const double kClimberAngleOffsetDegrees = -368.5;
    const int kClimberProximityThreshold = 1500;

    float m_climberAngleDegrees;
    bool m_rightProximity;
    bool m_leftProximity;
    frc::CAN m_climberCAN;

    void UnpackBellyPanCANData();
    const int kBellyPanDeviceID = 4;

    double m_rightBellyPanDistance;
    double m_leftBellyPanDistance;
    frc::CAN m_bellyPanCAN;
};
