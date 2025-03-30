#pragma once

#include "subsystems/ICoralIntakeDataProvider.h"
#include "subsystems/IAlgaeDataProvider.h"
#include "subsystems/IBellyPanDataProvider.h"
#include "subsystems/IClimberDataProvider.h"

#include <frc/CAN.h>


class FeatherCanDecoder:
public ICoralIntakeDataProvider,
public IClimberDataProvider,
public IBellyPanDataProvider,
public IAlgaeDataProvider
{
public:
    const int kCANReadTimeoutMs = 100;

    const int kCoralDeviceID = 1;
    const int kCoralAPIId = 1;
    const int kCoralProximityThreshold = 1500;
    const double kCoralAngleOffsetDegrees = -258;

    const int kAlgaeDeviceID = 2;
    const int kAlgaeAPIId = 2;
    const int kAlgaeProximityThreshold = 1500;
    const double kAlgaeAngleOffsetDegrees = -237.6;

    const int kClimberDeviceID = 3;
    const int kClimberAPIId = 3;
    const double kClimberAngleOffsetDegrees = -368.5;
    // const double kClimberAngleOffsetDegrees = 0.0;
    const int kClimberProximityThreshold = 1500;

    const int kBellyPanDeviceID = 4;

    FeatherCanDecoder();

    void Update();

    // **** ICoralIntakeDataProvider interface functions **** //
    float GetCoralIntakeAngleDegrees() override;
    float GetCoralIntakeRawAngleDegrees() override;
    bool IsCoralAngleValid() override;
    bool IsCoralCollected() override;

    // **** IAlgaeDataProvider interface functions **** //
    float GetAlgaeAngleDegrees() override;
    float GetAlgaeRawAngleDegrees() override;
    bool IsAlgaeCollected() override;

    // **** IClimberDataProvider interface functions **** //
    float m_climberAngleDegrees;
    bool m_rightProximity;
    bool m_leftProximity;
    frc::CAN m_climberCAN;

    float GetClimberAngleDegrees() override;
    float GetClimberRawAngleDegrees() override;
    bool IsLeftCageHooked() override;
    bool IsRightCageHooked() override;

    void UnpackClimberCANData();

    // **** IBellyPanDataProvider interface functions **** //
    double m_rightBellyPanDistance;
    double m_leftBellyPanDistance;
    frc::CAN m_bellyPanCAN;

    double GetBellyPanLeftDistance() override;
    double GetBellyPanRightDistance() override;

    void UnpackBellyPanCANData();

private:
    bool IsTOFDataValid(int proximityReading);

    float m_coralIntakeAngleDegrees;
    bool m_coralAngleValid;
    bool m_coralCollected;
    frc::CAN m_coralCAN;

    void UnpackCoralCANData();

    float m_algaeAngleDegrees;
    bool m_algaeAngleValid;
    bool m_algaeCollected;
    frc::CAN m_algaeCAN;

    void UnpackAlgaeCANData();
};
