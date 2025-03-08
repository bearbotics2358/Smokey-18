#pragma once

#include "subsystems/ICoralIntakeDataProvider.h"
#include "subsystems/IClimberDataProvider.h"

#include <frc/CAN.h>


class FeatherCanDecoder: public ICoralIntakeDataProvider, public IClimberDataProvider {
public:
    const int kCoralDeviceID = 1;
    const int kCoralAPIId = 1;
    const int kCoralProximityThreshold = 1500;
    const double kCoralAngleOffsetDegrees = -262.6;

    const int kClimberDeviceID = 3;
    const int kClimberAPIId = 3;
    // const int kClimberProximityThreshold = 1500;
    const double kClimberAngleOffsetDegrees = 0;
    const int kClimberProximityThreshold = 0;

    FeatherCanDecoder();

    void Update();

    // **** ICoralIntakeDataProvider interface functions **** //
    float GetCoralIntakeAngleDegrees() override;
    float GetCoralIntakeRawAngleDegrees() override;
    bool IsCoralCollected() override;

    float GetClimberAngleDegrees() override;
    float GetClimberRawAngleDegrees() override;
    bool IsCageHooked() override;

private:
    float m_coralIntakeAngleDegrees;
    bool m_coralCollected;
    frc::CAN m_coralCAN;

    void UnpackCoralCANData();

    float m_climberAngleDegrees;
    bool m_climberCollected;
    frc::CAN m_climberCAN;

    void UnpackClimberCANData();
};
