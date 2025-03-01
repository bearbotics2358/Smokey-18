#pragma once

#include "subsystems/ICoralIntakeDataProvider.h"

#include <frc/CAN.h>


class FeatherCanDecoder: public ICoralIntakeDataProvider {
public:
    const int kCoralDeviceID = 1;
    const int kCoralAPIId = 1;
    const int kCoralProximityThreshold = 1500;
    const double kCoralAngleOffsetDegrees = -262.6;

    FeatherCanDecoder();

    void Update();

    // **** ICoralIntakeDataProvider interface functions **** //
    float GetCoralIntakeAngleDegrees() override;
    float GetCoralIntakeRawAngleDegrees() override;
    float GetCoralIntakeRawAngleDegrees() override;
    bool IsCoralCollected() override;

private:
    float m_coralIntakeAngleDegrees;
    bool m_coralCollected;
    frc::CAN m_coralCAN;

    void UnpackCoralCANData();
};
