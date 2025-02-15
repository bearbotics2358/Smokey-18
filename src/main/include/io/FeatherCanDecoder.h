#pragma once

#include "subsystems/ICoralIntakeDataProvider.h"

#include <frc/CAN.h>


class FeatherCanDecoder: public ICoralIntakeDataProvider {
public:
    FeatherCanDecoder();

    void Update();

    // **** ICoralIntakeDataProvider interface functions **** //
    float GetCoralIntakeAngleDegrees() override;
    bool IsCoralCollected() override;
    const int kCoralCanID = 1; // @todo this is temporary, change to actual ID
    const int kCoralAPIId = 4;

private:
    float m_coralIntakeAngleDegrees;
    bool m_coralCollected;
    frc::CAN m_coralCAN;

    void UnpackCoralCANData();
};
