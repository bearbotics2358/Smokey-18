#pragma once

#include "subsystems/ICoralIntakeDataProvider.h"

class FeatherCanDecoder: public ICoralIntakeDataProvider {
public:
    FeatherCanDecoder();

    void Update();

    // **** ICoralIntakeDataProvider interface functions **** //
    float GetCoralIntakeAngleDegrees() override;
    bool IsCoralCollected() override;

private:
    float m_coralIntakeAngleDegrees;
    bool m_coralCollected;
};
