#pragma once

#include "subsystems/ICoralIntakeDataProvider.h"
#include "subsystems/IAlgaeDataProvider.h"

#include <frc/CAN.h>


class FeatherCanDecoder: public ICoralIntakeDataProvider, public IAlgaeDataProvider {
public:
    const int kCoralDeviceID = 1;
    const int kCoralAPIId = 1;
    const int kCoralProximityThreshold = 1500;
    const double kCoralAngleOffsetDegrees = -262.6;


    // All algae values are copied over from the coral values: modify algae values accordingly
    const int kAlgaeDeviceID = 1;
    const int kAlgaeAPIId = 1;
    const int kAlgaeProximityThreshold = 1500;
    const double kAlgaeAngleOffsetDegrees = -262.6;

    FeatherCanDecoder();

    void Update();

    // **** ICoralIntakeDataProvider interface functions **** //
    float GetCoralIntakeAngleDegrees() override;
    float GetCoralIntakeRawAngleDegrees() override;
    bool IsCoralCollected() override;

    // **** IAlgaeDataProvider interface functions **** //
    float GetAlgaeAngleDegrees() override;
    float GetAlgaeRawAngleDegrees() override;
    bool IsAlgaeCollected() override;

private:
    float m_coralIntakeAngleDegrees;
    bool m_coralCollected;
    frc::CAN m_coralCAN;

    void UnpackCoralCANData();

    float m_algaeAngleDegrees;
    bool m_algaeCollected;
    frc::CAN m_algaeCAN;

    void UnpackAlgaeCANData();
};
