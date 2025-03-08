#pragma once
#include "subsystems/IBellyPanDataProvider.h"
#include "subsystems/ICoralIntakeDataProvider.h"
#include "subsystems/IClimberDataProvider.h"

#include <frc/CAN.h>


class FeatherCanDecoder: public ICoralIntakeDataProvider, public IClimberDataProvider, public IBellyPanDataProvider {
public:
    const int kCoralDeviceID = 1;
    const int kCoralAPIId = 1;
    const int kCoralProximityThreshold = 1500;
    const double kCoralAngleOffsetDegrees = -262.6;

    const int kClimberDeviceID = 3;
    const int kClimberAPIId = 3;
    // const int kClimberProximityThreshold = 1500;
    const double kClimberAngleOffsetDegrees = 0;
    const int kClimberProximityThreshold = 0;//todo:add the threshold here


    const int kBellyPanDeviceID = 4;
    const int kBellyPanProximityThreshold = 0;//todo:add the threshold here

    FeatherCanDecoder();

    void Update();

    // **** ICoralIntakeDataProvider interface functions **** //
    float GetCoralIntakeAngleDegrees() override;
    float GetCoralIntakeRawAngleDegrees() override;
    bool IsCoralCollected() override;

    float GetClimberAngleDegrees() override;
    float GetClimberRawAngleDegrees() override;
    bool IsLeftCageHooked() override;
    bool IsRightCageHooked() override;

    bool IsLeftProximity() override;
    bool IsRightProximity() override;

private:
    float m_coralIntakeAngleDegrees;
    bool m_coralCollected;
    frc::CAN m_coralCAN;

    void UnpackCoralCANData();

    float m_climberAngleDegrees;
    bool m_rightProximity;
    bool m_leftProximity;
    frc::CAN m_climberCAN;

    void UnpackClimberCANData();

    bool m_rightBellyPanProximity;
    bool m_leftBellyPanProximity;
    frc::CAN m_bellyPanCAN;

    void UnpackBellyPanCANData();
};