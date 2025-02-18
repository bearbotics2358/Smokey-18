#include "io/FeatherCanDecoder.h"

#include "frc/smartdashboard/SmartDashboard.h"

FeatherCanDecoder::FeatherCanDecoder():m_coralCAN(kCoralDeviceID)
{
    m_coralIntakeAngleDegrees = 0.0;
    m_coralCollected = false;
}

void FeatherCanDecoder::Update() {
    UnpackCoralCANData();

    frc::SmartDashboard::PutNumber("Angles of Coral FeatherCan", m_coralIntakeAngleDegrees);
    frc::SmartDashboard::PutBoolean("Coral Collected?", m_coralCollected);
}

float FeatherCanDecoder::GetCoralIntakeAngleDegrees() {
    return m_coralIntakeAngleDegrees;
}

bool FeatherCanDecoder::IsCoralCollected() {
    return m_coralCollected;
}

void FeatherCanDecoder::UnpackCoralCANData() {
    frc::CANData data;

    bool isCoralDataValid = m_coralCAN.ReadPacketNew(kCoralAPIId, &data);

    if (isCoralDataValid) {
        int angleX10 = (data.data[0] << 8) | data.data[1];
        m_coralIntakeAngleDegrees = angleX10 / 10.0;

        int proximity = (data.data[2] << 8) | data.data[3];
        m_coralCollected = proximity > kCoralProximityThreshold;
    }
}
