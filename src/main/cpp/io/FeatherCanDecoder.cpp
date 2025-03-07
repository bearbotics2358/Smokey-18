#include "io/FeatherCanDecoder.h"

#include "frc/smartdashboard/SmartDashboard.h"

FeatherCanDecoder::FeatherCanDecoder():m_coralCAN(kCoralDeviceID),m_algaeCAN(kAlgaeDeviceID)
{
    m_coralIntakeAngleDegrees = 0.0;
    m_coralCollected = false;
    m_algaeAngleDegrees = 0.0;
    m_algaeCollected = false;
}

void FeatherCanDecoder::Update() {
    UnpackCoralCANData();

    frc::SmartDashboard::PutNumber("Raw Angle of Coral FeatherCan", GetCoralIntakeRawAngleDegrees());
    frc::SmartDashboard::PutNumber("Angle of Coral FeatherCan", GetCoralIntakeAngleDegrees());
    frc::SmartDashboard::PutBoolean("Coral Collected?", m_coralCollected);
}

float FeatherCanDecoder::GetCoralIntakeAngleDegrees() {
    return m_coralIntakeAngleDegrees - kCoralAngleOffsetDegrees;
}

float FeatherCanDecoder::GetCoralIntakeRawAngleDegrees() {
    return m_coralIntakeAngleDegrees;
}

bool FeatherCanDecoder::IsCoralCollected() {
    return m_coralCollected;
}

float FeatherCanDecoder::GetAlgaeAngleDegrees() {
    return m_algaeAngleDegrees - kAlgaeAngleOffsetDegrees;
}

float FeatherCanDecoder::GetAlgaeRawAngleDegrees() {
    return m_algaeAngleDegrees;
}

bool FeatherCanDecoder::IsAlgaeCollected() {
    return m_algaeCollected;
}

void FeatherCanDecoder::UnpackCoralCANData() {
    frc::CANData data;

    bool isCoralDataValid = m_coralCAN.ReadPacketNew(kCoralAPIId, &data);

    if (isCoralDataValid) {
        int angleX10 = (data.data[0] << 8) | data.data[1];
        m_coralIntakeAngleDegrees = -angleX10 / 10.0;

        int proximity = (data.data[2] << 8) | data.data[3];
        frc::SmartDashboard::PutNumber("Coral Collected Value", proximity);
        m_coralCollected = proximity > kCoralProximityThreshold;
    }
}

void FeatherCanDecoder::UnpackAlgaeCANData() {
    frc::CANData data;

    bool isAlgaeDataValid = m_algaeCAN.ReadPacketNew(kAlgaeAPIId, &data);

    if (isAlgaeDataValid) {
        int angleX10 = (data.data[0] << 8) | data.data[1];
        m_algaeAngleDegrees = -angleX10 / 10.0;

        int proximity = (data.data[2] << 8) | data.data[3];
        frc::SmartDashboard::PutNumber("Algae Collected Value", proximity);
        m_algaeCollected = proximity > kAlgaeProximityThreshold;
    }
}