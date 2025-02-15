#include "io/FeatherCanDecoder.h"

#include "frc/smartdashboard/SmartDashboard.h"

FeatherCanDecoder::FeatherCanDecoder():m_coralCAN(kCoralCanID)
{
    m_coralIntakeAngleDegrees = 0.0;
    m_coralCollected = false;
}

void FeatherCanDecoder::Update() {
    UnpackCoralCANData();

    frc::SmartDashboard::PutNumber("Angles of Coral FeatherCan", m_coralIntakeAngleDegrees);
}

float FeatherCanDecoder::GetCoralIntakeAngleDegrees() {
    return m_coralIntakeAngleDegrees;
}

bool FeatherCanDecoder::IsCoralCollected() {
    return m_coralCollected;
}

void FeatherCanDecoder::UnpackCoralCANData() {
    frc::CANData data;

    bool is_coral_data_valid = m_coralCAN.ReadPacketNew(kCoralAPIId, &data);

    if (is_coral_data_valid) {
        int angle_x10 = (data.data[0] << 8) | data.data[1];
        m_coralIntakeAngleDegrees = angle_x10 / 10.0;
    }
}
