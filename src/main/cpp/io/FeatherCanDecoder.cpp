#include "io/FeatherCanDecoder.h"

#include "frc/smartdashboard/SmartDashboard.h"

FeatherCanDecoder::FeatherCanDecoder():
m_coralCAN(kCoralDeviceID),
m_climberCAN(kClimberDeviceID)
{
    m_coralIntakeAngleDegrees = 0.0;
    m_coralCollected = false;

    m_climberAngleDegrees = 0.0;
    // m_climberCollected = false;
}

void FeatherCanDecoder::Update() {
    UnpackCoralCANData();

    frc::SmartDashboard::PutNumber("Raw Angle of Coral FeatherCan", GetCoralIntakeRawAngleDegrees());
    frc::SmartDashboard::PutNumber("Angle of Coral FeatherCan", GetCoralIntakeAngleDegrees());
    frc::SmartDashboard::PutBoolean("Coral Collected?", m_coralCollected);

    UnpackClimberCANData();

    frc::SmartDashboard::PutNumber("Raw Angle of Climber FeatherCan", GetClimberRawAngleDegrees());
    frc::SmartDashboard::PutNumber("Angle of Climber FeatherCan", GetClimberAngleDegrees());
    frc::SmartDashboard::PutBoolean("Climber Collected?", m_climberCollected);
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


float FeatherCanDecoder::GetClimberAngleDegrees() {
    return m_climberAngleDegrees - kClimberAngleOffsetDegrees;
}

float FeatherCanDecoder::GetClimberRawAngleDegrees() {
    return m_climberAngleDegrees;
}

bool FeatherCanDecoder::IsCageHooked() {
    return m_climberCollected;
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

void FeatherCanDecoder::UnpackClimberCANData() {
    frc::CANData data;

    bool isClimberDataValid = m_climberCAN.ReadPacketNew(kClimberAPIId, &data);

    if (isClimberDataValid) {
        int angleX10 = (data.data[0] << 8) | data.data[1];
        m_climberAngleDegrees = -angleX10 / 10.0;

        int proximity = (data.data[2] << 8) | data.data[3];
        frc::SmartDashboard::PutNumber("Is the Climber hooked?", proximity);
        m_climberCollected = proximity > kClimberProximityThreshold;
    }
}