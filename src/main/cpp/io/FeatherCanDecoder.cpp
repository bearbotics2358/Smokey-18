#include "io/FeatherCanDecoder.h"

#include "frc/smartdashboard/SmartDashboard.h"

FeatherCanDecoder::FeatherCanDecoder():
m_coralCAN(kCoralDeviceID),
m_climberCAN(kClimberDeviceID),
m_bellyPanCAN(kBellyPanDeviceID),
m_algaeCAN(kAlgaeDeviceID)
{
    m_coralIntakeAngleDegrees = 0.0;
    m_coralCollected = false;

    m_algaeAngleDegrees = 0.0;
    m_algaeCollected = false;

    m_climberAngleDegrees = 0.0;
}

void FeatherCanDecoder::Update() {
    UnpackCoralCANData();
    frc::SmartDashboard::PutNumber("Raw Angle of Coral FeatherCan", GetCoralIntakeRawAngleDegrees());
    frc::SmartDashboard::PutNumber("Angle of Coral FeatherCan", GetCoralIntakeAngleDegrees());
    frc::SmartDashboard::PutBoolean("Coral Collected?", m_coralCollected);

    UnpackAlgaeCANData();

    UnpackClimberCANData();
    frc::SmartDashboard::PutNumber("Raw Angle of Climber FeatherCan", GetClimberRawAngleDegrees());
    frc::SmartDashboard::PutNumber("Angle of Climber FeatherCan", GetClimberAngleDegrees());
    frc::SmartDashboard::PutBoolean("Climber Collected?", m_rightProximity);
    frc::SmartDashboard::PutBoolean("Climber Collected?", m_leftProximity);

    UnpackBellyPanCANData();
    frc::SmartDashboard::PutBoolean("BellyPan Right Proximity?", m_rightBellyPanProximity);
    frc::SmartDashboard::PutBoolean("BellyPan Left Proximity?", m_leftBellyPanProximity);
}

// **** ICoralIntakeDataProvider interface functions **** //
float FeatherCanDecoder::GetCoralIntakeAngleDegrees() {
    return m_coralIntakeAngleDegrees - kCoralAngleOffsetDegrees;
}

float FeatherCanDecoder::GetCoralIntakeRawAngleDegrees() {
    return m_coralIntakeAngleDegrees;
}

bool FeatherCanDecoder::IsCoralCollected() {
    return m_coralCollected;
}

// **** IAlgaeDataProvider interface functions **** //
float FeatherCanDecoder::GetAlgaeAngleDegrees() {
    return -(m_algaeAngleDegrees - kAlgaeAngleOffsetDegrees);
}

float FeatherCanDecoder::GetAlgaeRawAngleDegrees() {
    return m_algaeAngleDegrees;
}

bool FeatherCanDecoder::IsAlgaeCollected() {
    return m_algaeCollected;
}

// **** IClimberDataProvider interface functions **** //
float FeatherCanDecoder::GetClimberAngleDegrees() {
    return m_climberAngleDegrees - kClimberAngleOffsetDegrees;
}

float FeatherCanDecoder::GetClimberRawAngleDegrees() {
    return m_climberAngleDegrees;
}

bool FeatherCanDecoder::IsLeftCageHooked() {
    return m_leftProximity;
}

bool FeatherCanDecoder::IsRightCageHooked() {
    return m_rightProximity;
}

// **** IBellyPanDataProvider interface functions **** //
bool FeatherCanDecoder::IsLeftProximity() {
    return m_leftBellyPanProximity;
}

bool FeatherCanDecoder::IsRightProximity() {
    return m_rightBellyPanProximity;
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
    frc::SmartDashboard::PutBoolean("is algae data valid", isAlgaeDataValid);

    if (isAlgaeDataValid) {
        int angleX10 = (data.data[0] << 8) | data.data[1];
        m_algaeAngleDegrees = -angleX10 / 10.0;
        frc::SmartDashboard::PutNumber("Algae Raw Angle", m_algaeAngleDegrees);

        int proximity = (data.data[2] << 8) | data.data[3];
        frc::SmartDashboard::PutNumber("Algae Collected Value", proximity);
        m_algaeCollected = proximity > kAlgaeProximityThreshold;
    }
}

void FeatherCanDecoder::UnpackClimberCANData() {
    frc::CANData data;

    bool isClimberDataValid = m_climberCAN.ReadPacketNew(kClimberAPIId, &data);

    if (isClimberDataValid) {
        // TODO: Look at the Arduino documentation
        int angleX10 = (data.data[0] << 8) | data.data[1];
        m_climberAngleDegrees = -angleX10 / 10.0;

        int proximityright = (data.data[2] << 8) | data.data[3];
        frc::SmartDashboard::PutNumber("Is the Climber right hooked?", proximityright);
        m_rightProximity = proximityright > kClimberProximityThreshold;

        int proximityleft = (data.data[4] << 8) | data.data[5]; //todo: change 4 & 5 here to the correct ports
        frc::SmartDashboard::PutNumber("Is the Climber left hooked?", proximityleft);
        m_leftProximity = proximityleft > kClimberProximityThreshold;
    }
}

void FeatherCanDecoder::UnpackBellyPanCANData() {
    frc::CANData data;

    bool isBellyPanDataValid = m_bellyPanCAN.ReadPacketNew(kClimberAPIId, &data);

    if (isBellyPanDataValid) {
        int proximityright = (data.data[0] << 8) | data.data[1];
        frc::SmartDashboard::PutNumber("Is the BellyPan in proximity?", proximityright);
        m_rightBellyPanProximity = proximityright > kBellyPanProximityThreshold;

        int proximityleft = (data.data[2] << 8) | data.data[3]; 
        frc::SmartDashboard::PutNumber("Is the BellyPan in proximity?", proximityleft);
        m_leftBellyPanProximity = proximityleft > kBellyPanProximityThreshold;
    }
}