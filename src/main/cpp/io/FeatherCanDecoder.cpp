#include "io/FeatherCanDecoder.h"

#include "frc/smartdashboard/SmartDashboard.h"

FeatherCanDecoder::FeatherCanDecoder():
m_coralCAN(kCoralDeviceID),
m_algaeCAN(kAlgaeDeviceID),
m_climberCAN(kClimberDeviceID),
m_bellyPanCAN(kBellyPanDeviceID)
{
    m_coralIntakeAngleDegrees = 0.0;
    m_coralCollected = false;

    m_algaeAngleDegrees = 0.0;
    m_algaeCollected = false;

    m_climberAngleDegrees = 0.0;
}

/**
 * Detect when the FeatherCAN sent a bad reading for the time of flight (TOF) sensor.
 */
bool FeatherCanDecoder::IsTOFDataValid(int tofReading) {
    static constexpr int kBadTOFValue = 0xFFFF;

    return kBadTOFValue != tofReading;
}

void FeatherCanDecoder::Update() {
    UnpackCoralCANData();
    frc::SmartDashboard::PutNumber("Raw Angle of Coral FeatherCan", GetCoralIntakeRawAngleDegrees());
    frc::SmartDashboard::PutNumber("Angle of Coral FeatherCan", GetCoralIntakeAngleDegrees());
    frc::SmartDashboard::PutBoolean("Coral Collected?", m_coralCollected);

    UnpackAlgaeCANData();
    frc::SmartDashboard::PutNumber("Algae Raw Angle", GetAlgaeRawAngleDegrees());
    frc::SmartDashboard::PutNumber("Algae Angle", GetAlgaeAngleDegrees());

    UnpackClimberCANData();
    frc::SmartDashboard::PutNumber("Raw Angle of Climber FeatherCan", GetClimberRawAngleDegrees());
    frc::SmartDashboard::PutNumber("Angle of Climber FeatherCan", GetClimberAngleDegrees());
    frc::SmartDashboard::PutBoolean("Climber Right Collected?", m_rightProximity);
    frc::SmartDashboard::PutBoolean("Climber Left Collected?", m_leftProximity);

    UnpackBellyPanCANData();
}

float FeatherCanDecoder::GetCoralIntakeAngleDegrees() {
    return m_coralIntakeAngleDegrees - kCoralAngleOffsetDegrees;
}

float FeatherCanDecoder::GetCoralIntakeRawAngleDegrees() {
    return m_coralIntakeAngleDegrees;
}

bool FeatherCanDecoder::IsCoralAngleValid() {
    return m_coralAngleValid;
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
    // @todo The kAlgaeProximityThreshold TOF threshold needs to be tuned before this can be re-enabled
    //return m_algaeCollected;
    return false;
}

bool FeatherCanDecoder::IsAlgaeAngleValid() {
    return m_algaeAngleValid;
}

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

double FeatherCanDecoder::GetBellyPanLeftDistance() {
    return m_leftBellyPanDistance;
}

double FeatherCanDecoder::GetBellyPanRightDistance() {
    return m_rightBellyPanDistance;
}

void FeatherCanDecoder::UnpackCoralCANData() {
    frc::CANData data;

    bool isCoralDataValid = m_coralCAN.ReadPacketTimeout(kCoralAPIId, kCANReadTimeoutMs, &data);

    if (isCoralDataValid) {
        int angleX10 = (data.data[0] << 8) | data.data[1];
        m_coralIntakeAngleDegrees = -angleX10 / 10.0;

        int proximity = (data.data[2] << 8) | data.data[3];
        frc::SmartDashboard::PutNumber("Coral Collected Value", proximity);
        m_coralCollected = proximity > kCoralProximityThreshold;
    }

    m_coralAngleValid = isCoralDataValid;
}

void FeatherCanDecoder::UnpackAlgaeCANData() {
    frc::CANData data;

    bool isAlgaeDataValid = m_algaeCAN.ReadPacketTimeout(kAlgaeAPIId, kCANReadTimeoutMs, &data);
    frc::SmartDashboard::PutBoolean("is algae data valid", isAlgaeDataValid);

    if (isAlgaeDataValid) {
        int angleX10 = (data.data[0] << 8) | data.data[1];
        m_algaeAngleDegrees = -angleX10 / 10.0;
        frc::SmartDashboard::PutNumber("Algae Raw Angle", m_algaeAngleDegrees);

        int proximity = (data.data[2] << 8) | data.data[3];
        frc::SmartDashboard::PutNumber("Algae Collected Value", proximity);

        if (IsTOFDataValid(proximity)) {
            // Only update the algae collected value if the TOF data is good. Otherwise we
            // may incorrectly report that algae is collected.
            m_algaeCollected = proximity > kAlgaeProximityThreshold;
        }
    }

    m_algaeAngleValid = isAlgaeDataValid;
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
        frc::SmartDashboard::PutNumber("BellyPan Right Distance", proximityright);

        if (IsTOFDataValid(proximityright)) {
            // Only update the value if the TOF data is good. Otherwise we
            // may incorrectly report the distance.
            m_rightBellyPanDistance = proximityright;
        }

        int proximityleft = (data.data[2] << 8) | data.data[3];
        frc::SmartDashboard::PutNumber("BellyPan Left Distance", proximityleft);

        if (IsTOFDataValid(proximityleft)) {
            // Only update the value if the TOF data is good. Otherwise we
            // may incorrectly report the distance.
            m_leftBellyPanDistance = proximityleft;
        }
    }
}