#include "io/FeatherCanDecoder.h"

FeatherCanDecoder::FeatherCanDecoder() {
    m_coralIntakeAngleDegrees = 0.0;
    m_coralCollected = false;
}

void FeatherCanDecoder::Update() {
    // @todo Read and unpack the CAN bus data to set m_coralIntakeAngleDegrees and m_coralCollected
}

float FeatherCanDecoder::GetCoralIntakeAngleDegrees() {
    return m_coralIntakeAngleDegrees;
}

bool FeatherCanDecoder::IsCoralCollected() {
    return m_coralCollected;
}
