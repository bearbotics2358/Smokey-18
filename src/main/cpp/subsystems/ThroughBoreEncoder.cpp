#include <subsystems/ThroughBoreEncoder.h>

ThroughBoreEncoder::ThroughBoreEncoder(int dioPortA, int dioPortB): 
m_throughBoreEncoder{dioPortA, dioPortB, false, frc::Encoder::EncodingType::k4X} 
{
    // The number of pulses per revolution.
    m_throughBoreEncoder.SetDistancePerPulse(1.0 / 2048.0);
    // Resets the distance to zero.
    m_throughBoreEncoder.Reset();
};

// The two functions below are examples
double ThroughBoreEncoder::GetDistance() {
    return m_throughBoreEncoder.GetDistance();
};

double ThroughBoreEncoder::GetDirection() {
    return m_throughBoreEncoder.GetDirection();
};

void ThroughBoreEncoder::ResetEncoder() {
    m_throughBoreEncoder.Reset();
}