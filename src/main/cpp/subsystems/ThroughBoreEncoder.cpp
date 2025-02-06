#include <subsystems/ThroughBoreEncoder.h>

ThroughBoreEncoder::ThroughBoreEncoder() {};

// The two functions below are examples
double ThroughBoreEncoder::GetDistance() {
    return m_throughBorEncoder.GetDistance();
};

double ThroughBoreEncoder::GetDirection() {
    return m_throughBorEncoder.GetDirection();
};