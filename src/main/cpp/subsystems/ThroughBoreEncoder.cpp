#include <subsystems/ThroughBoreEncoder.h>

ThroughBoreEncoder::ThroughBoreEncoder() {

};

// The two functions below are examples

double ThroughBoreEncoder::GetDistanceExample () {
    return throughboreencoder.GetDistance();
};

double ThroughBoreEncoder::GetDirectionExample () {
    return throughboreencoder.GetDirection();
};