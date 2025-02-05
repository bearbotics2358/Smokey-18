#include <subsystems/ThroughBoreEncoder.h>

ThroughBoreEncoder::ThroughBoreEncoder() {

};

double ThroughBoreEncoder::GetDistanceExample () {
    return throughboreencoder.GetDistance();
};

double ThroughBoreEncoder::GetDirectionExample () {
    return throughboreencoder.GetDirection();
};