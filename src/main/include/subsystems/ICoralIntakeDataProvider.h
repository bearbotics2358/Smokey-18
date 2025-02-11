#pragma once

class ICoralIntakeDataProvider {
public:
    // Provide the angle of the coral intake/delivery mechanism in degrees
    // @todo Decide if this will return the raw angle or the offset from some fixed point (like straight out or down)
    virtual float GetCoralIntakeAngleDegrees() = 0;

    // Return a simple boolean to indicate whether a piece of coral has been collected
    virtual bool IsCoralCollected() = 0;
};
