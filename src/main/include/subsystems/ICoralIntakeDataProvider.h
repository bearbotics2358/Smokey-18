#pragma once

class ICoralIntakeDataProvider {
public:
    // Provide the angle of the coral intake/delivery mechanism in degrees
    virtual float GetCoralIntakeAngleDegrees() = 0;

    // Provide the raw angle of the coral mechanism. Does not include any offsets.
    virtual float GetCoralIntakeRawAngleDegrees() = 0;

    // Return a simple boolean to indicate whether a piece of coral has been collected
    virtual bool IsCoralCollected() = 0;
};
