#pragma once

class IClimberDataProvider {
public:
    // Provide the angle of the climber intake/delivery mechanism in degrees
    virtual float GetClimberAngleDegrees() = 0;

    // Provide the raw angle of the climber mechanism. Does not include any offsets.
    virtual float GetClimberRawAngleDegrees() = 0;

    // Return a simple boolean to indicate whether a piece of climber has collected the cage
    virtual bool IsRightCageHooked() = 0;
    virtual bool IsLeftCageHooked() = 0;
};