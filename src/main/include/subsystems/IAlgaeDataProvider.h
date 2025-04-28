#pragma once

class IAlgaeDataProvider {
public:
    // Provide the angle of the algae intake/delivery mechanism in degrees
    virtual float GetAlgaeAngleDegrees() = 0;

    // Provide the raw angle of the algae mechanism. Does not include any offsets.
    virtual float GetAlgaeRawAngleDegrees() = 0;

    // Return a simple boolean to indicate whether a piece of algae has been collected
    virtual bool IsAlgaeCollected() = 0;


    virtual bool IsAlgaeAngleValid() = 0;
};