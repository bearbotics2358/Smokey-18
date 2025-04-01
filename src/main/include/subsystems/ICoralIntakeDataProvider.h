/**
 * @file IClimberDataProvider.h
 */

#pragma once

/**
 * @brief Contains all the functions necessary for the coral arm.
 */
class ICoralIntakeDataProvider {
public:
    /**
     * @brief Provide the angle of the coral intake/delivery mechanism in degrees.
     */ 
    virtual float GetCoralIntakeAngleDegrees() = 0;

    /** 
     * @brief Provide the raw angle of the coral mechanism. Does not include any offsets.
     */ 
    virtual float GetCoralIntakeRawAngleDegrees() = 0;

    /** 
     * @brief Determines whether the **absolute** coral angle data is valid.
     */ 
    virtual bool IsCoralAngleValid() = 0;

    /** 
     * @brief Return a simple boolean to indicate whether a piece of coral has been collected
     */ 
    virtual bool IsCoralCollected() = 0;
};
