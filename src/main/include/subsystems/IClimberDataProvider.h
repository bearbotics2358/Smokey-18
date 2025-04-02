/**
 * @file IClimberDataProvider.h
 */

#pragma once

/**
 * @brief Contains all the functions necessary for the climber mechanism.
 */
class IClimberDataProvider {
public:
    /** 
     * @brief Provide the angle of the climber intake/delivery mechanism in degrees.
     */ 
    virtual float GetClimberAngleDegrees() = 0;

    /**
     * @brief Provide the raw angle of the climber mechanism. Does not include any offsets.
     */ 
    virtual float GetClimberRawAngleDegrees() = 0;

    /**
     * @brief Return a simple boolean to indicate whether the right part of climber has collected the cage.
     */ 
    virtual bool IsRightCageHooked() = 0;

    /**
     * @brief Return a simple boolean to indicate whether the left part of climber has collected the cage.
     */ 
    virtual bool IsLeftCageHooked() = 0;
};