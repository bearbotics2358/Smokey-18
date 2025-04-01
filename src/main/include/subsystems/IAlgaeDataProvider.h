/**
 * @file IAlgaeDataProvider.h
 */

#pragma once

/**
 * @brief Contains all the functions necessary for the algae arm.
 */
class IAlgaeDataProvider {
public:
    /**
     * @brief Provide the angle of the algae intake/delivery mechanism in degrees
     */
    virtual float GetAlgaeAngleDegrees() = 0;

    /** 
     * @brief Provide the raw angle of the algae mechanism. Does not include any offsets.
     */ 
    virtual float GetAlgaeRawAngleDegrees() = 0;

    /**
     * @brief Return a simple boolean to indicate whether a piece of algae has been collected
     */
    virtual bool IsAlgaeCollected() = 0;
};