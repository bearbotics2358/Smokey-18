/**
 * @file IBellyPanDataProvider.h
 */

#pragma once

/**
 * @brief Contains all the functions necessary for getting Time of Flight sensor data.
 */
class IBellyPanDataProvider {
public:
    /**
     * @todo Use this function.
     */
    virtual double GetBellyPanRightDistance() = 0;
    
    /**
     * @todo Use this function.
     */
    virtual double GetBellyPanLeftDistance() = 0;
};