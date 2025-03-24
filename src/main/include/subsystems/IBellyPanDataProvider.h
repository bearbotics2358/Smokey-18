#pragma once

class IBellyPanDataProvider {
public:
    virtual double GetBellyPanRightDistance() = 0;
    virtual double GetBellyPanLeftDistance() = 0;
};