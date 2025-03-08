#pragma once

class IBellyPanDataProvider {
public:
    virtual bool IsRightProximity() = 0;
    virtual bool IsLeftProximity() = 0;
};