#pragma once

#include "CompassBase.h"

/**
 * LIS3MDL compass implementation
 */
class SimulationCompass : public CompassBase {
public:
    SimulationCompass(){}
    
    bool begin() override {        
        connected = true;
        return true;
    }
    
    bool readMagnetometer() override {
        
        return false;
    }
    
    float getCompassHeading() override { 
        return calculateHeading(mag_x, mag_y);
    }
    
    const char* getType() override {
        return "Simulated";
    }
};
