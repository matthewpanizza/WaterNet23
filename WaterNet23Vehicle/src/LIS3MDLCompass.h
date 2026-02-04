#pragma once

#include "CompassBase.h"
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

/**
 * LIS3MDL compass implementation
 */
class LIS3MDLCompass : public CompassBase {
public:
    LIS3MDLCompass() {}
    
    bool begin() override {
        if (!sensor_.begin_I2C()) {
            connected = false;
            return false;
        }
        
        // Configure sensor settings
        sensor_.setPerformanceMode(LIS3MDL_HIGHMODE);
        sensor_.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        sensor_.setDataRate(LIS3MDL_DATARATE_155_HZ);
        sensor_.setRange(LIS3MDL_RANGE_8_GAUSS);
        sensor_.setIntThreshold(500);
        sensor_.configInterrupt(false, false, true, true, false, true);
        
        connected = true;
        return true;
    }
    
    bool readMagnetometer() override {
        if (!connected) return false;
        
        sensor_.read();
        sensors_event_t event;
        if (sensor_.getEvent(&event)) {
            mag_x = event.magnetic.x;
            mag_y = event.magnetic.y;
            mag_z = event.magnetic.z;
            return true;
        }
        return false;
    }
    
    float getCompassHeading() override {
        if (!readMagnetometer()) {
            return NAN;  // Return NaN if reading failed
        }
        
        return calculateHeading(mag_x, mag_y);
    }
    
    const char* getType() override {
        return "LIS3MDL";
    }
    
private:
    Adafruit_LIS3MDL sensor_;
};
