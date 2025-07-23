#pragma once

#include "Compass.h"
#include <LSM303.h>

/**
 * LSM303 compass implementation
 */
class LSM303Compass : public Compass {
public:
    LSM303Compass(LSM303* sensor) : sensor_(sensor) {}
    
    bool begin() override {
        if (!sensor_) return false;
        
        sensor_->init();
        if (sensor_->last_status != 0) {
            connected = false;
            return false;
        }
        
        sensor_->enableDefault();
        connected = true;
        return true;
    }
    
    bool readMagnetometer() override {
        if (!sensor_ || !connected) return false;
        
        sensor_->read();
        if (sensor_->last_status == 0) {
            // LSM303 library provides raw values, convert to float
            mag_x = sensor_->m.x;
            mag_y = sensor_->m.y;
            mag_z = sensor_->m.z;
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
        return "LSM303";
    }
    
private:
    LSM303* sensor_;
};
