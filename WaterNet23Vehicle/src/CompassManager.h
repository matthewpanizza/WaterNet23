#ifndef COMPASS_MANAGER_H
#define COMPASS_MANAGER_H

#include "Particle.h"
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include "LSM303.h"
#include "SimulationData.h"

// Compass type enumeration
enum CompassType {
    COMPASS_TYPE_LSM303 = 0,
    COMPASS_TYPE_LIS3MDL = 1,
    COMPASS_TYPE_AUTO = 2  // Try to auto-detect
};

class CompassManager {
private:
    CompassType activeType;
    bool initialized;
    Adafruit_LIS3MDL* lis3mdl_ptr;
    LSM303* lsm303_ptr;
    int calibrationOffset;
    float lastHeading;

public:
    // Constructor
    CompassManager() : activeType(COMPASS_TYPE_AUTO), initialized(false), 
                      lis3mdl_ptr(nullptr), lsm303_ptr(nullptr), 
                      calibrationOffset(0), lastHeading(0.0) {}
    
    // Initialize compass with specified type and external sensor objects
    bool begin(CompassType type, Adafruit_LIS3MDL* lis3mdl = nullptr, LSM303* lsm303 = nullptr) {
        lis3mdl_ptr = lis3mdl;
        lsm303_ptr = lsm303;
        
        if (type == COMPASS_TYPE_AUTO) {
            return autoDetectAndInitialize();
        } else {
            return initializeCompass(type);
        }
        return false;
    }
    
    // Check if compass is initialized and working
    bool isInitialized() const {
        return initialized;
    }
    
    // Get compass type
    CompassType getCompassType() const {
        return activeType;
    }
    
    // Get compass type as string
    const char* getCompassTypeString() const {
        switch(activeType) {
            case COMPASS_TYPE_LIS3MDL:
                return "LIS3MDL";
            case COMPASS_TYPE_LSM303:
                return "LSM303";
            default:
                return "None";
        }
    }
    
    // Check if compass is connected and responding
    bool isConnected() {
        if (!initialized) return false;
        
        switch(activeType) {
            case COMPASS_TYPE_LIS3MDL:
                if (lis3mdl_ptr) {
                    sensors_event_t event;
                    return lis3mdl_ptr->getEvent(&event);
                }
                break;
                
            case COMPASS_TYPE_LSM303:
                if (lsm303_ptr) {
                    lsm303_ptr->read();
                    return (lsm303_ptr->m.x != 0 || lsm303_ptr->m.y != 0 || lsm303_ptr->m.z != 0);
                }
                break;
        }
        return false;
    }
    
    // Get raw heading from compass (-180 to +180 degrees)
    float getRawHeading() {
        if (!initialized) return lastHeading;
        
        // Check if simulation is enabled
        extern SimulationData simulationData;
        if(simulationData.isSimulationEnabled()) {
            CompassSimData simData = simulationData.getCompassData();
            if(simData.valid) {
                // Convert from 0-360 range to -180 to +180 range
                float heading = simData.heading;
                if (heading > 180.0) {
                    heading -= 360.0;
                }
                lastHeading = heading;
                return lastHeading;
            }
        }
        
        float heading = 0.0;
        
        switch(activeType) {
            case COMPASS_TYPE_LIS3MDL:
                if (lis3mdl_ptr) {
                    sensors_event_t event;
                    if (lis3mdl_ptr->getEvent(&event)) {
                        heading = atan2(event.magnetic.y, event.magnetic.x) * 180.0 / M_PI;
                        lastHeading = normalizeAngle(heading);
                        return lastHeading;
                    }
                }
                break;
                
            case COMPASS_TYPE_LSM303:
                if (lsm303_ptr) {
                    lsm303_ptr->read();
                    heading = lsm303_ptr->heading();
                    // Convert from 0-360 range to -180 to +180 range
                    if (heading > 180.0) {
                        heading -= 360.0;
                    }
                    lastHeading = heading;
                    return lastHeading;
                }
                break;
        }
        
        return lastHeading;
    }
    
    // Read raw magnetometer values
    bool readMagnetometer(float& x, float& y, float& z) {
        if (!initialized) return false;
        
        // Check if simulation is enabled
        extern SimulationData simulationData;
        if(simulationData.isSimulationEnabled()) {
            CompassSimData simData = simulationData.getCompassData();
            if(simData.valid) {
                x = simData.x_mag;
                y = simData.y_mag;
                z = simData.z_mag;
                return true;
            }
        }
        
        switch(activeType) {
            case COMPASS_TYPE_LIS3MDL:
                if (lis3mdl_ptr) {
                    sensors_event_t event;
                    if (lis3mdl_ptr->getEvent(&event)) {
                        x = event.magnetic.x;
                        y = event.magnetic.y;
                        z = event.magnetic.z;
                        return true;
                    }
                }
                break;
                
            case COMPASS_TYPE_LSM303:
                if (lsm303_ptr) {
                    lsm303_ptr->read();
                    x = (float)lsm303_ptr->m.x;
                    y = (float)lsm303_ptr->m.y;
                    z = (float)lsm303_ptr->m.z;
                    return true;
                }
                break;
        }
        
        return false;
    }
    
    // Calibration functions
    void setCalibrationOffset(int offset) {
        calibrationOffset = offset;
    }
    
    int getCalibrationOffset() const {
        return calibrationOffset;
    }
    
    float getLastHeading() const {
        return lastHeading;
    }
    
    // Utility function to normalize angle to -180 to +180 range
    static float normalizeAngle(float angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

private:
    // Initialize specific compass type
    bool initializeCompass(CompassType type) {
        switch (type) {
            case COMPASS_TYPE_LIS3MDL:
                if (lis3mdl_ptr && !lis3mdl_ptr->begin_I2C()) {
                    Serial.println("Failed to find LIS3MDL chip");
                    return false;
                }
                if (lis3mdl_ptr) {
                    Serial.println("LIS3MDL Found!");
                    // Configure the LIS3MDL with optimal settings
                    lis3mdl_ptr->setPerformanceMode(LIS3MDL_HIGHMODE);
                    lis3mdl_ptr->setOperationMode(LIS3MDL_CONTINUOUSMODE);
                    lis3mdl_ptr->setDataRate(LIS3MDL_DATARATE_155_HZ);
                    lis3mdl_ptr->setRange(LIS3MDL_RANGE_8_GAUSS);
                    lis3mdl_ptr->setIntThreshold(500);
                    lis3mdl_ptr->configInterrupt(false, false, true, true, false, true);
                    activeType = COMPASS_TYPE_LIS3MDL;
                    initialized = true;
                    return true;
                }
                break;
                
            case COMPASS_TYPE_LSM303:
                if (lsm303_ptr && !lsm303_ptr->init()) {
                    Serial.println("Failed to find LSM303 chip");
                    return false;
                }
                if (lsm303_ptr) {
                    Serial.println("LSM303 Found!");
                    lsm303_ptr->enableDefault();
                    lsm303_ptr->m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
                    lsm303_ptr->m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
                    activeType = COMPASS_TYPE_LSM303;
                    initialized = true;
                    return true;
                }
                break;
        }
        return false;
    }
    
    // Auto-detect and initialize compass
    bool autoDetectAndInitialize() {
        Serial.println("Auto-detecting compass...");
        
        // Try LIS3MDL first (newer sensor)
        if (initializeCompass(COMPASS_TYPE_LIS3MDL)) {
            Serial.println("LIS3MDL compass detected and initialized");
            return true;
        }
        
        // Try LSM303 if LIS3MDL fails
        if (initializeCompass(COMPASS_TYPE_LSM303)) {
            Serial.println("LSM303 compass detected and initialized");
            return true;
        }
        
        Serial.println("No compatible compass found");
        return false;
    }
};

#endif // COMPASS_MANAGER_H
