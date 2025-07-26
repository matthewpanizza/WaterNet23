#pragma once

/**
 * Abstract base class for compass sensors
 * Provides a unified interface for different compass types (LIS3MDL, LSM303, etc.)
 */
class CompassBase {
public:
    // Constructor
    CompassBase() : mag_x(0.0), mag_y(0.0), mag_z(0.0) {}
    
    // Virtual destructor for proper cleanup
    virtual ~CompassBase() {}
    
    // Pure virtual functions that must be implemented by derived classes
    virtual bool begin() = 0;
    virtual bool readMagnetometer() = 0;
    virtual float getCompassHeading() = 0;
    
    // Optional virtual functions with default implementations
    virtual bool isConnected() { return connected; }
    virtual const char* getType() { return "Unknown"; }
    
protected:
    // Magnetic field readings accessible to derived classes
    float mag_x;
    float mag_y;
    float mag_z;
    
    // Connection status
    bool connected = false;
    
    // Helper function for heading calculation (can be overridden)
    virtual float calculateHeading(float x, float y) {
        float heading = atan2(y, x) * 180.0 / M_PI;
        if (heading < 0) heading += 360.0;
        return heading;
    }
};
