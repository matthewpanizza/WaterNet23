#pragma once

/**
 * Abstract base class for GPS sensors
 * Provides a unified interface for different compass types (SparkFun uBlox)
 */
class GPSBase {
public:
    // Constructor
    GPSBase(){}
    
    // Virtual destructor for proper cleanup
    virtual ~GPSBase() {}
    
    // Pure virtual functions that must be implemented by derived classes
    virtual bool begin() = 0;               // Function to perform initialization of hardware
    virtual bool isConnected() = 0;         // Function to check if hardware is connected
    virtual float getLatitude() = 0;        // Function to retrieve latitude in degrees
    virtual float getLongitude() = 0;       // Function to retrieve longitude in degrees
    
    
    // Optional virtual functions with default implementations
    virtual const char* getType() { return "Unknown"; }
    virtual int32_t getGroundSpeed() { return 0; }  //Get the ground speed in m/s
    virtual int32_t getHeading() { return 0; }
    
protected:
    // Connection status
    bool connected = false;
};
