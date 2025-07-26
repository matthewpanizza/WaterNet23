#pragma once

#include "GPSBase.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

/**
 * Neo-M8U GPS implementation
 */
class NeoM8UGPS : public GPSBase {
public:
    NeoM8UGPS() : sensor_() {}  // Default constructor initializes the sensor instance
    
    bool begin(){                       // Function to perform initialization of hardware
        if(sensor_.begin() == false){
            Serial.println("Error, Could not initialize GPS");
            return false;
        }
        sensor_.setI2COutput(COM_TYPE_UBX);
        sensor_.setPortInput(COM_PORT_I2C, COM_TYPE_UBX);
        sensor_.setNavigationFrequency(2);
        Wire.setClock(400000); //Increase I2C clock speed to 400kHz
        return true;
    }    

    bool isConnected(){         // Function to check if hardware is connected
        return sensor_.isConnected();
    }   

    float getLatitude(){        // Function to retrieve latitude in degrees
        if(isConnected()) return ((float)sensor_.getLatitude())/10000000.0;
        return -999.0;
    }   

    float getLongitude(){       // Function to retrieve longitude in degrees
        if(isConnected()) return ((float)sensor_.getLongitude())/10000000.0;
        return -999.0;
    }       
    
    int32_t getGroundSpeed() override {     //Function to retrieve ground speed in m/s
        return sensor_.getGroundSpeed() * 0.001;    //Convert mm/s to m/s
    }

    int32_t getHeading() override {         // Function to get heading relative to North in degrees
        return sensor_.getHeading() / 100000.0;    //Convert to degrees
    }
    
    const char* getType() override {        // Function to get a string with the type of GPS
        return "Neo-M8U";
    }
    
private:
    SFE_UBLOX_GNSS sensor_;  // Instance stored directly in the class
};
