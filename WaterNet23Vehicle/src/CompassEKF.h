#ifndef COMPASS_EKF_H
#define COMPASS_EKF_H

#include "Particle.h"
#include <math.h>

class CompassEKF {
public:
    CompassEKF();
    void init(float initial_heading, float initial_heading_rate = 0.0);
    void predict(float dt, float angular_velocity = 0.0);
    void updateCompass(float compass_heading, float compass_variance = 25.0);
    void updateGPS(float gps_course, float gps_speed, float gps_variance = 100.0);
    float getHeading();
    float getHeadingRate();
    bool isInitialized();

private:
    // State vector: [heading, heading_rate]
    float state[2];
    
    // Error covariance matrix (2x2)
    float P[2][2];
    
    // Process noise covariance
    float Q[2][2];
    
    // Measurement noise covariance
    float R_compass;
    float R_gps;
    
    bool initialized;
    
    // Helper functions
    float normalizeAngle(float angle);
    void matrixMultiply2x2(float A[2][2], float B[2][2], float result[2][2]);
    void matrixInvert2x2(float matrix[2][2], float result[2][2]);
    float determinant2x2(float matrix[2][2]);
};

#endif // COMPASS_EKF_H
