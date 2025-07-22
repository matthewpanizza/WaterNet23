#include "CompassEKF.h"

CompassEKF::CompassEKF() : initialized(false) {
    // Initialize process noise covariance Q
    Q[0][0] = 0.01;  // heading process noise
    Q[0][1] = 0.0;
    Q[1][0] = 0.0;
    Q[1][1] = 0.1;   // heading rate process noise
    
    // Initialize measurement noise
    R_compass = 25.0;  // compass variance (degrees^2)
    R_gps = 100.0;     // GPS course variance (degrees^2)
}

void CompassEKF::init(float initial_heading, float initial_heading_rate) {
    // Initialize state
    state[0] = normalizeAngle(initial_heading);
    state[1] = initial_heading_rate;
    
    // Initialize error covariance
    P[0][0] = 25.0;  // initial heading uncertainty
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 10.0;  // initial heading rate uncertainty
    
    initialized = true;
}

void CompassEKF::predict(float dt, float angular_velocity) {
    if (!initialized) return;
    
    // State prediction: x = F * x + B * u
    float new_heading = state[0] + state[1] * dt;
    float new_heading_rate = state[1]; // assume constant heading rate
    
    // If we have angular velocity measurement, use it
    if (angular_velocity != 0.0) {
        new_heading_rate = angular_velocity;
    }
    
    state[0] = normalizeAngle(new_heading);
    state[1] = new_heading_rate;
    
    // Jacobian of state transition (F matrix)
    float F[2][2] = {{1.0, dt}, {0.0, 1.0}};
    
    // Predict error covariance: P = F * P * F^T + Q
    float temp[2][2];
    float FT[2][2] = {{1.0, 0.0}, {dt, 1.0}}; // F transpose
    
    matrixMultiply2x2(F, P, temp);
    matrixMultiply2x2(temp, FT, P);
    
    // Add process noise
    P[0][0] += Q[0][0];
    P[0][1] += Q[0][1];
    P[1][0] += Q[1][0];
    P[1][1] += Q[1][1];
}

void CompassEKF::updateCompass(float compass_heading, float compass_variance) {
    if (!initialized) return;
    
    R_compass = compass_variance;
    
    // Measurement residual
    float y = normalizeAngle(compass_heading - state[0]);
    
    // Measurement Jacobian H = [1, 0] for heading measurement
    float H[1][2] = {{1.0, 0.0}};
    
    // Innovation covariance S = H * P * H^T + R
    float S = P[0][0] + R_compass;
    
    // Kalman gain K = P * H^T * S^(-1)
    float K[2] = {P[0][0] / S, P[1][0] / S};
    
    // State update: x = x + K * y
    state[0] = normalizeAngle(state[0] + K[0] * y);
    state[1] = state[1] + K[1] * y;
    
    // Covariance update: P = (I - K * H) * P
    float temp_P[2][2];
    temp_P[0][0] = P[0][0] - K[0] * P[0][0];
    temp_P[0][1] = P[0][1] - K[0] * P[0][1];
    temp_P[1][0] = P[1][0] - K[1] * P[0][0];
    temp_P[1][1] = P[1][1] - K[1] * P[0][1];
    
    P[0][0] = temp_P[0][0];
    P[0][1] = temp_P[0][1];
    P[1][0] = temp_P[1][0];
    P[1][1] = temp_P[1][1];
}

void CompassEKF::updateGPS(float gps_course, float gps_speed, float gps_variance) {
    if (!initialized) return;
    
    // Only use GPS course if speed is sufficient (avoid noise at low speeds)
    if (gps_speed < 0.5) return; // 0.5 m/s minimum speed
    
    R_gps = gps_variance;
    
    // Measurement residual
    float y = normalizeAngle(gps_course - state[0]);
    
    // Same update as compass (H = [1, 0])
    float S = P[0][0] + R_gps;
    float K[2] = {P[0][0] / S, P[1][0] / S};
    
    // State update
    state[0] = normalizeAngle(state[0] + K[0] * y);
    state[1] = state[1] + K[1] * y;
    
    // Covariance update
    float temp_P[2][2];
    temp_P[0][0] = P[0][0] - K[0] * P[0][0];
    temp_P[0][1] = P[0][1] - K[0] * P[0][1];
    temp_P[1][0] = P[1][0] - K[1] * P[0][0];
    temp_P[1][1] = P[1][1] - K[1] * P[0][1];
    
    P[0][0] = temp_P[0][0];
    P[0][1] = temp_P[0][1];
    P[1][0] = temp_P[1][0];
    P[1][1] = temp_P[1][1];
}

float CompassEKF::getHeading() {
    return initialized ? state[0] : 0.0;
}

float CompassEKF::getHeadingRate() {
    return initialized ? state[1] : 0.0;
}

bool CompassEKF::isInitialized() {
    return initialized;
}

float CompassEKF::normalizeAngle(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

void CompassEKF::matrixMultiply2x2(float A[2][2], float B[2][2], float result[2][2]) {
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    result[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1];
    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
    result[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1];
}
