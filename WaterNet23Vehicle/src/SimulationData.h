#ifndef SIMULATION_DATA_H
#define SIMULATION_DATA_H

#include "Particle.h"
#include "math.h"
#include "WaterNet23Vehicle.h"

// Simulation mode flags
#define SIM_MODE_DISABLED       0
#define SIM_MODE_STATIC         1   // Static position/heading
#define SIM_MODE_WAYPOINT       2   // Follow predefined waypoints
#define SIM_MODE_CIRCLE         3   // Circle pattern
#define SIM_MODE_RANDOM_WALK    4   // Random walk pattern

// Timing constants for realistic simulation
#define SIM_SEQUENCE_DURATION_MS    300000  // 5 minutes per sequence
#define SIM_UPDATE_INTERVAL_MS      1000    // Update every second
#define SIM_VEHICLE_SPEED_MPS       2.0     // 2 m/s vehicle speed
#define SIM_LOG_INTERVAL_MS         500     // Log every 500ms (0.5 seconds)

// Structure for GPS simulation data
struct GPSSimData {
    float latitude;
    float longitude;
    float altitude;
    float speed;       // m/s
    float course;      // degrees
    bool valid;
    uint32_t timestamp;
};

// Structure for compass simulation data
struct CompassSimData {
    float heading;     // degrees (0-360)
    float x_mag;       // magnetometer X reading
    float y_mag;       // magnetometer Y reading  
    float z_mag;       // magnetometer Z reading
    bool valid;
    uint32_t timestamp;
};

// Structure for combined sensor simulation
struct SimulationStep {
    GPSSimData gps;
    CompassSimData compass;
    uint32_t stepDuration_ms;  // How long to hold this data
};

class SimulationData {
private:
    static const int MAX_WAYPOINTS = 20;
    static const int MAX_SIM_STEPS = 50;
    
    // Current simulation state
    int currentMode;
    bool simulationEnabled;
    uint32_t lastUpdateTime;
    uint32_t simulationStartTime;
    uint32_t lastLogTime;
    int currentWaypoint;
    int currentStep;
    
    // Current vehicle position for smooth interpolation
    float currentLat;
    float currentLon;
    float currentHeading;
    float targetLat;
    float targetLon;
    float targetHeading;
    
    // Simulation logging
    bool logSimulationData;
    char simLogFilename[40];
    
    // Static test data
    GPSSimData staticGPS;
    CompassSimData staticCompass;
    
    // Waypoint data for testing autonomous navigation
    GPSSimData waypoints[MAX_WAYPOINTS];
    int waypointCount;
    
    // Complex simulation scenarios
    SimulationStep scenarios[MAX_SIM_STEPS];
    int scenarioStepCount;
    
    // Circle pattern parameters
    float circleCenter_lat;
    float circleCenter_lon;
    float circleRadius_m;
    float circleSpeed_mps;
    uint32_t circleStartTime;
    
    // Random walk parameters
    float randomWalk_lat;
    float randomWalk_lon;
    float randomWalk_heading;
    uint32_t randomWalkLastUpdate;
    
public:
    SimulationData() : currentMode(SIM_MODE_DISABLED), simulationEnabled(false), 
                      lastUpdateTime(0), simulationStartTime(0), lastLogTime(0),
                      currentWaypoint(0), currentStep(0), waypointCount(0), scenarioStepCount(0),
                      currentLat(0.0), currentLon(0.0), currentHeading(0.0),
                      targetLat(0.0), targetLon(0.0), targetHeading(0.0),
                      logSimulationData(false) {
        initializeTestData();
        strcpy(simLogFilename, "SimData");
    }
    
    void initializeTestData() {
        // Static test position (example: Lake location)
        staticGPS.latitude = 42.3601;      // Detroit area
        staticGPS.longitude = -83.0732;
        staticGPS.altitude = 177.0;        // meters
        staticGPS.speed = 0.0;
        staticGPS.course = 0.0;
        staticGPS.valid = true;
        staticGPS.timestamp = millis();
        
        staticCompass.heading = 45.0;      // Northeast heading
        staticCompass.x_mag = 150.0;
        staticCompass.y_mag = 150.0;
        staticCompass.z_mag = -200.0;
        staticCompass.valid = true;
        staticCompass.timestamp = millis();
        
        // Initialize test waypoints for autonomous navigation testing
        initializeWaypoints();
        
        // Initialize complex simulation scenarios
        initializeScenarios();
        
        // Circle pattern parameters (100m radius)
        circleCenter_lat = 42.3601;
        circleCenter_lon = -83.0732;
        circleRadius_m = 100.0;
        circleSpeed_mps = 1.0;  // 1 m/s
        circleStartTime = millis();
        
        // Random walk starting position
        randomWalk_lat = 42.3601;
        randomWalk_lon = -83.0732;
        randomWalk_heading = 0.0;
        randomWalkLastUpdate = millis();
    }
    
    void initializeWaypoints() {
        // Test waypoints forming a square pattern around start position
        // Each waypoint is ~100m from the previous
        waypointCount = 5;
        
        // Start position
        waypoints[0].latitude = 42.3601;
        waypoints[0].longitude = -83.0732;
        waypoints[0].altitude = 177.0;
        waypoints[0].valid = true;
        
        // North 100m
        waypoints[1].latitude = 42.3610;   // ~100m north
        waypoints[1].longitude = -83.0732;
        waypoints[1].altitude = 177.0;
        waypoints[1].valid = true;
        
        // East 100m
        waypoints[2].latitude = 42.3610;
        waypoints[2].longitude = -83.0720;  // ~100m east
        waypoints[2].altitude = 177.0;
        waypoints[2].valid = true;
        
        // South 100m
        waypoints[3].latitude = 42.3601;
        waypoints[3].longitude = -83.0720;
        waypoints[3].altitude = 177.0;
        waypoints[3].valid = true;
        
        // Back to start
        waypoints[4].latitude = 42.3601;
        waypoints[4].longitude = -83.0732;
        waypoints[4].altitude = 177.0;
        waypoints[4].valid = true;
    }
    
    void initializeScenarios() {
        // Scenario 1: Compass calibration test - rotating in place
        scenarioStepCount = 8;
        
        for(int i = 0; i < 8; i++) {
            scenarios[i].gps.latitude = 42.3601;
            scenarios[i].gps.longitude = -83.0732;
            scenarios[i].gps.altitude = 177.0;
            scenarios[i].gps.speed = 0.0;
            scenarios[i].gps.course = i * 45.0;  // 0, 45, 90, 135, 180, 225, 270, 315
            scenarios[i].gps.valid = true;
            
            scenarios[i].compass.heading = i * 45.0;
            scenarios[i].compass.x_mag = 200.0 * cos(deg2rad(i * 45.0));
            scenarios[i].compass.y_mag = 200.0 * sin(deg2rad(i * 45.0));
            scenarios[i].compass.z_mag = -180.0;
            scenarios[i].compass.valid = true;
            
            scenarios[i].stepDuration_ms = 5000;  // 5 seconds per step
        }
    }
    
    static float deg2rad(float deg) {
        return deg * (M_PI / 180.0);
    }
    
    static float rad2deg(float rad) {
        return rad * (180.0 / M_PI);
    }
    
    // Control functions
    void enableSimulation(int mode) {
        currentMode = mode;
        simulationEnabled = true;
        simulationStartTime = millis();
        lastUpdateTime = millis();
        lastLogTime = millis();
        currentWaypoint = 0;
        currentStep = 0;
        circleStartTime = millis();
        randomWalkLastUpdate = millis();
        
        // Initialize starting position based on mode
        if (mode == SIM_MODE_WAYPOINT && waypointCount > 0) {
            currentLat = waypoints[0].latitude;
            currentLon = waypoints[0].longitude;
            currentHeading = 0.0;
            if (waypointCount > 1) {
                targetLat = waypoints[1].latitude;
                targetLon = waypoints[1].longitude;
                currentWaypoint = 1;
            }
        } else {
            currentLat = staticGPS.latitude;
            currentLon = staticGPS.longitude;
            currentHeading = staticCompass.heading;
        }
        
        // Enable logging and create filename
        logSimulationData = true;
        createSimLogFilename();
        initializeSimLogFile();
        
        Serial.printlnf("Simulation enabled, mode: %d", mode);
    }
    
    void disableSimulation() {
        simulationEnabled = false;
        logSimulationData = false;
        currentMode = SIM_MODE_DISABLED;
        Serial.println("Simulation disabled");
    }
    
    bool isSimulationEnabled() const {
        return simulationEnabled;
    }
    
    int getSimulationMode() const {
        return currentMode;
    }
    
    // Get simulated GPS data based on current mode
    GPSSimData getGPSData() {
        if (!simulationEnabled) {
            GPSSimData invalid = {0};
            invalid.valid = false;
            return invalid;
        }
        
        uint32_t currentTime = millis();
        
        switch(currentMode) {
            case SIM_MODE_STATIC:
                return getStaticGPS();
                
            case SIM_MODE_WAYPOINT:
                return getWaypointGPS(currentTime);
                
            case SIM_MODE_CIRCLE:
                return getCircleGPS(currentTime);
                
            case SIM_MODE_RANDOM_WALK:
                return getRandomWalkGPS(currentTime);
                
            default:
                return getStaticGPS();
        }
    }
    
    // Get simulated compass data based on current mode
    CompassSimData getCompassData() {
        if (!simulationEnabled) {
            CompassSimData invalid = {0};
            invalid.valid = false;
            return invalid;
        }
        
        uint32_t currentTime = millis();
        
        switch(currentMode) {
            case SIM_MODE_STATIC:
                return getStaticCompass();
                
            case SIM_MODE_WAYPOINT:
                return getWaypointCompass(currentTime);
                
            case SIM_MODE_CIRCLE:
                return getCircleCompass(currentTime);
                
            case SIM_MODE_RANDOM_WALK:
                return getRandomWalkCompass(currentTime);
                
            default:
                return getStaticCompass();
        }
    }
    // Update simulation and log data if needed
    void updateSimulation() {
        if (!simulationEnabled) return;
        
        uint32_t currentTime = millis();
        
        // Update position and heading based on mode
        if (currentTime - lastUpdateTime >= SIM_UPDATE_INTERVAL_MS) {
            updateSimulationStep(currentTime);
            lastUpdateTime = currentTime;
        }
        
        // Log data if interval has passed
        if (logSimulationData && (currentTime - lastLogTime >= SIM_LOG_INTERVAL_MS)) {
            logSimulationStep();
            lastLogTime = currentTime;
        }
        
        // Check if sequence is complete (5 minutes)
        if (currentTime - simulationStartTime >= SIM_SEQUENCE_DURATION_MS) {
            if (currentMode == SIM_MODE_WAYPOINT) {
                // Reset waypoint sequence
                simulationStartTime = currentTime;
                currentWaypoint = 0;
                if (waypointCount > 0) {
                    currentLat = waypoints[0].latitude;
                    currentLon = waypoints[0].longitude;
                    if (waypointCount > 1) {
                        targetLat = waypoints[1].latitude;
                        targetLon = waypoints[1].longitude;
                        currentWaypoint = 1;
                    }
                }
                Serial.println("Simulation sequence completed, restarting waypoint cycle");
            }
        }
    }

private:
    GPSSimData getStaticGPS() {
        staticGPS.timestamp = millis();
        return staticGPS;
    }
    
    CompassSimData getStaticCompass() {
        staticCompass.timestamp = millis();
        return staticCompass;
    }
    
    GPSSimData getWaypointGPS(uint32_t currentTime) {
        if (waypointCount == 0) return getStaticGPS();
        
        GPSSimData data;
        data.latitude = currentLat;
        data.longitude = currentLon;
        data.altitude = 177.0;
        data.speed = SIM_VEHICLE_SPEED_MPS;
        data.course = currentHeading;
        data.valid = true;
        data.timestamp = currentTime;
        return data;
    }
    
    CompassSimData getWaypointCompass(uint32_t currentTime) {
        CompassSimData data;
        data.heading = currentHeading;
        data.x_mag = 200.0 * cos(deg2rad(currentHeading));
        data.y_mag = 200.0 * sin(deg2rad(currentHeading));
        data.z_mag = -180.0;
        data.valid = true;
        data.timestamp = currentTime;
        return data;
    }
    
    GPSSimData getCircleGPS(uint32_t currentTime) {
        float elapsed_s = (currentTime - circleStartTime) / 1000.0;
        float angle = (elapsed_s * circleSpeed_mps / circleRadius_m); // radians
        
        GPSSimData data;
        data.latitude = circleCenter_lat + (circleRadius_m / 111320.0) * cos(angle);
        data.longitude = circleCenter_lon + (circleRadius_m / (111320.0 * cos(deg2rad(circleCenter_lat)))) * sin(angle);
        data.altitude = 177.0;
        data.speed = circleSpeed_mps;
        data.course = rad2deg(angle + M_PI/2); // Tangent to circle
        if (data.course < 0) data.course += 360.0;
        if (data.course >= 360.0) data.course -= 360.0;
        data.valid = true;
        data.timestamp = currentTime;
        
        return data;
    }
    
    CompassSimData getCircleCompass(uint32_t currentTime) {
        float elapsed_s = (currentTime - circleStartTime) / 1000.0;
        float angle = (elapsed_s * circleSpeed_mps / circleRadius_m);
        float heading = rad2deg(angle + M_PI/2); // Tangent to circle
        if (heading < 0) heading += 360.0;
        if (heading >= 360.0) heading -= 360.0;
        
        CompassSimData data;
        data.heading = heading;
        data.x_mag = 200.0 * cos(deg2rad(heading));
        data.y_mag = 200.0 * sin(deg2rad(heading));
        data.z_mag = -180.0;
        data.valid = true;
        data.timestamp = currentTime;
        
        return data;
    }
    
    GPSSimData getRandomWalkGPS(uint32_t currentTime) {
        // Update position every 2 seconds
        if (currentTime - randomWalkLastUpdate > 2000) {
            randomWalkLastUpdate = currentTime;
            
            // Random heading change (-30 to +30 degrees)
            float headingChange = (random(-30, 31));
            randomWalk_heading += headingChange;
            if (randomWalk_heading < 0) randomWalk_heading += 360.0;
            if (randomWalk_heading >= 360.0) randomWalk_heading -= 360.0;
            
            // Move 2-5 meters in current direction
            float distance_m = random(20, 51) / 10.0; // 2.0 to 5.0 meters
            float headingRad = deg2rad(randomWalk_heading);
            
            randomWalk_lat += (distance_m / 111320.0) * cos(headingRad);
            randomWalk_lon += (distance_m / (111320.0 * cos(deg2rad(randomWalk_lat)))) * sin(headingRad);
        }
        
        GPSSimData data;
        data.latitude = randomWalk_lat;
        data.longitude = randomWalk_lon;
        data.altitude = 177.0;
        data.speed = 1.5; // m/s
        data.course = randomWalk_heading;
        data.valid = true;
        data.timestamp = currentTime;
        
        return data;
    }
    
    CompassSimData getRandomWalkCompass(uint32_t currentTime) {
        CompassSimData data;
        data.heading = randomWalk_heading;
        data.x_mag = 200.0 * cos(deg2rad(randomWalk_heading));
        data.y_mag = 200.0 * sin(deg2rad(randomWalk_heading));
        data.z_mag = -180.0;
        data.valid = true;
        data.timestamp = currentTime;
        
        return data;
    }
    
    // New helper methods for enhanced simulation
    void updateSimulationStep(uint32_t currentTime) {
        if (currentMode == SIM_MODE_WAYPOINT && waypointCount > 1) {
            updateWaypointProgress(currentTime);
        }
    }
    
    void updateWaypointProgress(uint32_t currentTime) {
        // Calculate distance to current target
        float distance = calculateDistance(currentLat, targetLat, currentLon, targetLon);
        
        // If close to target, move to next waypoint
        if (distance < 5.0) { // 5 meter threshold
            currentWaypoint = (currentWaypoint + 1) % waypointCount;
            targetLat = waypoints[currentWaypoint].latitude;
            targetLon = waypoints[currentWaypoint].longitude;
            
            // Calculate new heading
            float deltaLat = targetLat - currentLat;
            float deltaLon = targetLon - currentLon;
            targetHeading = rad2deg(atan2(deltaLon, deltaLat));
            if (targetHeading < 0) targetHeading += 360.0;
        }
        
        // Move towards target at realistic speed
        float deltaTime = (currentTime - lastUpdateTime) / 1000.0; // seconds
        float moveDistance = SIM_VEHICLE_SPEED_MPS * deltaTime; // meters
        
        if (distance > 0.1) { // Avoid division by zero
            float moveRatio = moveDistance / distance;
            if (moveRatio > 1.0) moveRatio = 1.0; // Don't overshoot
            
            currentLat += (targetLat - currentLat) * moveRatio;
            currentLon += (targetLon - currentLon) * moveRatio;
        }
        
        // Smoothly adjust heading
        float headingDiff = targetHeading - currentHeading;
        if (headingDiff > 180.0) headingDiff -= 360.0;
        if (headingDiff < -180.0) headingDiff += 360.0;
        
        float maxHeadingChange = 45.0 * deltaTime; // 45 degrees per second max
        if (abs(headingDiff) <= maxHeadingChange) {
            currentHeading = targetHeading;
        } else {
            currentHeading += (headingDiff > 0 ? maxHeadingChange : -maxHeadingChange);
        }
        
        if (currentHeading < 0) currentHeading += 360.0;
        if (currentHeading >= 360.0) currentHeading -= 360.0;
    }
    
    float calculateDistance(float lat1, float lat2, float lon1, float lon2) {
        float dLat = deg2rad(lat1 - lat2);
        float dLon = deg2rad(lon1 - lon2);
        float a = sin(dLat/2) * sin(dLat/2) + cos(deg2rad(lat2)) * cos(deg2rad(lat1)) * sin(dLon/2) * sin(dLon/2);
        float c = 2 * atan2(sqrt(a), sqrt(1.0-a));
        return 6371000.0 * c; // Distance in meters
    }
    
    void createSimLogFilename() {
        // Create filename with timestamp
        char timestamp[16];
        snprintf(timestamp, 16, "%02d%02d%04d%02d%02d%02d", 
                Time.month(), Time.day(), Time.year(), 
                Time.hour(), Time.minute(), Time.second());
        snprintf(simLogFilename, 40, "SimData%s.csv", timestamp);
    }
    
    void initializeSimLogFile() {
        // This will be called from main code where SD card access is available
        // The actual implementation is in the main .ino file
    }
    
    void logSimulationStep() {
        // Call the external logging function implemented in main code
        // This avoids circular dependencies and SD card access issues
        extern void logSimulationData();
        logSimulationData();
    }
};

// Global simulation data instance
extern SimulationData simulationData;

#endif // SIMULATION_DATA_H
