#include "VehicleSimulator.h"
#include "Particle.h"

bool VehicleSimulator::begin() {
    return true;            // Not actually using hardware, so we will tell the caller it succeeded
}

bool VehicleSimulator::isConnected() {
    return true;            // Not actually using hardware, so we will tell the caller it succeeded
}

bool VehicleSimulator::readMagnetometer() {
    return true;            // Not actually using hardware, so we will tell the caller it succeeded
}

float VehicleSimulator::getCompassHeading() {
    return (float)(currentHeading + compassOffset);
}

float VehicleSimulator::getLatitude() {
    return (float)currentLatitude;
}

float VehicleSimulator::getLongitude() {
    return (float)currentLongitude;
}

double VehicleSimulator::getTargetLatitude() {
    return targetLatitude; // Return the target latitude
}

double VehicleSimulator::getTargetLongitude() {
    return targetLongitude; // Return the target longitude
}

void VehicleSimulator::updateCompassOffset(int offset) {
    compassOffset = offset; // Update the compass offset
}

const char* VehicleSimulator::getType() {
    return "Simulator"; // Placeholder for actual implementation
}

/// @brief Bread and butter function that updates compass/GPS simulation based on the resulting motor speed the control system calculated
/// @param leftSpeed 
/// @param rightSpeed 
void VehicleSimulator::updateSimulationKinematics(int leftSpeed, int rightSpeed) {
    //Only going to approximate straight lines to start out. Need to add a circular approximation later for when motor speeds don't  match
    const double maxVehicleSpeed = 2;          //Maximum vehicle speed in m/s
    double averageMotorSpeed = ((double)(leftMotorSpeed + rightMotorSpeed) / 2.0) - 90;    // Get average of motor speeds relative to forward (will range -90 to 90)
    double targetSpeed = maxVehicleSpeed * averageMotorSpeed / 90.0;            // 90 = full speed forwards, -90 = full speed backwards. Take as a linear fraction of that.

    double timeDeltaSeconds = (double)(millis() - simulationTimer) / 1000.0;     //Calculate how much time has elapsed since the simulation was last updated
    simulationTimer = millis();

    double headingRad = currentHeading * 3.14159265358979323846 / 180.0;
    double metersNorth = targetSpeed * timeDeltaSeconds * cos(headingRad);
    double metersEast = targetSpeed * timeDeltaSeconds * sin(headingRad);

    double deltaLat = metersToDeltaLatitude(metersNorth);
    double deltaLon = metersToDeltaLongitude(metersEast, currentLatitude);

    currentLatitude += deltaLat;
    currentLongitude += deltaLon;


    // Estimation for the compass behavior

    // For each difference of 1 in the motor speed, how much does the compass rotate (in degrees) per second. 
    // If the motor speeds were 135L and 115R, then the difference is 20. The compass would rotate at 20 * compassMotorDeltaPerSecond degrees per second
    const double compassMotorDeltaPerSecond = 0.3;     

    // Update the compass heading based on the difference between left and right motor speed.
    // This is like a polygon-with-n-sides approximation of the direction of the water vehicle
    double motorDelta = leftMotorSpeed - rightMotorSpeed;
    double headingDelta = timeDeltaSeconds * compassMotorDeltaPerSecond * motorDelta;
    currentHeading += headingDelta;
    if(currentHeading < -180.0) currentHeading += 360.0;
    if(currentHeading > 180.0) currentHeading -= 360.0;

    // Update these last since the vehicle has been operating at the previous speed for the current elapsed time range
    leftMotorSpeed = leftSpeed; // Update left motor speed
    rightMotorSpeed = rightSpeed; // Update right motor speed
}

/// @brief Converts a distance in meters to a change in latitude in degrees
/// @param meters Distance north/south in meters
/// @return Change in latitude in degrees
double VehicleSimulator::metersToDeltaLatitude(double meters) {
    // 1 degree latitude is approximately 111,320 meters
    return meters / 111320.0;
}

double VehicleSimulator::latitudeToDeltaMeters(double latitude){
    // 1 degree latitude is approximately 111,320 meters
    return latitude * 111320.0;
}

/// @brief Converts a distance in meters to a change in longitude in degrees, at a given latitude
/// @param meters Distance east/west in meters
/// @param latitude Current latitude in degrees
/// @return Change in longitude in degrees
double VehicleSimulator::metersToDeltaLongitude(double meters, double latitude) {
    // 1 degree longitude = 111,320 * cos(latitude) meters
    double latRad = latitude * 3.14159265358979323846 / 180.0;
    double metersPerDegree = 111320.0 * cos(latRad);
    if (metersPerDegree == 0.0) return 0.0; // Avoid division by zero at the poles
    return meters / metersPerDegree;
}

void VehicleSimulator::disableSimulation() {
    simulationActive = false; // Disable the simulation
}

bool VehicleSimulator::isSimulationActive() {
    return simulationActive; // Return the current state of the simulation
}

void VehicleSimulator::setScenario(int scenarioId) {
    // Logic to set scenario by ID
    // This could involve setting different waypoints or behaviors based on the scenario
    if (scenarioId == 0){
        simulationActive = false; // Disable simulation if scenario ID is 0
    }

    if (scenarioId == 1) {
        simulationActive = true; // Enable simulation for scenario 1
        setScenario1(); // Call specific scenario setup
    }
    else if (scenarioId == 2) {
        simulationActive = true; // Enable simulation for scenario 2
        setScenario2(); // Call specific scenario setup
    }

    // Additional scenarios can be added here
}

void VehicleSimulator::setScenario1() {
    // Logic to set scenario 1, e.g., vertical travel to one waypoint
    currentLatitude = 35.766605;       // Starting latitude on the dock of Lake Raleigh
    currentLongitude = -78.677878;     // Starting longitude on the dock of Lake Raleigh
    currentHeading = 180.0;            // Start the vehicle facing due South

    targetLatitude = 35.765705;        // Destination latitude of the bottom of Lake Raleigh, due south of the dock
    targetLongitude = -78.677878;      // Destination latitude of the bottom of Lake Raleigh, no change in longitude
    
    simulationTimer = millis();         // Update the simulation timer since we just started the simulation
}

void VehicleSimulator::setScenario2() {
    // Logic to set scenario 1, e.g., vertical travel to one waypoint
    currentLatitude = 35.766605;       // Starting latitude on the dock of Lake Raleigh
    currentLongitude = -78.677878;     // Starting longitude on the dock of Lake Raleigh
    currentHeading = -125.4;           // Start the vehicle facing South West

    targetLatitude = 35.765705;        // Destination latitude of the bottom of Lake Raleigh, southwest of the dock
    targetLongitude = -78.679000;
    
    simulationTimer = millis();         // Update the simulation timer since we just started the simulation
}

