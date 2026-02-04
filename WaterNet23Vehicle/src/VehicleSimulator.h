#include "GPSBase.h"
#include "CompassBase.h"
#include "stdint.h"

class VehicleSimulator : public CompassBase, public GPSBase{
public:
    // Constructor
    VehicleSimulator() {}

    // Implement pure virtual functions from CompassBase
    bool begin();
    bool readMagnetometer();
    float getCompassHeading();
    void updateCompassOffset(int offset);

    // Implement pure virtual functions from GPSBase
    bool isConnected();
    float getLatitude();
    float getLongitude();
    const char* getType();

    // Other methods specific to the vehicle simulator
    double getTargetLatitude();
    double getTargetLongitude();

    // Methods to update the state of the simulator
    void disableSimulation(); // Disable the simulation
    bool isSimulationActive();
    void updateSimulationKinematics(int leftSpeed, int rightSpeed);

    // Methods to define simulation scenarios
    void setScenario(int scenarioId);       // Set a scenario by ID. Setting to a non-zero ID enables simulation
    void setScenario1();                    // Scenario with vertical travel to one waypoint
    void setScenario2();                    // Scenario with diagonal travel to one waypoint

private:
    bool simulationActive = false;          // Flag to indicate if simulation is active
    int compassOffset = 0;                  // Offset for compass heading, can be set externally
    double currentLatitude = 0.0;           // Current latitude of the simulator
    double currentLongitude = 0.0;          // Current longitude of the simulator
    double currentHeading = 0.0;            // Current heading of the emulated compass
    double targetLatitude = 0.0;            // Target latitude for the simulator
    double targetLongitude = 0.0;           // Target longitude for the simulator
    uint32_t simulationTimer = 0;           // Counter for when the simulation was last updated

    // Additional private members for vehicle state, if needed
    uint8_t leftMotorSpeed = 0;
    uint8_t rightMotorSpeed = 0;

    // Conversion helpers
    double metersToDeltaLatitude(double meters);
    double metersToDeltaLongitude(double meters, double latitude);
    double latitudeToDeltaMeters(double latitude);
};