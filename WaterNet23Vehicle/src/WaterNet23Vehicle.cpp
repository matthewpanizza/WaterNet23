/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/matthewpanizza/Library/CloudStorage/OneDrive-Personal/Particle/WaterNet23-GY511/WaterNet23Vehicle/src/WaterNet23Vehicle.ino"
/*
 * Project WaterNet23PreAlpha
 * Description: Initial code for B404 with GPS and serial communications
 * Date: 3/18/2022
 */
//////////////////////////////////////////////////////////////
//                Main Software Architecture                //
//                                                          //
//    Read these functions to understand code flow          //
//    -- setup(): Initializes hardware, timers, counters    //
//    -- loop(): Main body, contiunously executes           //
//    -- processCommand(): command dictionary from comms    //
//                                                          //
//  Timers: read these to understand asynchronous activity  //
//                                                          //
//////////////////////////////////////////////////////////////

#include "application.h"                    //Needed for I2C to GPS
void processCommand(const char *command, uint8_t mode, bool sendAck);
void handleControlCommand(const char* dataStr, uint8_t mode);
void handleStatusDelayPeriodCommand(const char* dataStr, uint8_t mode);
void handleMotorCommand(const char* dataStr, uint8_t mode);
void handleDataRequest(const char* dataStr, uint8_t mode);
void handlePrintString(const char* dataStr, uint8_t mode);
void handleStatusCommand(const char* dataStr, uint8_t mode);
void handleHelloAck(const char* dataStr, uint8_t mode);
void handleDumpMode(const char* dataStr, uint8_t mode);
void handleCompassCal(const char* dataStr, uint8_t mode);
void handleEmulatedGPS(const char* dataStr, uint8_t mode);
void handleStopCommand(const char* dataStr, uint8_t mode);
void handleEKFCommand(const char* dataStr, uint8_t mode);
void handleCompassCommand(const char* dataStr, uint8_t mode);
void handleSimulationCommand(const char* dataStr, uint8_t mode);
void handleHelpCommand(const char* dataStr, uint8_t mode);
void handleTableCommand(const char* dataStr, uint8_t mode);
void handleTareCommand(const char* dataStr, uint8_t mode);
void initializeLogFiles();
void updateSimulationData();
void logSimulationData();
void writeMotionDataLog();
void processQueuedCommands();
void processQueuedDebugMessages();
void cmdLTEHandler(const char *event, const char *data);
void setup();
void loop();
void setupPins();
void setupBLE();
void readEEPROM();
void writeEEPROM();
void setupXBee();
bool setupNavigationSensors();
void compassCalibration();
uint8_t readPowerSys();
float deg2rad(float deg);
float lis3mdlCompassHeading(float x_accel, float y_accel);
float calcDistance(float lat1, float lat2, float lon1, float lon2);
float calcDelta(float compassHead, float targetHead);
float getRawCompassHeading();
float getCalibratedCompassHeading();
void getPositionData();
void sendResponseData();
void statusUpdate();
float averageAndAdvanceCircularBuffer(float* arr, int len, int *idx, float newVal);
void updateMotors();
void sendData(const char *dataOut, uint8_t sendMode, bool sendBLE, bool sendXBee, bool sendLTE);
void printBLE(const char *dataOut);
void StatusHandler();
void sensorHandler();
void XBeeHandler();
void SerialConsoleHandler();
static void BLEDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void motionHandler();
void wdogHandler();
void dataOffloader();
void buttonActionDecode();
void buttonHandler();
void LEDHandler();
int LTEInputCommand(String cmd);
void printStatusTable();
#line 19 "/Users/matthewpanizza/Library/CloudStorage/OneDrive-Personal/Particle/WaterNet23-GY511/WaterNet23Vehicle/src/WaterNet23Vehicle.ino"
#define ARDUINO 0
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include <Adafruit_Sensor.h>
#define X_AXIS_ACCELERATION 0
//#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
//#include <MicroNMEA.h>                      //http://librarymanager/All#MicroNMEA
#include "SdFat.h"
#include "sdios.h"
#undef min
#undef max
#include <vector>
#include <stdarg.h>
#include "CompassEKF.h"
#include "CompassBase.h"
#include "GPSBase.h"
#include "LIS3MDLCompass.h"
#include "LSM303Compass.h"
#include "NeoM8UGPS.h"
#include "VehicleSimulator.h"
#include "CommandQueue.h"

#define COMPASS_TYPE_LSM303     0           //Value for COMPASS_TYPE to indicate LSM303DLHC    
#define COMPASS_TYPE_LIS3MDL    1           //Value for COMPASS_TYPE to indicate LIS3MDL
#define COMPASS_TYPE_AUTO       2           //Value for COMPASS_TYPE to auto-detect compass

#define COMPASS_TYPE            COMPASS_TYPE_AUTO           //0 = LSM303DLHC, 1 = LIS3MDL


SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

///////////////////////
// BLE Configuration //
///////////////////////

const char* WaterNetService = "b4206910-dc4b-5743-c8b1-92d0e75182b0"; //Main BLE Service
const char* rxUuid          = "b4206912-dc4b-5743-c8b1-92d0e75182b0"; //GPS Latitude Service
const char* txUuid          = "b4206913-dc4b-5743-c8b1-92d0e75182b0"; //GPS Longitude Service
const char* offldUuid       = "b4206914-dc4b-5743-c8b1-92d0e75182b0"; //GPS Longitude Service

BleCharacteristic txCharacteristic("tx", BleCharacteristicProperty::NOTIFY, txUuid, WaterNetService);
BleCharacteristic rxCharacteristic("rx", BleCharacteristicProperty::WRITE_WO_RSP, rxUuid, WaterNetService, BLEDataReceived, NULL);
BleCharacteristic offloadCharacteristic("off", BleCharacteristicProperty::NOTIFY, offldUuid, WaterNetService);

#ifdef BLE_DEBUG_ENABLED
    const char* bledbgUuid      = "b4206915-dc4b-5743-c8b1-92d0e75182b0"; //BLE Debug Console characteristic
    BleCharacteristic bledbgCharacteristic("dbg", BleCharacteristicProperty::NOTIFY, bledbgUuid, WaterNetService);
#endif

BleAdvertisingData advData;                         //Advertising data

uint8_t BLECustomData[CUSTOM_DATA_LEN];             //Byte array for custom data transmitted in the BLE advertising packet. Can contain up to 31 bytes in an advertisement

////////////////////
// Timer Objects //
///////////////////

Timer watchdog(WATCHDOG_PD, wdogHandler);           //Create timer for watchdog, which checks if certain methods of communication are available
Timer ledTimer(300,LEDHandler);                     //Create timer for LED, which updates the color of the LED based on what communication/hardware modes are available
Timer motionTimer(250, motionHandler);             //Create timer for motor watchdog, which cuts off motors if messages from CC have not been received recently enough
Timer motorHandler(MTR_RAMP_TIME,updateMotors);
Timer statusPD(STATUS_PD,StatusHandler);            //Create timer for status, which calculates the status values that will be transmitted to CC and sets a flag for transmitting out the status


//////////////////////////////
// Logging Global Variables //
//////////////////////////////

CommandQueue commandQueue;                      //Command queue for incoming commands from various sources. Helps with thread-safety and asynchronous command processing
CommandQueue debugQueue;                        //Command queue for debug messages to be logged to the uSD card. Helps with thread-safety and asynchronous command processing


VehicleSimulator* vehicleSim = nullptr;         //Pointer to the vehicle simulator object, if enabled
CompassBase* compass = nullptr;                 //Unified compass pointer using abstract base class
GPSBase* gps = nullptr;                         //Unified gps pointer using abstract base class

LEDStatus status;                               //LED Control object

//////////////////////////////
// Logging Global Variables //
//////////////////////////////

// uSD controller class
SdFat sd((SPIClass*)&SPI);                      //SD card object, initialized on SPI for the Beta/Alpha PCB, SPI1 on the Bsom breakout board

// Filename strings
char filename[MAX_FILENAME_LEN];                //Filename for the file holding sensor data
char logFileName[MAX_FILENAME_LEN];        //Filename for the file holding log messages
char simFilename[MAX_FILENAME_LEN];             //Filename for the simulation data log file
char autFilename[MAX_FILENAME_LEN];             //Filename for the autonomous navigation data log file

// Data log files
File dataFile;                                  //File for the sensor data
File logFile;                                   //File for messages logged by the program
File simLogFile;                                //File for simulation data logging
File autLogFile;                                //File for autonomous navigation data logging
File logDir;                                    //File directory 

bool SDAvail;    
bool logSensors = true; 
bool logMessages = true;                                 //Flags for sensor timing/enables
bool statusTableEnabled = true;                                                //Flag to control status table printing

SerialLogHandler logHandler(LOG_LEVEL_INFO);    //Log Configuration


////////////////////////////////////
// Motor Control Global Variables //
////////////////////////////////////

// Instances of the servo class for controlling the ESCs
Servo ESCL;                                                             //Object for servo esc of left motor 
Servo ESCR;                                                             //Object for servo esc of right motor

uint8_t leftMotorSpeed, leftMotorSpeedSetpoint;                         //Global for the left motor speed and left motor target speed
uint8_t rightMotorSpeed, rightMotorSpeedSetpoint;                       //Global for the right motor speed and right motor target speed

bool stopActive = false;                                                //Flag to indicate that the CChub has had a stop hit
uint32_t lastMotorCommandTime;                                          //Timers for when the last motor control command was received


////////////////////////////////////
//   Navigation Global Variables  //
////////////////////////////////////

bool usingVehicleSim = false;                                           //Flag to indicate that the vehicle simulator is being used instead of real GPS/compass
float latitude, longitude;                                              //Globals to hold the latitude and longitude read in from the GPS
float compassHeading; 
float travelHeading, headingDelta;                                      //Compassheading is the calibrated compass reading relative to north, travel heading is the heading between current point and target point
float targetLat, targetLon;                                             //Globals to hold the latitude and longitude sent from the CC for where the bot should target
float travelDistance;                                                   //Global to hold the distance between the current latitude and longitude and the target latitude and longitude
bool telemetryAvail = false;                                            //Boolean global to check if the compass and GPS are available
bool waypointArrived = false;                                           //Flag to indicate that the waypoint has been reached
uint16_t waypointIndex = 0;                                             //Index of the waypoint this bot is currently targeting

double varCompassHead;
int compOffset = COMP_OFFSET;                                           //Offset for the compass calibration. Degrees off north to add to the compass reading to calibrate it to true north
bool doCompassCal = false;                                              //Flag to indicate that the compass calibration has been requested
bool GPSAvail = false;                                                  //Flag to indicate that the GPS is available
bool CompassAvail = false;                                              //Flag to indicate that the compass is available

uint16_t motorTare = 100;                                               //Tare value for the motors, used to calibrate the ESCs when one motor is faster than the other

uint32_t lastTelemTime = 0;
uint32_t lastCalibrationTime = 0;                                       //Timer for when the last compass calibration occurred 
uint32_t lastAutonomousLogTime = 0;                                     //Timer for when the last autonomous navigation data was logged

uint8_t driveMode = DRIVE_MODE_MANUAL;                                  //Global mode for drive mode, 0 = manual remote control, 1 = sentry, 2 = autonomous

// EKF variables for improved compass heading
CompassEKF compassEKF;                                                  //Extended Kalman Filter for compass heading fusion
float filteredCompassHeading = 0.0;                                     //EKF-filtered compass heading
float lastEKFUpdateTime = 0.0;                                          //Last time EKF was updated
bool useEKF = false;                                                     //Flag to enable/disable EKF filtering

                             


////////////////////////////////////
// Communication Global Variables //
////////////////////////////////////

bool waitForConnection;                                                 //Flag used on startup until the CC acknowledges this bot
bool LTEAvail = false;                                                  //Flag to indicate that LTE is available
bool XBeeAvail = false;                                                 //Flag to indicate that XBee is available
bool BLEAvail = false;                                                  //Flag to indicate that BLE is available
uint32_t XBeeRxTime = 0, BLERxTime = 0;                                 //Timers for when the last valid Xbee and BLE message was received
bool statusReady = false;                                               //Flag to indicate that a status is ready
uint8_t requestActive = 0;                                              //Flag to indicate that a sensor request has been made from the CChub
uint16_t LTEStatusCount = LTE_MAX_STATUS;                               //Counter to determine number of LTE messages that should be sent to limit data usage
uint16_t statusFlags = 0;                                               //Global status flag
uint32_t lastStatusTime = 0;                                            //Timer for when the last status control packet was received


////////////////////////////////////
// Power System Global Variables  //
////////////////////////////////////

bool warnedBattLeak = false; 
bool warnedLeak = false;                                                //Flags to indicate a leak warning has been published to avoid publishing too frequently
uint8_t battPercent;                                                    //Global battery percentage indicator
float battVoltage, battCurrent, solarCurrent;                           //Global battery and solar current read in from power system
bool lowBattery;                                                        //Flag to indicate that the battery is low


////////////////////////////////////
// Sensor System Global Variables //
////////////////////////////////////

float sensePH, senseTemp, senseCond, senseMCond, senseDO;               //Global variables for holding sensor data received from last Atlas sensors
bool dataWait = false;                                                //Flag to indicate that the Atlas sensors are being read and data is being waited on


////////////////////////////////////
// Miscellaneous Global Variables //
////////////////////////////////////

bool offloadMode = false;                                               //Flag to indicate that SD card data is being offloaded
bool signalLED;                                                         //Flag to indicate that the CC hub has requested that the LED should be signaling flashing orange
uint32_t senseTimer = 0;                                                //Timer for when the last time data was requested from the Atlas sensors
uint32_t dataTimer = 0;                                                 //Timer for when the last time data was read from the Atlas sensors
uint32_t positionTimer = 0;                                             //Timer for when the last GPS reading was taken
uint32_t compassTimer = 0;                                              //Timers for when the last compass reading was taken
uint32_t lastButtonClickTime;                                           //Timer for when the last button click was received
bool buttonPressSequenceStarted = false;                                //Flag to indicate that the button press sequence has started
uint16_t buttonPressCount = 0;                                          //Counter for the number of repeated button presses



// Command lookup table for cleaner command processing
const CommandEntry commandTable[] = {
    {"ctl", handleControlCommand},
    {"mtr", handleMotorCommand},
    {"tar", handleTareCommand},
    {"req", handleDataRequest},
    {"pts", handlePrintString},
    {"spc", handleStatusCommand},
    {"hwa", handleHelloAck},
    {"dmp", handleDumpMode},
    {"cmp", handleCompassCal},
    {"egp", handleEmulatedGPS},
    {"stp", handleStopCommand},
    {"ekf", handleEKFCommand},
    {"cms", handleCompassCommand},
    {"sim", handleSimulationCommand},
    {"tbl", handleTableCommand},
    {"sdp", handleStatusDelayPeriodCommand},
    {"hlp", handleHelpCommand}
};
const int commandTableSize = sizeof(commandTable) / sizeof(CommandEntry);

// Helper function to log to logFile with printf-style arguments (no vprintf in File, so use buffer)
/**
 * @brief Helper function to log messages to debug file with printf-style formatting
 * @param fmt Format string (printf-style)
 * @param ... Variable arguments for format string
 */
void logToDebugFile(const char* fmt, ...) {
    if(!SDAvail || !logMessages) return; // Do not log if SD card is not available
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if(!logFile.isOpen()) {
        logFile.open(logFileName, O_RDWR | O_CREAT | O_AT_END);
        logFile.println(buf);
        logFile.close();
    } else {
        logFile.println(buf);
    }
}

/**
 * @brief Dictionary for all bot commands that is called when XBee, BLE, and LTE strings are received. Mode 1 - BLE, Mode 2 - XBEE, Mode 4 - LTE
 * @param command The command string to process
 * @param mode Communication mode: 1 = BLE, 2 = XBEE, 4 = LTE
 * @param sendAck Whether to send acknowledgment back to sender
 */
void processCommand(const char *command, uint8_t mode, bool sendAck){
    const char* modeStr = "UNKNOWN";
    if (mode == 1) modeStr = "BLE";
    else if (mode == 2) modeStr = "XBEE";
    else if (mode == 4) modeStr = "LTE";
    logToDebugFile("[INFO] Received Message (%s): %s", modeStr, command);
    //Process if command is addressed to this bot "Bx" or all bots "AB"
    if((command[2] == 'B' && command[3] == BOTNUM+48) || (command[2] == 'A' && command[3] == 'B')){
        
        // Validate command length
        int cmdLen = strlen(command);
        if(cmdLen < 8) {
            Serial.println("Warning: Command too short");
            return;
        }
        
        // Extract and validate checksum
        char checkStr[3] = {command[cmdLen-2], command[cmdLen-1], '\0'};
        uint8_t checksum = (uint8_t)strtol(checkStr, NULL, 16);
        
        if(checksum != cmdLen-2) {
            #ifdef VERBOSE
            Serial.printlnf("String Len: %d, Checksum: %d", cmdLen-2, checksum);
            Serial.println("Warning, checksum does not match");
            #endif
            logToDebugFile("[WARN] Message Checksum Does Not Match!: %s", command);
            return;
        }
        
        // Extract command string (3 characters)
        char cmdStr[4];
        strncpy(cmdStr, &command[4], 3);
        cmdStr[3] = '\0';
        
        // Extract data string (everything between command and checksum)
        int dataLen = cmdLen - 9; // Total length - address(4) - cmd(3) - checksum(2)
        char dataStr[dataLen + 1];
        if(dataLen > 0) {
            strncpy(dataStr, &command[7], dataLen);
        }
        dataStr[dataLen] = '\0';
        
        #ifdef VERBOSE
        Serial.printlnf("Command: %s, Data: %s, Checksum: %02x", cmdStr, dataStr, checksum);
        #endif
        
        // Look up command in table and execute handler
        bool commandFound = false;
        for(int i = 0; i < commandTableSize; i++) {
            if(!strcmp(cmdStr, commandTable[i].cmd)) {
                commandTable[i].handler(dataStr, mode);
                commandFound = true;
                break;
            }
        }
        
        if(!commandFound) {
            Serial.printlnf("Warning: Unknown command '%s'", cmdStr);
            logToDebugFile("[WARN] Unknown command received: %s", cmdStr);
        }
    }
}

// Individual command handler functions for cleaner organization
/**
 * @brief Control command from CC that contains data about the drive mode, target latitude and longitude, and offloading
 * @param dataStr The data string containing control parameters
 * @param mode Communication mode the command was received from
 */
void handleControlCommand(const char* dataStr, uint8_t mode) {
    //Control command from CC that contains data about the drive mode, target latitude and longitude, and offloading
    char tLat[10];              //String buffer for latitude, as sscanf doesn't handle floats well
    char tLon[10];              //String buffer for longitude, as sscanf doesn't handle floats well
    uint8_t lastDriveMode = driveMode;  //Store the last drive mode to check if it has changed
    static float lastTargetLat = -1000.0f, lastTargetLon = -1000.0f; //Store the last target lat and lon to check if they have changed

    // Use temporary variables with correct types for sscanf
    unsigned int tempDriveMode, tempWaypointIndex;
    int tempLogSensors, tempSignalLED;
    sscanf(dataStr,"%s %s %u %d %d %u",
        tLat,tLon,
        &tempDriveMode,
        &tempLogSensors,
        &tempSignalLED,
        &tempWaypointIndex);
    
    // Assign to actual variables
    driveMode = (uint8_t)tempDriveMode;
    logSensors = (bool)tempLogSensors;
    signalLED = (bool)tempSignalLED;
    waypointIndex = (uint16_t)tempWaypointIndex;
    
    if(lastDriveMode != DRIVE_MODE_SENTRY && driveMode == DRIVE_MODE_SENTRY) {
        //If the drive mode has changed to sentry, reset the target lat and lon
        targetLat = latitude;  //Set target lat to current lat
        targetLon = longitude; //Set target lon to current lon
    }
    else if(driveMode == DRIVE_MODE_AUTONOMOUS || driveMode == DRIVE_MODE_MANUAL) {
        //If the drive mode is autonomous, set the targets to the values received from the CC hub
        targetLat = 35.766191f;//atof(tLat);     //Convert latitude string to float
        targetLon = -78.677869f;//atof(tLon);     //Convert longitude string to float
    }

    if(targetLat != lastTargetLat || targetLon != lastTargetLon) {
        //If the target lat or lon has changed, reset the waypointArrived flag
        waypointArrived = false; //Clear waypoint arrived flag
        lastTargetLat = targetLat;
        lastTargetLon = targetLon;
    }
    #ifdef VERBOSE
    Serial.printlnf("New target GPS, Lat: %f Lon: %f", targetLat, targetLon);
    #endif
}

/**
 * @brief Handler for the 'sdp' (status delay period) command.
 *
 * Allows dynamic adjustment of the status update period via command.
 * Accepts a value in seconds (1-10) and updates the status timer period accordingly.
 *
 * @param dataStr The data string containing the new status period in seconds (as a string).
 * @param mode Communication mode the command was received from (e.g., BLE, XBee, LTE).
 */
void handleStatusDelayPeriodCommand(const char* dataStr, uint8_t mode) {
    int seconds = atoi(dataStr);
    if (seconds < 1 || seconds > 10) {
        Serial.println("[SDP] Invalid status period. Must be 1-10 seconds.");
        logToDebugFile("[WARN] SDP: Invalid status period: %s", dataStr);
        return;
    }
    int newPeriod = seconds * 1000;
    statusPD.changePeriod(newPeriod);
    Serial.printlnf("[SDP] Status period changed to %d ms", newPeriod);
    logToDebugFile("[INFO] SDP: Status period changed to %d ms", newPeriod);
}

/**
 * @brief Motor Speed Control
 * @param dataStr The data string containing motor speed commands
 * @param mode Communication mode the command was received from
 */
void handleMotorCommand(const char* dataStr, uint8_t mode) {
    //Motor Speed Control
    if(strlen(dataStr) < 6) {
        Serial.println("Warning: Motor command data too short");
        return;
    }
    char lSpd[4] = {dataStr[0],dataStr[1],dataStr[2],'\0'};  //Get the first three characters of the data for the left target speed
    char rSpd[4] = {dataStr[3],dataStr[4],dataStr[5],'\0'};  //Get the second three characters of the data for the right target speed
    leftMotorSpeedSetpoint = atoi(lSpd);                             //Convert string to integer in global target speed, motor speed is ramped to new target by updateMotors
    rightMotorSpeedSetpoint = atoi(rSpd);                             //Convert string to integer in global target speed, motor speed is ramped to new target by updateMotors
    #ifdef VERBOSE
    Serial.printlnf("Received Motor Command: LSpeed=%d,RSpeed=%d",leftMotorSpeedSetpoint,rightMotorSpeedSetpoint);
    #endif
    lastMotorCommandTime = millis();         //Update timer for the watchdog that a motor speed was received from CC hub
    driveMode = DRIVE_MODE_MANUAL;  //In case we missed the switch from an autonomous to manual mode, switch to manual mode
}

/**
 * @brief Data Request from CChub to get the bundle of sensor data and transmit it out
 * @param dataStr The data string (unused for this command)
 * @param mode Communication mode the command was received from (used to determine response method)
 */
void handleDataRequest(const char* dataStr, uint8_t mode) {
    //Data Request from CChub to get the bundle of sensor data and transmit it out
    requestActive = mode;           //Set flag, as it's not possible to use 2/3 communication modes in an interrupt handler
}

/**
 * @brief Command used for debugging, which allows the CChub (or any bluetooth device) to print a string to the console and to the SD card
 * @param dataStr The string message to print and log
 * @param mode Communication mode the command was received from
 */
void handlePrintString(const char* dataStr, uint8_t mode) {
    //Command used for debugging, which allows the CChub (or any bluetooth device) to print a string to the console and to the SD card
    Serial.println(dataStr);        //Print to console
    logToDebugFile("[PUTS] Received String Command: %s", dataStr); //Log to SD card
}

/**
 * @brief Incoming communication status from CChub, this data is used in addition to control strings to determine which communication methods are available
 * @param dataStr The data string (unused for this command)
 * @param mode Communication mode the command was received from
 */
void handleStatusCommand(const char* dataStr, uint8_t mode) {
    //Incoming communication status from CChub, this data is used in addition to control strings to determine which communication methods are available
    lastStatusTime = millis();          //Update timer with the current time, and the watchdog will automatically set the flags based on this timer and the current time
}

/**
 * @brief Hello-world acknowledge command from the CCHub, which will bring this bot out of pairing mode on startup
 * @param dataStr The data string (unused for this command)
 * @param mode Communication mode the command was received from
 */
void handleHelloAck(const char* dataStr, uint8_t mode) {
    //Hello-world acknowledge command from the CCHub, which will bring this bot out of pairing mode on startup
    waitForConnection = false;          //Setup loop waits for this to be set true before moving into main loop
}

/**
 * @brief Enter SD Card "Dump Mode" for Bluetooth offloading
 * @param dataStr The data string (unused for this command)
 * @param mode Communication mode the command was received from
 */
void handleDumpMode(const char* dataStr, uint8_t mode) {
    //Enter SD Card "Dump Mode" for Bluetooth offloading
    offloadMode = true;                 //Set flag for offloading mode, which is checked by the main loop
    status.setPattern(LED_PATTERN_BLINK);   //Set the LED pattern immediately so the user can tell that it has successfully entered offloading mode
    status.setColor(RGB_COLOR_BLUE);
    status.setSpeed(LED_SPEED_FAST);
}

/**
 * @brief Command to calibrate the compass, which is used to set the offset for the compass heading
 * @param dataStr The data string (unused for this command)
 * @param mode Communication mode the command was received from
 */
void handleCompassCal(const char* dataStr, uint8_t mode) {
    //Command to calibrate the compass, which is used to set the offset for the compass heading
    doCompassCal = true;               //Set flag to indicate that the compass calibration
    logToDebugFile("[INFO] Compass calibration requested");
}

/**
 * @brief Emulated GPS point for testing purposes. Spoofs the GPS latitude and longitude which allows testing of the distance and bearing functions without hardware
 * @param dataStr The data string containing latitude and longitude values
 * @param mode Communication mode the command was received from
 */
void handleEmulatedGPS(const char* dataStr, uint8_t mode) {
    //Emulated GPS point for testing purposes. Spoofs the GPS latitude and longitude which allows testing of the distance and bearing functions without hardware
        
    char tLat[12];                      //Strings for the latitude and longitude, as sscanf cannot handle floats very well, copies string then converts to a float using atof()
    char tLon[12];
    sscanf(dataStr,"%s %s",tLat,tLon);      //Scan in the target latitude and longitude from the data string
    latitude = atof(tLat);              //Convert strings with latitude and longitude to a float variable
    longitude = atof(tLon);
    #ifdef VERBOSE
    Serial.printlnf("Manual GPS override set to Lat: %f, Lon: %f", latitude, longitude);
    #endif
}

/**
 * @brief Stop Command (Emergency stop for motors)
 * @param dataStr The data string (unused for this command)
 * @param mode Communication mode the command was received from
 */
void handleStopCommand(const char* dataStr, uint8_t mode) {
    //Stop Command (Emergency stop for motors)
    driveMode = DRIVE_MODE_MANUAL;                      //Set drive mode back to manual mode
    leftMotorSpeedSetpoint = 90;                     //Stop motors
    rightMotorSpeedSetpoint = 90;                 
    leftMotorSpeed = 90;                //Immediately stop motors (no ramp)
    rightMotorSpeed = 90;
    ESCL.write(90);                     //Immediately write to the ESC a stopped state
    ESCR.write(90);
    stopActive = true;                  //Set flag to indicate that stop was hit
    #ifdef VERBOSE
    Serial.println("Emergency stop activated");
    #endif
}

/**
 * @brief EKF Control Command - enable/disable EKF filtering and tuning parameters
 * @param dataStr The data string containing EKF enable flag and optional noise parameters
 * @param mode Communication mode the command was received from
 */
void handleEKFCommand(const char* dataStr, uint8_t mode) {
    //EKF Control Command - enable/disable EKF filtering and tuning parameters
    int enableFlag = 0;
    float processNoise = 0.0;
    float compassNoise = 0.0;
    
    if(strlen(dataStr) >= 1) {
        enableFlag = atoi(&dataStr[0]);  // First character: 0=disable, 1=enable
        useEKF = (enableFlag == 1);
        
        if(strlen(dataStr) >= 4) {  // Optional noise parameters
            sscanf(dataStr, "%d %f %f", &enableFlag, &processNoise, &compassNoise);
            // Could implement parameter tuning here if needed
        }
        
        #ifdef VERBOSE
        Serial.printlnf("EKF %s, Process Noise: %0.3f, Compass Noise: %0.3f", 
                       useEKF ? "Enabled" : "Disabled", processNoise, compassNoise);
        #endif
        
        // Re-initialize EKF if enabled
        if(useEKF && CompassAvail) {
            float currentHeading = getRawCompassHeading() - compOffset;
            compassEKF.init(currentHeading);
            lastEKFUpdateTime = millis();
        }
    }
}

/**
 * @brief Compass Control Command - switch compass types, status, auto-detect
 * @param dataStr The data string containing compass type (0=LSM303, 1=LIS3MDL, 2=Auto-detect) or empty for status
 * @param mode Communication mode the command was received from
 */
void handleCompassCommand(const char* dataStr, uint8_t mode) {
    //Compass Control Command - switch compass types, status, auto-detect
    
    if(strlen(dataStr) == 0) {
        // Status report
        Serial.printlnf("Compass Status:");
        Serial.printlnf("  Compass pointer: %s", compass ? "Valid" : "NULL");
        Serial.printlnf("  Active compass: %s", compass ? compass->getType() : "None");
        Serial.printlnf("  Compass connected: %s", compass && compass->isConnected() ? "Yes" : "No");
        Serial.printlnf("  Configured compass type: %d", COMPASS_TYPE);
        
        float heading = getRawCompassHeading();
        if (isnan(heading)) {
            Serial.printlnf("  Current heading: NaN (ERROR!)");
        } else {
            Serial.printlnf("  Current heading: %0.2f", heading);
        }
        
        Serial.printlnf("  Compass offset: %d", compOffset);

    } else {
        int compassTypeCmd = atoi(dataStr);
        
        if(compassTypeCmd == 2) {
            // Auto-detect - try both compass types
            Serial.println("Attempting compass auto-detection...");
            
            // Clean up existing compass
            if (compass) {
                delete compass;
                compass = nullptr;
            }
            
            // Temporarily set to auto for setupCompass function
            #define COMPASS_TYPE_AUTO_TEMP 2
            bool success = false;
            
            // Try LIS3MDL first
            compass = new LIS3MDLCompass();
            if (compass->begin()) {
                Serial.printlnf("Auto-detection successful: %s", compass->getType());
                success = true;
            } else {
                delete compass;
                compass = nullptr;
                
                // Try LSM303
                compass = new LSM303Compass();
                if (compass->begin()) {
                    Serial.printlnf("Auto-detection successful: %s", compass->getType());
                    success = true;
                } else {
                    delete compass;
                    compass = nullptr;
                }
            }
            
            if (!success) {
                Serial.println("Compass auto-detection failed");
            }
            
        } else if(compassTypeCmd == 0 || compassTypeCmd == 1) {
            // Switch to specific compass type
            Serial.printlnf("Switching to compass type: %d", compassTypeCmd);
            
            // Clean up existing compass
            if (compass) {
                delete compass;
                compass = nullptr;
            }
            
            if (compassTypeCmd == 0) {
                // LSM303
                compass = new LSM303Compass();
                if (compass->begin()) {
                    Serial.printlnf("Successfully switched to: %s", compass->getType());
                } else {
                    Serial.printlnf("Failed to initialize LSM303 compass");
                    delete compass;
                    compass = nullptr;
                }
            } else {
                // LIS3MDL
                compass = new LIS3MDLCompass();
                if (compass->begin()) {
                    Serial.printlnf("Successfully switched to: %s", compass->getType());
                } else {
                    Serial.printlnf("Failed to initialize LIS3MDL compass");
                    delete compass;
                    compass = nullptr;
                }
            }
        } else {
            Serial.println("Invalid compass type. Use: 0=LSM303, 1=LIS3MDL, 2=Auto-detect");
        }
    }
}

/**
 * @brief Simulation Control Command - enable/disable GPS and compass simulation
 * @param dataStr The data string containing simulation mode (0=Disable, 1-4=Various simulation modes) or empty for status
 * @param mode Communication mode the command was received from
 */
void handleSimulationCommand(const char* dataStr, uint8_t mode) {
    //Simulation Control Command - enable/disable GPS and compass simulation
    
    if(strlen(dataStr) == 0) {
        // Status report
        Serial.printlnf("Simulation Status:");

    } else {
        int simMode = atoi(dataStr);
        
        if(simMode == 0) {
            // Disable simulation
            //simulationData.disableSimulation();
            vehicleSim->disableSimulation();
            Serial.println("Simulation disabled");
        } else if(simMode >= 1 && simMode <= 4) {
            // Enable simulation with specified mode
            vehicleSim->setScenario(simMode);
            const char* modeNames[] = {"", "Lake Raleigh - Vertical", "", "", ""};
            Serial.printlnf("Simulation enabled: %s mode", modeNames[simMode]);
        } else {
            Serial.println("Invalid simulation mode. Use: 0=Disable, 1=Static, 2=Waypoint, 3=Circle, 4=Random Walk");
        }
    }
}

/**
 * @brief Help Command - display available commands and usage
 * @param dataStr The data string (unused for this command)
 * @param mode Communication mode the command was received from
 */
void handleHelpCommand(const char* dataStr, uint8_t mode) {
    //Help Command - display available commands and usage
    Serial.println("\n=== WaterNet23 Vehicle Commands ===");
    Serial.println("Commands can be entered in two ways:");
    Serial.println("1. Raw format: B1CC<cmd><data><checksum>");
    Serial.println("2. Simple format: <cmd> <data> (automatically formatted)");
    Serial.println("");
    Serial.println("Available Commands:");
    Serial.println("  ctl <lat> <lon> <mode> <log> <led>  - Control/navigation command");
    Serial.println("  mtr <lspeed><rspeed>                - Motor control (3 digits each)");
    Serial.println("  tar <value>                         - Set motor tare (000-200, 100 = no tare)");
    Serial.println("  req                                 - Request sensor data");
    Serial.println("  pts <message>                       - Print string to console/log");
    Serial.println("  spc                                 - Status/ping command");
    Serial.println("  hwa                                 - Hello world acknowledge");
    Serial.println("  dmp                                 - Enter data dump mode");
    Serial.println("  cmp                                 - Compass calibration");
    Serial.println("  egp <lat> <lon>                     - Manual GPS override (disables sim)");
    Serial.println("  stp                                 - Emergency stop");
    Serial.println("  ekf <enable> [proc_noise] [comp_noise] - EKF control");
    Serial.println("  cms [type]                          - Compass manager control");
    Serial.println("  sim [mode]                          - Simulation control (auto-logs to CSV)");
    Serial.println("  tbl                                 - Enable status table printing");
    Serial.println("  sdp <seconds>                       - Set status update period (1-10 seconds)");
    Serial.println("  hlp                                 - Show this help");
    Serial.println("");
    Serial.println("Examples:");
    Serial.println("  egp 42.360 -83.073     - Override GPS to specific coordinates");
    Serial.println("  cms                     - Show compass status");
    Serial.println("  ekf1                    - Enable EKF filtering");
    Serial.println("  tbl                     - Re-enable status table");
    Serial.println("  mtr090090               - Stop both motors");
    Serial.println("  pts Hello World         - Print 'Hello World'");
    Serial.printlnf("Current Bot Number: %d", BOTNUM);
    Serial.println("");
    Serial.println("================================");
}

/**
 * @brief Table Command - enable/re-enable status table printing
 * @param dataStr The data string (unused for this command)
 * @param mode Communication mode the command was received from
 */
void handleTableCommand(const char* dataStr, uint8_t mode) {
    //Table Command - enable/re-enable status table printing
    statusTableEnabled = true;
    Serial.println("Status table printing enabled");
    // Clear the screen and immediately print the table
    printStatusTable();
}

/**
 * @brief Tare Command - set motor tare scaling value (0-200, 100 = no tare)
 * Format from CC Hub: "tar%03d" (e.g., tar105)
 * @param dataStr The data string containing a 3-digit tare value
 * @param mode Communication mode the command was received from
 */
void handleTareCommand(const char* dataStr, uint8_t mode) {
    if (strlen(dataStr) < 3) {
        Serial.println("Warning: Tare command data too short");
        logToDebugFile("[WARN] Tare command too short: %s", dataStr);
        return;
    }
    // Parse first three characters as integer
    char tStr[4] = {0};
    strncpy(tStr, dataStr, 3);
    int val = atoi(tStr);
    // Clamp to [0, 200]
    if (val < 0) val = 0;
    if (val > 200) val = 200;
    motorTare = (uint16_t)val;
    Serial.printlnf("[TAR] Motor tare set to %d", motorTare);
    logToDebugFile("[INFO] Motor tare set to %d", motorTare);
}

/// @brief Creates files on the SD card for logging purposes. This function is called once at startup to create the files needed for logging
void initializeLogFiles(){
    if(!SDAvail) return; // No SD card available

    snprintf(filename, MAX_FILENAME_LEN-1, "%sB%d%02d%02d%04d%02d%02d%02d", DEF_FILENAME, BOTNUM, Time.month(),Time.day(),Time.year(),Time.hour(),Time.minute(),Time.second());
    strcpy(logFileName,filename);                           //Copy the base filename to the log file name
    strcpy(autFilename,filename);                           //Copy the base filename to the autonomous navigation log file name    
    strcpy(simFilename,filename);                           //Copy the base filename to the simulation log file name
    strcat(autFilename,"_AUT.csv");                         //Append "_AUT.csv" suffix to the autonomous navigation log file name
    strcat(logFileName,"_LOG.txt");                         //Append "_LOG.txt" suffix to the log file name
    strcpy(simFilename, "_SIM.csv");                        //Initialize simulation filename as empty until simulation is enabled
    strcat(filename,".csv");                                //Append ".csv" suffix to the base filename

    //Open the log file for writing, if it doesn't exist, create it
    if(logSensors){                                 //Logsensors enables logging of sensor data, if enabled, then create the file on the SD card
        dataFile.open(filename, O_RDWR | O_CREAT | O_AT_END);
        dataFile.println(FILE_LABELS);
        dataFile.close();
    }

    //Create the autonomous navigation log file
    if(!autLogFile.isOpen()) {
        autLogFile.open(autFilename, O_RDWR | O_CREAT | O_AT_END);
        if(autLogFile.isOpen()) {
            autLogFile.println("Timestamp,CurrentHeading,TargetHeading,CurrentLat,CurrentLon,TargetLat,TargetLon,LeftMotor,RightMotor");
            autLogFile.close();
            Serial.printlnf("Autonomous log file created: %s", autFilename);
        } else {
            Serial.printlnf("Failed to create autonomous log file: %s", filename);
        }
    }

    //Create the simulation log file
    if(!simLogFile.isOpen()) {
        simLogFile.open(simFilename, O_RDWR | O_CREAT | O_AT_END);
        if(simLogFile.isOpen()) {
            simLogFile.println("Timestamp,Latitude,Longitude,Heading,ActualLeftMotor,ActualRightMotor,Mode");
            simLogFile.close();
            Serial.printlnf("Simulation log file created: %s", simFilename);
        } else {
            Serial.printlnf("Failed to create simulation log file: %s", simFilename);
        }
    }
}

/// @brief Function to update the state of the simulation and water vehicle. Motor speed is fed in, target lat/lon is fetched for scenarios
void updateSimulationData(){
    // Update simulation data
    if(!usingVehicleSim || !vehicleSim->isSimulationActive()) return;

    // Limit update rate of simulation
    static uint32_t lastSimulationUpdate = 0;
    if(millis() - lastSimulationUpdate < 500) return;
    lastSimulationUpdate = millis();

    vehicleSim->updateSimulationKinematics(leftMotorSpeed, rightMotorSpeed);
    
    targetLat = vehicleSim->getTargetLatitude();
    targetLon = vehicleSim->getTargetLongitude();

}

/**
 * @brief Function to log simulation data to SD card
 */
void logSimulationData() {
    if(!SDAvail) return;
}

/**
 * @brief Helper function to write autonomous navigation data to SD card (called from main loop)
 */
void writeMotionDataLog() {
    if(!SDAvail) return; // No logging needed or SD card unavailable
    
    if(millis() - lastAutonomousLogTime < AUTN_LOG_INTERVAL) return; // Limit log rate
    lastAutonomousLogTime = millis(); // Update last log time

    // Create timestamp
    char timestamp[20];
    snprintf(timestamp, 20, "%02d/%02d/%04d %02d:%02d:%02d", 
             Time.month(), Time.day(), Time.year(),
             Time.hour(), Time.minute(), Time.second());
    
    // Use filtered compass heading if available, otherwise use regular compass heading
    float currentHeading = (filteredCompassHeading != 0.0) ? filteredCompassHeading : compassHeading;
    
    // Write to file
    if(!autLogFile.isOpen()) {
        autLogFile.open(autFilename, O_RDWR | O_CREAT | O_AT_END);
    }
    
    if(autLogFile.isOpen()) {
        autLogFile.printlnf("%s,%0.1f,%0.1f,%0.6f,%0.6f,%0.6f,%0.6f,%d,%d",
                           timestamp, currentHeading, travelHeading,
                           latitude, longitude, targetLat, targetLon,
                           leftMotorSpeed, rightMotorSpeed);
        autLogFile.close();
    }

}

/**
 * @brief Call this in loop() to process queued commands
 */
void processQueuedCommands() {
    CommandMsg msg;
    while (commandQueue.pop(msg)) {
        processCommand(msg.msg, msg.mode, msg.sendAck);
    }
}

/**
 * @brief Call this in loop() to process queued commands
 */
void processQueuedDebugMessages() {
    CommandMsg msg;
    while (debugQueue.pop(msg)) {
        logToDebugFile(msg.msg);
    }
}

/**
 * @brief ISR Function to take in a command string received over Cellular and process it using the proccessCommand dictionary
 * @param event Event name from Particle cloud subscription
 * @param data Command data received from Particle cloud
 */
void cmdLTEHandler(const char *event, const char *data){
    commandQueue.push(data, 4, false); // Mode 4 = LTE
}

/**
 * @brief Function that is called by the system once upon startup. Initializes variables used by the system as well as all hardware like the SD card, GPS, sensors, XBee
 */
void setup(){
    status.setPriority(LED_PRIORITY_IMPORTANT);         //First set the LED color to green to indicate to the user that the system has properly started up
    status.setColor(RGB_COLOR_GREEN);
    status.setPattern(LED_PATTERN_SOLID);
    status.setActive(true);                             //Set the LED status to active to change from Particle's LED scheme to the custom patterns
    
    setupPins();                                        //Setup the pins used by the system, which includes the motor control pins, LED pins, and sensor pins    
    readEEPROM();                                       //Read the EEPROM to get the compass calibration

    Particle.connect();                                 //Connect to Particle Cloud via cellular to get LTE messages
    
    uint32_t mtrArmTime = millis();                     //Create a timer to make sure the motors are initialized to 90 (stopped) for at least 2 seconds, otherwise ESC will not become armed
    leftMotorSpeed = leftMotorSpeedSetpoint = 90;       //Set the initial left motor speed of 90, which is stopped. The controller must be held here for 2 seconds to arm the ESC
    rightMotorSpeed = rightMotorSpeedSetpoint = 90;     //Set the initial right motor speed of 90, which is stopped. The controller must be held here for 2 seconds to arm the ESC
    ESCL.attach(ESC_PWM_L,1000,2000);                   //Start PWM-based ESC, with 1ms min pulse width and 2ms max pulse width
    ESCR.attach(ESC_PWM_R,1000,2000);                   //Start PWM-based ESC, with 1ms min pulse width and 2ms max pulse width
    ESCL.write(leftMotorSpeedSetpoint);                 //Set the initial speed of the left motor
    ESCR.write(rightMotorSpeedSetpoint);                //Set the initial speed of the right motor

    Serial.begin(115200);                               //Start serial for USB debugging, 115200 baud rate
    Serial1.begin(115200);                                //Start serial for XBee module

    setupBLE();                                         //Setup BLE module, which includes advertising and setting up the service and characteristics
    setupXBee();                                        //Setup XBee module
    setupNavigationSensors();                           //Setup navigation sensors, which includes GPS and compass

    Particle.subscribe("CCHub", cmdLTEHandler);         //Subscribe to LTE data from Central Control Hub
    Particle.function("Input Command", LTEInputCommand);        //Debug function to feed in commands over LTE

    battPercent = 50;                           //Initially set battery reading to 50% until we read the actual voltage so the LED states are not affected 

    Wire.begin();
    Wire.setClock(CLOCK_SPEED_400KHZ);

    // Initialize EKF for compass heading filtering
    if(CompassAvail) {
        float initialHeading = getRawCompassHeading();
        if (isnan(initialHeading)) {
            Serial.println("Error: Compass heading is NaN, cannot initialize EKF");
            initialHeading = 0.0; // Fallback to zero if heading is invalid
        }
        compassEKF.init(initialHeading - compOffset);
        lastEKFUpdateTime = millis();
        Serial.printlnf("EKF initialized with heading: %0.2f", initialHeading - compOffset);
    }

    watchdog.start();                           //Start the timers
    //motionTimer.start();
    ledTimer.start();
    statusPD.start();

    if (sd.begin(chipSelect, SD_SCK_MHZ(8))) {     //Try to connect to the SD card
        SDAvail = true;                          //Set flag that the SD card is available
        initializeLogFiles();                       //Create the log files on the SD card
        logToDebugFile("[INFO] WaterBot %d: Started Logging!",BOTNUM);
    }
    else{
        Serial.println("Error: could not connect to SD card!");     //If not, warn the user in the console
        logSensors = false;                         //Set flags that the SD card is not available which will warn the user on the CChub
        logMessages = false;
        SDAvail = false;
    }
    
    if(STARTUP_WAIT_PAIR){                          //If a wait pair is enabled, wait for an acknowlede from the CChub before continuing to the main loop
        waitForConnection = true;                   //Set flag true, which will be set false when command received from CChub
        uint32_t publishMS = millis();              //Get current time for limiting rate of publish over cellular
        char dataBuf[10];                           //Hello world string
        sprintf(dataBuf,"B%dCChwd",BOTNUM);
        while(waitForConnection){                   //Wait for flag to be set false
            XBeeHandler();
            if(millis() - publishMS >= XBEE_START_PUB){
                publishMS = millis();
                sendData(dataBuf,0,false,true,false);
            }
            delay(100);
        }
        Serial.println("Successfully paired with controller");
    }
    while(millis() - mtrArmTime < MTR_IDLE_ARM) delay(5);   //Check that the we've been in this setup function for at least two seconds so the ESC's will arm and allow movement
    motorHandler.start();
    Serial.println("Setup complete, entering main loop");   //Print to console that setup is complete
    Serial.printlnf("\n=== WaterNet23 Bot %d Console Ready ===", BOTNUM);
    Serial.println("Type 'help' or 'hlp' for command list");
    Serial.println("Commands can be entered as: <cmd> <data>");
    Serial.println("Example: sim1, cms, ekf1, hlp");
    Serial.println("=====================================\n");
}

/**
 * @brief Function called by the system that continuously loops as long as the device is on. Interrupts will pause this, execute what they are doing (change flags monitored here) and then return control here
 */
void loop(){
    //Serial.printlnf("Time: %d", millis());
    compassCalibration();               //Check if the compass calibration has been requested, and if so, run the calibration function
    getPositionData();                  //Grab position data from GPS and Compass
    readPowerSys();                     //Read power from battery and solar panel
    //sensorHandler();                    //Read and request data from Atlas sensor
    XBeeHandler();                      //Check if a string has come in from XBee
    processQueuedCommands();            //Process any queued commands from the command queue
    processQueuedDebugMessages();       //Writes any debug messages from interrupts to the uSD card
    SerialConsoleHandler();             //Check if a string has come in from Serial console
    statusUpdate();                     //Check if a status update has to be sent out
    writeMotionDataLog();               //Write autonomous navigation data if timer flag is set
    //updateMotors();                     //Update the motor speeds dependent on the mode
    updateSimulationData();             //Updates the state of the simulation and the target waypoint based on the active simulation
    buttonActionDecode();
    if(offloadMode) dataOffloader();    //Check if a signal to offload has been received
    sendResponseData();                 //Send sensor data if requested from the CC
    printStatusTable();                 //Prints the status table over Serial
    varCompassHead = (double)compassHeading;
}

/// @brief Configures the pins used by the system. This function is called once at startup to configure the pins used by the system
void setupPins(){
    //Pin configuration
    pinMode(SENSE_EN, OUTPUT);                          //Configure the pin for the Atlas sensors as an output and pull low to enable power to the Atlas sensors
    digitalWrite(SENSE_EN,LOW);                     
    pinMode(PWR_BUT, INPUT);                            //Configure power button input as an input, no pull as the resistor divider will handle pin floating
    attachInterrupt(PWR_BUT, buttonHandler, RISING);    //Attach the buttonHandler function to trigger whenever the button is pressed
    #ifdef LEAK_DET
    pinMode(LEAK_DET,INPUT);                            //Configure the leak detect output of the PCB to be an input with no pull. External pull on PCB
    #endif
    pinMode(BAT_LEAK_DET,INPUT);                        //Configure the battery leak detect output of the PCB to be an input with no pull. External pull on PCB
    #ifdef PWR_EN                                       //Macro to disable power disable if we are using the Boron as a test platform, as D22 is not present there
        pinMode(PWR_EN, OUTPUT);
        digitalWrite(PWR_EN,HIGH);                      //Set the power enable pin output high to latch the mosfet on the PCB so the system maintains power
    #endif
    #ifdef LEAK_DET                                     //Macro to disable battery leak if we are using the Boron as a test platform, as D23 is not present there
        pinMode(LEAK_DET, INPUT);
    #endif
}

/// @brief Configures the BLE advertising and callbacks. This function is called once at startup to configure the BLE module
void setupBLE(){
    BLE.on();                                           //Turn on Bluetooth
    BLE.setTxPower(8);                                  //Max transmitting power

    BLE.addCharacteristic(txCharacteristic);    //Add characteristic for the stream of transmitting out data over BLE
    BLE.addCharacteristic(rxCharacteristic);    //Add characteristic for the stream of receiving data from BLE
    BLE.addCharacteristic(offloadCharacteristic);   //Add characteristic for offloading stream for BLE
    #ifdef BLE_DEBUG_ENABLED
        BLE.addCharacteristic(bledbgCharacteristic);    //Add BLE Characteristics for BLE serial debug stream
    #endif

    BLECustomData[0] = BOTNUM;                  //Put this bot's bot number in the advertising data so other bots can see it's ID without connecting to it

    advData.appendServiceUUID(WaterNetService); // Add the app service
    advData.appendCustomData(BLECustomData,CUSTOM_DATA_LEN);

    BLE.advertise(&advData);                    //Start advertising the characteristics
}

/**
 * @brief Reads from the EEPROM if it is properly formatted. Updates the EEPROM if formatting does not match
 */
void readEEPROM(){
    //Check if the EEPROM has been configured for use by the CAN analyzer. Read memory items if the keys match
    if(EEPROM.read(EEPROM_KEY1_LOC) == EEPROM_KEY1 && EEPROM.read(EEPROM_KEY2_LOC) == EEPROM_KEY2){
        EEPROM.get(EEPROM_COMP_CAL_LOC, compOffset);    //Read the compass calibration
        EEPROM.get(EEPROM_TARE_LOC, motorTare);         //Read the motor tare value
    }
    //Otherwise, write the default values and the keys for the CAN Analyzer so the EEPROM is set up for the next time
    else{
        EEPROM.put(EEPROM_TARE_LOC, (uint16_t)100);     //Write the default tare value of 100 to the EEPROM
        EEPROM.put(EEPROM_COMP_CAL_LOC, compOffset);    //Write the compass calibration value to the EEPROM
        EEPROM.write(EEPROM_KEY1_LOC, EEPROM_KEY1);
        EEPROM.write(EEPROM_KEY2_LOC, EEPROM_KEY2);
    }
}

void writeEEPROM(){
    EEPROM.put(EEPROM_COMP_CAL_LOC, compOffset);    //Write the compass calibration
    EEPROM.put(EEPROM_TARE_LOC, motorTare);         //Write the motor tare value
}

/**
 * @brief Code to initially configure XBee module over serial. Sends newline and 'B' to bypass the microcontroller onboard the XBee module.
 */
void setupXBee(){
    Serial1.printf("\n");    //First character to set Bypass mode
    delay(20);              //Wait some time before sending next character
    Serial1.printf("B");     //Second character to set Bypass mode
    delay(20);
    //Serial1.printf("Hello from Bot %d\n", BOTNUM);   //Send Hello World message!
}

/**
 * @brief Function to initialize the compass (LIS3MDL or LSM303) and set the parameters for the compass
 * @returns True if navigation sensors were successfully initialized, false otherwise
 */
bool setupNavigationSensors(){
    
    vehicleSim = new VehicleSimulator(); // Initialize vehicle simulator for testing purposes

    // Try to initialize GPS first
    gps = new NeoM8UGPS();
    if(gps->begin()){               // We were able to initialize the GPS
        GPSAvail = true;
    }
    else{
        gps = vehicleSim;           // Use vehicle simulator if GPS fails to initialize
        gps->begin();               // Initialize the simulator
        usingVehicleSim = true;     // Set flag to indicate we are using the vehicle simulator
        Serial.println("Failed to initialize GPS, using vehicle simulator instead");
        GPSAvail = false;
    }
    
    if(usingVehicleSim){    //If the GPS wasn't initialized, then we are using the vehicle simulator. Also use the simulator for compass data
        vehicleSim->updateCompassOffset(compOffset); // Update the compass offset in the simulator so it doesn't use the real-sensor offset
        compass = vehicleSim; // Use vehicle simulator for compass data
        CompassAvail = false; // Set compass availability to false since we are using the simulator
        Serial.println("Using Vehicle Simulator for Compass data");
    }
    else{
        // Try LIS3MDL first (either explicitly requested or auto-detect mode)
        if (COMPASS_TYPE == COMPASS_TYPE_LIS3MDL || COMPASS_TYPE == COMPASS_TYPE_AUTO) {
            compass = new LIS3MDLCompass();
            if (compass->begin()) {
                Serial.printlnf("Compass initialized with: %s", compass->getType());
                CompassAvail = true; // Set compass availability to true
                return true;
            }
            delete compass;
            compass = nullptr;

            // If explicitly requested LIS3MDL and it failed, don't try others
            if (COMPASS_TYPE == COMPASS_TYPE_LIS3MDL) {
                Serial.println("Failed to initialize LIS3MDL compass");
                CompassAvail = false; // Set compass availability to false
                return false;
            }
        }

        // Try LSM303 (either explicitly requested or auto-detect mode)
        if (COMPASS_TYPE == COMPASS_TYPE_LSM303 || COMPASS_TYPE == COMPASS_TYPE_AUTO) {
            compass = new LSM303Compass();
            if (compass->begin()) {
                Serial.printlnf("Compass initialized with: %s", compass->getType());
                CompassAvail = true; // Set compass availability to true
                return true;
            }
            delete compass;
            compass = nullptr;
        }
    }   
    
    return true;
}

/**
 * @brief Checks if the remote control has requested a compass calibration and reads the raw heading to calculate the offset
 */
void compassCalibration(){
    if(doCompassCal){
        lastCalibrationTime = millis();    //Set the last calibration time to the current time
        float sum = 0;
        for(int i = 0; i < COMP_CAL_AVG_COUNT; i++){
            sum += getRawCompassHeading();    //Get the raw compass heading from the compass
            delay(10);    //Delay for 10ms between readings to allow the compass to stabilize
        }
        float avg = sum / (float)COMP_CAL_AVG_COUNT;    //Average the compass heading over the number of samples
        compOffset = (int) avg;                 //Set the compass offset to the average heading
        writeEEPROM();                     //Write the compass offset to the EEPROM so it is saved for next time
        //Serial.printlnf("Compass Calibration: %d",compOffset);    //Print the compass calibration to the console for debugging
        logToDebugFile("[INFO] Compass calibrated with offset: %d", compOffset);    //Log the compass calibration
        doCompassCal = false;    //Set flag to false so we don't keep calibrating the compass
    }
}

/**
 * @brief Function to read power draw of the system and check for leaks. Updates global variables for power system.
 * @returns Battery percentage (0-100)
 */
uint8_t readPowerSys(){
    #ifdef BATT_VSENSE                                                      //Disable voltage sensing if on Boron
        battVoltage = (float) analogRead(BATT_VSENSE) * VDIV_MULT;          //Calculate voltage read in from voltage divider
    #endif
    int rawPCT = (int)(100 * (battVoltage - BAT_MIN)/(BAT_MAX - BAT_MIN));  //Get raw percentage from voltage divider
    if(rawPCT < 0) rawPCT = 0;                                              //Max/min percentage so there is a slight deadzone at either end
    if(rawPCT > 100) rawPCT = 100;
    battPercent = (uint8_t) rawPCT;                                         //Copy raw percentage into global variable
    if(battPercent <= LOW_BATT_PCT) lowBattery = true;                      //Check if battery percentage is low, if low then set flag so the LED flashes and the CChub pops up an alert
    else lowBattery = false;
    battCurrent = (float) analogRead(BATT_ISENSE) * BAT_ISENSE_MULT / 4095; //Read the amplified input from the shunt from the batter and solar array and calculate the multiplier based on the resistor value and datasheet
    solarCurrent = (float) analogRead(SOL_ISENSE) * SLR_ISENSE_MULT / 4095;

    #ifdef LEAK_DET
    if(!digitalRead(LEAK_DET) && !warnedLeak){                              //LEAK_DET pin is pulled low when a leak is detected
        char warnChar[12];                                                  //String to hold transmitted string
        if(!LEAK_DET_BYPASS) sprintf(warnChar,"B%dCCldt",BOTNUM);           //Create error string based on if it's a cutoff trigger or a just a warning
        else sprintf(warnChar,"B%dCCwld",BOTNUM);                           //Warn only
        sendData(warnChar,0,true,true,true);                                //Send data out over all transmission methods regardless
        logToDebugFile("[WARN] Leak detected, sending warning: %s", warnChar); //Log the warning to the debug file
        delay(50);                                                          //wait 50ms for data to go out
        if(!LEAK_DET_BYPASS) digitalWrite(PWR_EN,LOW);                      //kill system
        warnedLeak = true;                                                  //Set flag to not spam console in case cutoff doesn't work
    }
    if(!digitalRead(BAT_LEAK_DET) && !warnedBattLeak){                      //BAT_LEAK_DET pin is pulled low when a leak is detected
        char warnChar[12];                                                  //String to hold transmitted string
        if(!LEAK_DET_BYPASS && BATT_TRIG_LEAK) sprintf(warnChar,"B%dCCldb",BOTNUM);
        else sprintf(warnChar,"B%dCCwlb",BOTNUM);                           //Message to warn leak in battery
        sendData(warnChar,0,true,true,true);                                //Send data out over all transmission methods regardless
        logToDebugFile("[WARN] Battery leak detected, sending warning: %s", warnChar); //Log the warning to the debug file
        delay(50);                                                          //wait 50ms for data to go out
        if(!LEAK_DET_BYPASS && BATT_TRIG_LEAK) digitalWrite(PWR_EN,LOW);    //kill system
        warnedBattLeak = true;
    }
    #endif
    return battPercent;
}

/**
 * @brief Convert degrees to radians, used by the compass bearing calculation
 * @param deg Angle in degrees
 * @returns Angle in radians
 */
float deg2rad(float deg) {
  return deg * (3.14159/180);   //Multiply by Pi/180
}

/**
 * @brief Function to take an x and y acceleration from the compass and return a raw value between -180 and +180 degrees
 * @param x_accel X-axis acceleration from compass
 * @param y_accel Y-axis acceleration from compass
 * @returns Compass heading in degrees (-180 to +180)
 */
float lis3mdlCompassHeading(float x_accel, float y_accel){
    float rawHeading = atan2(y_accel, x_accel) * 180.0 / M_PI;  //Convert x and y compass acceleration to a heading
    #ifdef VERBOSE
    //Serial.printlnf("Raw Heading: %f", rawHeading);
    #endif
    return rawHeading;   //Call the remap function to get a calibrated heading
}

/**
 * @brief function that takes two latitudes and longitudes and calculates the distance (in meters) between them. Used by autonomous system for determining arrival at a point
 * @param lat1 First latitude in degrees
 * @param lat2 Second latitude in degrees
 * @param lon1 First longitude in degrees
 * @param lon2 Second longitude in degrees
 * @returns Distance between points in meters
 */
float calcDistance(float lat1, float lat2, float lon1, float lon2){
    float dLat = deg2rad(lat1-lat2);    //Calculate difference between latitudes
    float dLon = deg2rad(lon1-lon2);    //Calculate difference between longitudes
    float a = sinf(dLat/2) * sinf(dLat/2) + cosf(deg2rad(lat2)) * cosf(deg2rad(lat1)) * sinf(dLon/2) * sinf(dLon/2);    //Formula from a stackexchange post
    float c = 2 * atan2(sqrt(a), sqrt(1.0-a)); 
    return 6371000.0 * c; // Distance in m
}

/// @brief Function to calculate the shortest rotation to get to the target heading based on the current heading
/// @param compassHead current compass reading
/// @param targetHead target heading from the current lat/lon to the target lat/lon
/// @return Degrees to rotate the vehicle to get to target heading (-180 to 180)
float calcDelta(float compassHead, float targetHead){
    // Return the shortest signed angle difference between compassHead and targetHead
    float delta = targetHead - compassHead;
    // Wrap to [-180, 180]
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

/**
 * @brief Function to get the raw compass heading from the compass module (0-360). This is used for debugging and testing purposes, as well as for the autonomous system to determine which way to turn
 * @returns Raw compass heading in degrees (0-360), or 0.0 if compass unavailable
 */
float getRawCompassHeading(){
    float rawHeading = 0;     //Create a variable to hold the heading from the compass, regardless
    
    // Use the new compass interface if available
    if (compass && compass->isConnected()) {
        rawHeading = compass->getCompassHeading();
        
        // Check for NaN values and handle gracefully
        if (isnan(rawHeading)) {
            Serial.println("Warning: Compass returned NaN");
            return 0.0; // Return 0 as fallback heading
        } else {
            // Convert from -180/+180 range to 0-360 range for compatibility
            if (rawHeading < 0) rawHeading += 360;
            #ifdef VERBOSE
            Serial.printlnf("Raw Heading (via compass): %0.2f", rawHeading); 
            #endif
        }
        return rawHeading;
    }
    
    // No compass available
    Serial.println("Warning: No compass available");
    return 0.0; // Return 0 as default heading
}

/**
 * @brief Function to get the calibrated compass heading, which is used by the autonomous system to determine which way to turn
 * @returns Calibrated compass heading in degrees (-180 to 180)
 */
float getCalibratedCompassHeading(){
    float cHeading = getRawCompassHeading();   //Get the raw compass heading from the compass module
    cHeading -= compOffset;   //Add the offset to the compass heading to get the calibrated heading
    if(cHeading > 180) cHeading -= 360;
    else if(cHeading < -180) cHeading += 360;
    
    // EKF Processing for improved heading estimation
    if(useEKF && CompassAvail && compassEKF.isInitialized()) {
        float currentTime = millis();
        float dt = (currentTime - lastEKFUpdateTime) / 1000.0; // Convert to seconds
        
        if(dt > 0.001) { // Only update if sufficient time has passed
            // Prediction step
            compassEKF.predict(dt);
            
            // Update with compass measurement
            float compassVariance = 25.0; // Adjust based on your compass noise characteristics
            compassEKF.updateCompass(cHeading, compassVariance);
            
            // Update with GPS course if available and moving
            if(GPSAvail && gps->isConnected()) {
                float gpsSpeed = gps->getGroundSpeed(); // Convert mm/s to m/s
                if(gpsSpeed > 0.5) { // Only use GPS course if moving fast enough
                    float gpsCourse = gps->getHeading(); // Convert to degrees
                    if(gpsCourse > 180) gpsCourse -= 360;
                    float gpsVariance = 100.0; // GPS course is typically less accurate than compass
                    compassEKF.updateGPS(gpsCourse, gpsSpeed, gpsVariance);
                }
            }
            
            // Get filtered heading
            filteredCompassHeading = compassEKF.getHeading();
            lastEKFUpdateTime = currentTime;
            
            // Use filtered heading instead of raw heading
            cHeading = filteredCompassHeading;
            
            #ifdef VERBOSE
            Serial.printlnf("Raw: %0.2f, Filtered: %0.2f", 
                           getRawCompassHeading() - compOffset, filteredCompassHeading);
            #endif
        }
    }
    
    if(targetLat >= -90.0f && targetLat <= 90.0f && targetLon >= -90.0f && targetLon <= 90.0f){         //Check that the target latitude and longitude are valid
        travelHeading = (atan2(targetLon-longitude, targetLat-latitude) * 180 / M_PI);      //Calculate the heading between the current and target location
        travelDistance = calcDistance(targetLat,latitude,targetLon,longitude);              //Calculate the distance between the current and target location
        headingDelta = calcDelta(cHeading, travelHeading);                             //Calculate delta to control angle of the bot
        lastTelemTime = millis();                                                           //Update telemetry time
        if(CompassAvail) telemetryAvail = true;                                             //If compass and GPS are available, set flag to true
        #ifdef VERBOSE
            Serial.printlnf("Head: %0.2f, Target: %0.2f, Delta: %0.2f", cHeading, travelHeading, headingDelta);
        #endif
    }  
    return cHeading;
}

/** @brief Function to read data from the GPS and compass module and then call the distance calculation functions for updating autonomous movement */
void getPositionData(){
    if(millis() - positionTimer > GPS_POLL_TIME){       //Use a timer to slow the poll rate on GPS and Compass, as they do not same that quickly
        positionTimer = millis();                       //Reset timer
        
        // Use real GPS
        if(gps->isConnected()){                        //Only read from GPS if it is connected
            float newLat = gps->getLatitude();      //Get latitude in degrees
            float newLon = gps->getLongitude();  
            if(newLat != -999.0f) latitude = newLat;
            if(newLon != -999.0f) longitude = newLon;
            //Serial.printlnf("Lat: %0.7f Lon: %0.7f", latitude, longitude);
            GPSAvail = true;
        }
        else GPSAvail = false;                          //Set flag to indicate GPS unavailable if not connected
    }
    if(millis() - compassTimer > COMP_POLL_TIME){
        compassTimer = millis();                       //Reset timer
        compassHeading = getCalibratedCompassHeading();   //Get the calibrated compass heading
        //Serial.printlnf("Compass Heading: %0.2f", compassHeading);   //Print the heading to the console for debugging
        #ifdef VERBOSE
            Serial.printlnf("Compass Heading: %0.2f", compassHeading);   //Print the heading to the console for debugging
        #endif
    }
}

/** @brief Function to check if response data to a request needs to be sent out */
void sendResponseData(){
    if(requestActive){              //If the CC has requested data using the req command
        char responseStr[65];       //Create string to hold sensor data
        memset(responseStr,0,65);   //Empty the string if it had something
        sprintf(responseStr,"B%dCCsns%0.6f %0.6f %d %d %d %d %d ",BOTNUM,latitude,longitude,(int)(senseDO*1000),(int)(sensePH*1000),(int)(senseCond*1000),(int)(senseMCond*1000),(int)(senseTemp*1000));
        sendData(responseStr,requestActive,false,false,false);  //transmit out data over the same mode the request was recived over
        requestActive = 0;          //Set flag back to 0
    }
}

/** @brief Function to check if the status is updated based on a flag and then transmit it out to the CChub */
void statusUpdate(){
    static uint32_t lastLTESendTime = 0;
    if(statusReady){        //Check if status flag has been set by timer that calculates system status flags
        #ifdef VERBOSE
        Serial.println("Sending a status update!");     //Log to console (for debug purposes)
        #endif
        char updateStr[70];                             //Create local string to hold status being sent out
        int txCompassHead = compassHeading;    //Get the compass heading to send out over the status update
        if(txCompassHead < 0) txCompassHead += 360;   //If the heading is negative, add 360 to it to get a positive value
        sprintf(updateStr,"B%dABsup%d %d %0.6f %0.6f %d %d %d %d %d %d %u %u",
            BOTNUM,
            battPercent,
            statusFlags,
            latitude, longitude,
            (int)(battVoltage * battCurrent),
            (int)(battVoltage * solarCurrent), 
            txCompassHead, (int)travelHeading,
            leftMotorSpeed, rightMotorSpeed, motorTare,
            waypointIndex);  //Print status flags, battery, latitude and logitude, compass heading and motor speeds to string
        bool sentOverLTE = false;
        if(!BLEAvail && !XBeeAvail && LTEStatusCount && (LTEStatusCount%LTE_STAT_PD == 0)){
            uint32_t now = millis();
            if(now - lastLTESendTime >= 30000 || lastLTESendTime == 0){ // Only send over LTE every 30 seconds
                sendData(updateStr,0,false,false,true);     //Only send out over LTE
                lastLTESendTime = now;
                sentOverLTE = true;
            }
        }
        if(!sentOverLTE){
            if(XBeeAvail || BLEAvail) LTEStatusCount = LTE_MAX_STATUS;  //Otherwise, we're sending updates over BLE or XBee, reset counter for cellular
            sendData(updateStr,0,true,true,false);
        }
        if(!BLEAvail && !XBeeAvail && LTEStatusCount) LTEStatusCount--;            //Decrement a large coounter for the LTE status. This stops sending the status over LTE after a while to not burn up monthly quota. Should be recovering bots if on cell only
        statusReady = false;                            //Clear ready flag
        //sendData("B1CCptsbigbot",0,true,false,false);
    }
}

/// @brief Function to calculate the average of an array of floats
/// @param arr array holding the previous error values
/// @param len number of elements in the array
/// @param idx circular buffer index
/// @return average of elements in the array (before substitution)
float averageAndAdvanceCircularBuffer(float* arr, int len, int *idx, float newVal) {
    if (len <= 0 || arr == nullptr || idx == nullptr) return 0.0;
    float sum = 0.0;
    for (int i = 0; i < len; ++i) { // Sum up all errors
        sum += arr[i];
    }
    if(*idx >= len) *idx = 0;       // Wrap circular buffer around
    arr[*idx++] = newVal;            // Assign new error value to error array
    return sum / len;               // Divide by sum to get average
}

/// @brief Function to calculate what speed the motors should move at based on the current drive mode (manual, sentry, autonomous)
void updateMotors(){
    // The control system architecture for movement will involve two PID controllers

    bool doCompassPID = false;
    static bool pointArrived = false;
    if(driveMode == DRIVE_MODE_SENTRY || driveMode == DRIVE_MODE_AUTONOMOUS){
        doCompassPID = true;
        if(travelDistance < MTR_CUTOFF_RAD || (travelDistance < SENTRY_IDLE_RAD && pointArrived)){            //If the bot is close enough to the center when in autonomous and sentry, then disable motors and float there
            waypointArrived = true;                     //Set flag to indicate that the waypoint has been reached
            pointArrived = true;                        //Indicate that the bot has arrived at the target point, which acts as a disable until it drifts out of the larger radius
            leftMotorSpeedSetpoint = 90;
            rightMotorSpeedSetpoint = 90;
        }
        else{                                           //Otherwise, we are outside the radius of both circles
            waypointArrived = false;                    //Clear the waypoint arrived flag
            pointArrived = false;
            leftMotorSpeedSetpoint = MTR_TRAVEL_SPD;
            rightMotorSpeedSetpoint = MTR_TRAVEL_SPD;
        }
    }

    // The first PID controller is for controlling the speed of the vehicle.
    // When the joystick is pressed forward, we don't want the motors to immediately jump to full speed or the power draw will be crazy
    // Same thing for when a new waypoint is given, the vehicle would ramp to max speed without a control system.
    // The error signal will be calculated by the current motor speed vs the commanded motor speed (either by joystick or waypoint)
    const float kp_speed = 0.05f;
    const float ki_speed = 0.3f;
    const float kd_speed = 0.0f;

    const uint8_t int_speed_count = 20;                         // Number of samples for the integral term
    static int int_buf_idx_left = 0;                            // Index for the left motor error circular buffer
    static int int_buf_idx_right = 0;                           // Index for the right motor error circular buffer
    static float int_speed_left[int_speed_count] = {0.0f};      // Array to hold the previous error values for integral
    static float int_speed_right[int_speed_count] = {0.0f};     // Array to hold the previous error values for integral


    // The second PID controller is for controlling the vehicle rotation based on the compass heading
    // When operating autonomously, we want to smooth out the rotation of the vehicle to the correct heading
    // The error signal will be calculated by the current compass heading vs the target compass heading
    const float kp_angle = 0.15f;
    const float ki_angle = 0.15f;
    const float kd_angle = 0.0f;
    
    const uint8_t int_angle_count = 15;                          // Number of samples for the integral term
    static int int_buf_idx_angle = 0;                            // Index for the compass angle error circular buffer
    static float int_angle[int_angle_count] = {0.0f};            // Array to hold the previous error values for integral
    

    // Speed PID Calculations
    static float leftMotorSpeedSet = (float)leftMotorSpeed;    // Local variable to hold the left motor speed in float for better accuracy
    static float rightMotorSpeedSet = (float)rightMotorSpeed;  // Local variable to hold the right motor speed in float for better accuracy

    float e_speed_left = (float)leftMotorSpeedSetpoint - leftMotorSpeedSet;       // Calculate error for left motor speed
    float e_speed_right = (float)rightMotorSpeedSetpoint - rightMotorSpeedSet;    // Calculate error for right motor speed

    static float last_e_speed_left = 0.0f;                      // Copy of the last error value - use for differential term
    static float last_e_speed_right = 0.0f;                     // Copy of the last error value - use for differential term

    float e_int_speed_left = averageAndAdvanceCircularBuffer(int_speed_left, int_speed_count, &int_buf_idx_left, e_speed_left);
    float e_int_speed_right = averageAndAdvanceCircularBuffer(int_speed_right, int_speed_count, &int_buf_idx_right, e_speed_right);

    float e_diff_speed_left = e_speed_left - last_e_speed_left;
    float e_diff_speed_right = e_speed_right - last_e_speed_right;

    leftMotorSpeedSet = leftMotorSpeedSet + ((kp_speed * e_speed_left) + (ki_speed * e_int_speed_left) + (kd_speed * e_diff_speed_left));
    rightMotorSpeedSet = rightMotorSpeedSet + ((kp_speed * e_speed_right) + (ki_speed * e_int_speed_right) + (kd_speed * e_diff_speed_right));


    // Rotation PID Calculations
    float e_angle = calcDelta(compassHeading, travelHeading);   // e will be positive if we need to rotate clockwise (faster left motor)
    float e_int_angle = averageAndAdvanceCircularBuffer(int_angle, int_angle_count, &int_buf_idx_angle, e_angle);
    
    float leftRightDifferential = 0.0f;
    if(doCompassPID) leftRightDifferential = ((kp_angle * e_angle) + (ki_angle * e_int_angle) + (kd_angle * 0));
    

    // Now feed in the output of the speed PID to the rotation differential PID to get the output motor speed
    float leftMotorSpeedF = leftMotorSpeedSet - leftRightDifferential;
    float rightMotorSpeedF = rightMotorSpeedSet + leftRightDifferential;

    // Now do motor taring to account for differences in motors, ESCs and propellers
    // The ESC command range is [0..180] with 90 as neutral. Apply scaling to the deviation from 90 so
    // neutral remains unchanged and both forward (>90) and reverse (<90) are reduced proportionally.
    // Linear scaling:
    //  - motorTare > 100: slow LEFT motor to (2.0 - motorTare/100) of its deviation from 90 (e.g., 105 -> 0.95x)
    //  - motorTare < 100: slow RIGHT motor to (motorTare/100) of its deviation from 90 (e.g., 90 -> 0.90x)
    //  - motorTare == 100: no change
    {
        float leftDelta = leftMotorSpeedF - 90.0f;
        float rightDelta = rightMotorSpeedF - 90.0f;
        if (motorTare > 100) {
            float scale = 2.0f - ((float)motorTare / 100.0f);
            if (scale < 0.0f) scale = 0.0f;
            if (scale > 1.0f) scale = 1.0f;
            leftDelta *= scale;
        } else if (motorTare < 100) {
            float scale = ((float)motorTare) / 100.0f;
            if (scale < 0.0f) scale = 0.0f;
            if (scale > 1.0f) scale = 1.0f;
            rightDelta *= scale;
        }
        leftMotorSpeedF = 90.0f + leftDelta;
        rightMotorSpeedF = 90.0f + rightDelta;
    }

    // Clamp motor speeds to [0, 180] before casting to uint8 to avoid wrap
    if (leftMotorSpeedF < 0.0f) leftMotorSpeedF = 0.0f;
    else if (leftMotorSpeedF > 180.0f) leftMotorSpeedF = 180.0f;
    if (rightMotorSpeedF < 0.0f) rightMotorSpeedF = 0.0f;
    else if (rightMotorSpeedF > 180.0f) rightMotorSpeedF = 180.0f;

    leftMotorSpeed = (uint8_t)leftMotorSpeedF;                  // Convert back to int to command ESC
    rightMotorSpeed = (uint8_t)rightMotorSpeedF;                // Convert back to int to command ESC

    if(!stopActive){                    //If there has not been a stop command, then update the ESC
        ESCL.write(180-leftMotorSpeed);
        ESCR.write(rightMotorSpeed);
        //Serial.printlnf("Update motor speed (%dms): %d %d", millis(), rightMotorSpeedSetpoint, leftMotorSpeedSetpoint);
    }
}

/**
 * @brief Major function for sending a string of data out over BLE, XBee or LTE. Automatically calculates the checksum from the given string
 * @param dataOut The data string to send
 * @param sendMode Communication mode to use (0=use individual flags, 1=BLE, 2=XBEE, 4=LTE)
 * @param sendBLE Flag to enable sending over BLE
 * @param sendXBee Flag to enable sending over XBee
 * @param sendLTE Flag to enable sending over LTE
 */
void sendData(const char *dataOut, uint8_t sendMode, bool sendBLE, bool sendXBee, bool sendLTE){
    //Can either use sendMode or the individual flags. sendMode enables easy re-transmission from the same method that it was received from
    char outStr[strlen(dataOut)+2];                             //Make a temporary string to hold the inputted strigng and also the checksum
    sprintf(outStr,"%s%02x",dataOut,strlen(dataOut));           //Copy in the inputted string and concatenate the checksum on the end, which is just the string length of the inputted string
    if(sendLTE || sendMode == 4){                               //If sending over LTE, just use the particle event
        Particle.publish("Bot1dat", outStr, PRIVATE);
    }
    if((sendBLE || sendMode == 1) && BLE.connected()){          //Check that BLE is connected before trying to transmit here
        uint8_t txBuf_tmp[strlen(outStr)];                      //Create an array of uint8_t, which is needed for the BLE characteristic.
        memcpy(txBuf_tmp,outStr,strlen(outStr));                //Convert the char array to a byte array
        txCharacteristic.setValue(txBuf_tmp, strlen(outStr));   //Transmit out the byte array
    }
    if(sendXBee || sendMode == 2){                              //Send out over the harware serial to go through XBee
        Serial1.println(outStr);
    }
}

/** @brief Function used for BLE debugging that allows printing debug messages to a remote device */
void printBLE(const char *dataOut){
    #ifdef BLE_DEBUG_ENABLED                                        //Only functional if BLE debugging is enabled, disable to reduce overhead in final build
        static uint32_t BLEdbgTimer = 0;
        if(millis() - BLEdbgTimer < 1000) return;
        BLEdbgTimer = millis();
        uint8_t txBuf_tmp[strlen(dataOut)];                         //Convert input string to byte array to transmit out of BLE
        memcpy(txBuf_tmp,dataOut,strlen(dataOut));                  //Copy character array elements into byte array
        bledbgCharacteristic.setValue(txBuf_tmp, strlen(dataOut));  //Transmit out byte array
    #endif
}

/** @brief ISR Function to calculate bitmasked status number and signal to the main loop that the status is ready */
void StatusHandler(){
    statusFlags = 0;                        //Reset status flags to 0, then add up the individual flags
    statusFlags = LTEAvail;                 //Bit 0 indicates LTE is available
    statusFlags |= XBeeAvail << 1;          //Bit 1 indicates XBee is available
    statusFlags |= BLEAvail << 2;           //Bit 2 indicates BLE is available
    statusFlags |= offloadMode << 3;        //Bit 3 indicates bot is currently offloading
    statusFlags |= driveMode << 4;          //Bit 4 indicates the current drive mode
    statusFlags |= lowBattery << 6;         //Bit 6 indicates that the battery is low
    statusFlags |= logSensors << 7;         //Bit 7 indicates that the Atlas sensors are being logged to the SD card
    statusFlags |= GPSAvail << 8;           //Bit 8 indicates neo-m8u GPS is available and receiving non-null data
    statusFlags |= CompassAvail << 9;       //Bit 9 indicates the LIS3MDL compass is connected and providing data
    statusFlags |= SDAvail << 10;           //Bit 10 indicates the SD card is functional and can record data
    statusFlags |= waypointArrived << 11;   //Bit 11 indicates that the bot has arrived at the target waypoint
    statusReady = true;                     //Set flag true, so the main loop will transmit out status to CChub
    //Serial.println("Sending a status update!");
}

/** @brief Function to read and request data from the Atlas scientific sensors over I2C. Uses millis() timer to ensure at least 850ms between request and read (required for Atlas sensors) */
void sensorHandler(){
    if(!SENS_CONNECTED) return;                //If sensors are not connected, then return and do nothing
    if(dataTimer < millis() && dataWait){       //Check if the timer for waiting after a data request has expired
        if(Wire.requestFrom(PHADDR, 20, 1)){    //Request 20 bytes from the PH sensor
            Wire.read();            //the first byte is the response code, we read this separately.
            char tempSense[20];                 //Temporary string to hold string returned by the sensor
            int c = 0;                          //Index variable for the temporary string
            while(Wire.available()){            // slave may send less than requested
                tempSense[c++] = Wire.read();   //Read each of the bytes returned by the sensor into a string

            }
            sensePH = atof(tempSense);          //Convert the string to a float and store it in the global pH variable
        }
        //Serial.printlnf("pH: %f", sensePH);
        if(Wire.requestFrom(MCOND, 20, 1)){
            Wire.read();            //the first byte is the response code, we read this separately.
            char mcondSense[20];
            int c = 0;
            while(Wire.available()){   // slave may send less than requested
                mcondSense[c++] = Wire.read();

            }
            senseMCond = atof(mcondSense);
        }
        //Serial.printlnf("MiniCond: %f",senseMCond);
        if(Wire.requestFrom(COND, 20, 1)){
            Wire.read();            //the first byte is the response code, we read this separately.
            char condSense[20];
            int c = 0;
            while(Wire.available()){   // slave may send less than requested
                condSense[c++] = Wire.read();

            }
            senseCond = atof(condSense);
        }
        //Serial.printlnf("Conductivity: %f",senseCond);
        if(Wire.requestFrom(TEMPADDR, 20, 1)){
            Wire.read();             //the first byte is the response code, we read this separately.
            char addrSense[20];
            int c = 0;
            while(Wire.available()){   // slave may send less than requested
                addrSense[c++] = Wire.read();

            }
            senseTemp = atof(addrSense);
        }
        if(Wire.requestFrom(DOADDR, 20, 1)){
            Wire.read();             //the first byte is the response code, we read this separately.
            char addrSense[20];
            int c = 0;
            while(Wire.available()){   // slave may send less than requested
                addrSense[c++] = Wire.read();

            }
            senseDO = atof(addrSense);
        }
        //Serial.printlnf("Temperature: %f",senseTemp);
        dataWait = false;                       //Set flag false until next data request has been made
        if(logSensors){                         //Log sensors to SD card if enabled
            char timestamp[18];                 //String to hold timestamp being logged
            snprintf(timestamp,16,"%02d%02d%04d%02d%02d%02d",Time.month(),Time.day(),Time.year(),Time.hour(),Time.minute(),Time.second());
            if(!dataFile.isOpen()){               //Print out each of the global sensor values
                dataFile.open(filename, O_RDWR | O_CREAT | O_AT_END);
                dataFile.printlnf("%s,%f,%f,%f,%f,%f,%f,%f",timestamp,latitude,longitude,senseTemp,sensePH,senseDO,senseMCond,senseCond);
                dataFile.close();
            } 
            else{
                dataFile.printlnf("%s,%f,%f,%f,%f,%f,%f,%f",timestamp,latitude,longitude,senseTemp,sensePH,senseDO,senseMCond,senseCond);
            }
        }
    }
    if(senseTimer < millis()){                  //Check time to see if we should request more data
        senseTimer = millis() + SENS_POLL_RT;   //Calculate next time to request data from the sensors
        dataTimer = millis() + SENS_DATA_DLY;   //Calculate the time to read the data that has just been requested
        Wire.beginTransmission(PHADDR);         //call the circuit by its ID number.
        Wire.write('r');                        //transmit the command that was sent through the serial port.
        Wire.endTransmission();                 //end the I2C data transmission.
        Wire.beginTransmission(MCOND);          //call the circuit by its ID number.
        Wire.write('r');                        //transmit the command that was sent through the serial port.
        Wire.endTransmission();                 //end the I2C data transmission.
        Wire.beginTransmission(COND);           //call the circuit by its ID number.
        Wire.write('r');                        //transmit the command that was sent through the serial port.
        Wire.endTransmission();                 //end the I2C data transmission.
        Wire.beginTransmission(TEMPADDR);       //call the circuit by its ID number.
        Wire.write('r');                        //transmit the command that was sent through the serial port.
        Wire.endTransmission();                 //end the I2C data transmission.
        Wire.beginTransmission(DOADDR);         //call the circuit by its ID number.
        Wire.write('r');                        //transmit the command that was sent through the serial port.
        Wire.endTransmission();      
        dataWait = true;
    }
}

/** @brief Function to check the UART serial buffer from XBee and then send any received commands to the processCommand function */
void XBeeHandler(){  
    while(Serial1.available()){                         //Read data from the XBee buffer
        String data = Serial1.readStringUntil('\n');    //Each command is terminated by a null character, so use this to separate multiple commands if multiple in the buffer
        char buffer[data.length()];                     //Create a buffer to take the received string object and make a character array to pass to processCommand
        for(uint16_t i = 0 ; i < data.length(); i++) buffer[i] = data.charAt(i);    //Loop over characters and copy them into char array
        if(data.length() > 1 && data.charAt(data.length()-1) == '\r') buffer[data.length()-1] = 0;      //If there was a carriage return, then get rid of it and set to terminate character
        #ifdef VERBOSE
        Serial.println("New XBee Command:");
        Serial.println(data);                           //Print out command for debugging
        #endif
        commandQueue.push(buffer, 2, true); // Mode 2 = XBEE
        if(buffer[0] == 'B' || buffer[0] == 'C') XBeeRxTime = millis(); //If the first characters were from another bot or from the CC, then assume Xbee is working, so update it's watchdog counter
    }
}

/** @brief Function to handle commands received from Serial console (COM port) */
void SerialConsoleHandler(){
    while(Serial.available()){                          //Read data from the Serial buffer
        String data = Serial.readStringUntil('\n');     //Each command is terminated by a newline character
        data.trim();                                     //Remove any leading/trailing whitespace
        
        if(data.length() == 0) return;                  //Ignore empty lines
        
        // Disable status table printing when any command is sent
        statusTableEnabled = false;
        
        // Handle special console commands
        if(data.equalsIgnoreCase("help") || data.equals("?")) {
            data = "hlp";  // Convert to standard help command
        }
        
        char buffer[data.length() + 10];                //Create a buffer with extra space for formatting
        
        Serial.printlnf("> %s", data.c_str());          //Echo the command
        
        #ifdef VERBOSE
        Serial.println("New Serial Console Command:");
        Serial.println(data);                           //Print out command for debugging
        #endif
        
        // Check if this is a raw command (starts with B or C) or needs to be formatted
        if(data.startsWith("B") || data.startsWith("C")) {
            // Already formatted command - process directly
            strcpy(buffer, data.c_str());
            processCommand(buffer, 1, true);             //Process using mode 1 (similar to BLE)
        } else {
            // Format command for this bot: B<BOTNUM>CC<command><checksum>
            String formattedCmd = "CCB" + String(BOTNUM) + data;
            int checksum = formattedCmd.length();
            
            // Format checksum as 2-digit hex
            if(checksum < 16) {
                formattedCmd += "0" + String(checksum, HEX);
            } else {
                formattedCmd += String(checksum, HEX);
            }
            
            strcpy(buffer, formattedCmd.c_str());
            #ifdef VERBOSE
            Serial.printlnf("Formatted command: %s", buffer);
            #endif
            processCommand(buffer, 1, true);             //Process the formatted command
        }
                
        Serial.println(""); // Add blank line for readability
    }
}

/**
 * @brief ISR function triggered whenever data is received over BLE. Converts to a string and then sends data to processCommand dictionary
 * @param data Pointer to received data buffer
 * @param len Length of received data in bytes
 * @param peer BLE peer device that sent the data
 * @param context Context pointer (unused)
 */
static void BLEDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context){
    char btBuf[len+1];                                              //Create character array for the received command
    for (size_t ii = 0; ii < len; ii++) btBuf[ii] = data[ii];       //Convert byte array into character array
    if(btBuf[len-1] != '\0') btBuf[len] = '\0';                     //Make sure there is a null character at the end (another bug that cost many hours and seeing random data from surrounding memory)
    else btBuf[len-1] = '\0';
    #ifdef VERBOSE
    Serial.println("New BT Command:");
    Serial.println(btBuf);                                          //Print out command for debugging purposes
    #endif
    commandQueue.push(btBuf, 1, true); // Mode 1 = BLE
    if(btBuf[0] == 'A' || btBuf[0] == 'C') BLERxTime = millis();    //If the first characters were from another bot or from the CC, then assume Xbee is working, so update it's watchdog counter
}

/** @brief ISR timer to check if strings have been received from the CCHub, and will cut off motors if an update has not been received recently */
void motionHandler(){
    //If the bot is operating in manual RC mode, then check that a mtr message has been received from the CC recently, otherwise cut off to prevent driving into oblivion
    if(driveMode == DRIVE_MODE_MANUAL && leftMotorSpeedSetpoint != 90 && rightMotorSpeedSetpoint != 90 && millis() - lastMotorCommandTime > MTR_TIMEOUT){
        leftMotorSpeedSetpoint = 90;
        rightMotorSpeedSetpoint = 90;
        leftMotorSpeed = 90;
        rightMotorSpeed = 90;
        ESCL.write(leftMotorSpeedSetpoint);
        ESCR.write(rightMotorSpeedSetpoint);
        //Serial.printlnf("Warning, motor command has not been received in over %dms, cutting motors", MTR_TIMEOUT);
    }
    //If we're in an autonomous mode, also check that telemetry is available, otherwise, return to manual RC mode
    if(!telemetryAvail && driveMode != DRIVE_MODE_MANUAL && millis() - lastTelemTime > MTR_TIMEOUT){
        driveMode = DRIVE_MODE_MANUAL;
        telemetryAvail = false;
        leftMotorSpeedSetpoint = 90;
        rightMotorSpeedSetpoint = 90;
        leftMotorSpeed = 90;
        rightMotorSpeed = 90;
        ESCL.write(leftMotorSpeedSetpoint);
        ESCR.write(rightMotorSpeedSetpoint);
        //Serial.printlnf("Warning, GPS or Compass data not available for greater than %dms, exiting autonomous mode", MTR_TIMEOUT);
    }
}

/** @brief ISR timer to check if messages over communication modes have been received. updates flags used to determine which methods to send data over */
void wdogHandler(){
    if(Particle.connected()) LTEAvail = true;   //If particle cloud is connected, assume that LTE is available
    else if(LTEAvail){
        char debugMsg[128];
        snprintf(debugMsg, sizeof(debugMsg), "[WARN] LTE Messages have not been received in %ds, assuming LTE is unavailable", (XBEE_WDOG_AVAIL/1000));
        debugQueue.push(debugMsg, 0, false);
        LTEAvail = false;
    }
    if(millis()-XBeeRxTime > XBEE_WDOG_AVAIL || !XBeeRxTime){   //If the time since the last XBee message is too long, print warning and set status flag to false
        if(XBeeAvail){
            char debugMsg[128];
            snprintf(debugMsg, sizeof(debugMsg), "[WARN] XBee Messages have not been received in %ds, assuming XBee is unavailable", (XBEE_WDOG_AVAIL/1000));
            debugQueue.push(debugMsg, 0, false);
        }
        XBeeAvail = false;
    }
    else XBeeAvail = true;
    if(millis()-BLERxTime > BLE_WDOG_AVAIL || !BLERxTime){      //If the time since the last BLE message is too long, print warning and set status flag to false
        if(BLEAvail && BLERxTime){
            char debugMsg[128];
            snprintf(debugMsg, sizeof(debugMsg), "[WARN] BLE Messages have not been received in %ds, assuming BLE is unavailable", (BLE_WDOG_AVAIL/1000));
            debugQueue.push(debugMsg, 0, false);
        }
        if(BLE.connected() && XBeeAvail) BLEAvail = true;
        else BLEAvail = false;
    }
    else BLEAvail = true;
    if(stopActive && millis() - stopActive > STOP_RST_TIME) stopActive = false;                          //Set stop to false in case the CChub somehow crashed (though we have already entered a "float" mode where drivemode = 0)
}

/** @brief Function to pause operation and copy data off of SD card over bluetooth to the CChub */
void dataOffloader(){
    Serial.println("Entering Data Offloader Mode");
    dataFile.close();
    if (!logDir.open("/")) {
        offloadMode = false;
        Serial.println("Error, could not open root SD card directory");
        return;
    }
    Serial.println("Waiting for CCHub connection...");
    while(!BLE.connected()){
        BLE.advertise(&advData);;
        delay(10);
    }
    Serial.println("Starting transfer...");
    char fileCode[8 + MAX_FILENAME_LEN];
    while (dataFile.openNext(&logDir, O_RDONLY) && BLE.connected()) {
        char namebuf[MAX_FILENAME_LEN];
        dataFile.getName(namebuf,MAX_FILENAME_LEN);
        Serial.printlnf("Checking if file %s is a .csv or .txt...", namebuf);
        if(!strstr(strlwr(namebuf + (strlen(namebuf) - 4)), ".csv")){
            dataFile.close();
            continue;
        }
        else{
            memset(fileCode,0,8+MAX_FILENAME_LEN);
            strcpy(fileCode,"filename");
            strcat(fileCode,namebuf);
            Serial.printlnf("Sending command %s",fileCode);
            offloadCharacteristic.setValue(fileCode);
            Serial.printlnf("File %s is a .csv or .txt printing data", namebuf);
            delay(150);
            noInterrupts();
            while(dataFile.available()){
                char lineBuffer[BLE_OFFLD_BUF];
                memset(lineBuffer,0,BLE_OFFLD_BUF);
                //dataFile.readBytes(lineBuffer,BLE_OFFLD_BUF);
                dataFile.readBytesUntil('\r',lineBuffer,BLE_OFFLD_BUF);
                offloadCharacteristic.setValue(lineBuffer);
                //Serial.println(lineBuffer);
            }
            interrupts();
            memset(fileCode,0,8+MAX_FILENAME_LEN);
            strcpy(fileCode,"filecomp");
            //memcpy(codeBuf,fileCode,8+MAX_FILENAME_LEN);
            offloadCharacteristic.setValue(fileCode);
            delay(150);
            
            dataFile.close();
        }
    }
    logDir.close();
    memset(fileCode,0,8+MAX_FILENAME_LEN);
    strcpy(fileCode,"filedone");
    //memcpy(codeBuf,fileCode,8+MAX_FILENAME_LEN);
    offloadCharacteristic.setValue(fileCode);
    //BLE.disconnect();
    offloadMode = false;
}

/// @brief Function that decodes a sequence of button presses into a command. This does various actions based on the number of presses and the time between them
void buttonActionDecode(){
    //If the button press sequence has started and the last button click time is greater than the idle time, then decode the button presses
    if(buttonPressSequenceStarted && (millis() - lastButtonClickTime > BUTTON_IDLE_TIME)){  
        buttonPressSequenceStarted = false;                //Set the flag to false, so we don't decode the same sequence again
        if(buttonPressCount == 3){
            signalLED = !signalLED;                     //If the button was pressed 8 times, then toggle the signal LED mode
        }
        else if(buttonPressCount == 5){
            logToDebugFile("[INFO] Power off button pressed 5 times, shutting down bot");
            sendData("B1ABsdn",0,true,true,true);         //Send shutdown executed command to CChub
            #ifdef PWR_EN
            digitalWrite(PWR_EN, LOW);                      //If the button was pressed 5 times, then turn off the power to the bot
            #endif
        }
        else if(buttonPressCount == 7){
            doCompassCal = true;                          //If the button was pressed 7 times, then set the compass calibration
            logToDebugFile("[INFO] Compass calibration button pressed 7 times, setting compass calibration mode");
        }
        else if(buttonPressCount == 10){
            logToDebugFile("[INFO] Entering DFU mode, button pressed 10 times");
            System.dfu();              // Enters DFU mode normally
        }
        buttonPressCount = 0;
    }

}

/** @brief ISR triggered when button is pressed or released */
void buttonHandler(){
    if(millis() - lastButtonClickTime < BUTTON_DEB_TIME){  //If the button was pressed within the debounce time, then ignore this press
        lastButtonClickTime = millis();                        //Update the last button click time
        return;
    }
    lastButtonClickTime = millis();                        //Update the last button click time
    buttonPressSequenceStarted = true;
    buttonPressCount++;                                    //Increment the button press count

}

/** @brief ISR timer to update the color and pattern of the LED based on the status of the system */
void LEDHandler(){
    uint32_t SetColor;
    LEDPattern SetPattern;
    LEDSpeed SetSpeed;
    uint8_t statusMode;
    //Special LED Modes
    if(stopActive){         //The user has pressed the stop button on the CChub
        status.setPattern(LED_PATTERN_BLINK);
        status.setColor(RGB_COLOR_YELLOW);
        status.setSpeed(LED_SPEED_FAST);
        return;
    }
    if(millis() - lastCalibrationTime < COMPASS_CAL_TIMEOUT && millis() > COMPASS_CAL_TIMEOUT){ //The bot is currently calibrating the compass
        status.setPattern(LED_PATTERN_BLINK);
        status.setColor(RGB_COLOR_MAGENTA);
        status.setSpeed(LED_SPEED_FAST);
        return;
    }
    if(offloadMode){        //The CChub is offloading data over BLE
        status.setPattern(LED_PATTERN_BLINK);
        status.setColor(RGB_COLOR_BLUE);
        status.setSpeed(LED_SPEED_FAST);
        return;                
    }
    if(signalLED){          //The user has enabled the "Signal" menu item
        status.setPattern(LED_PATTERN_BLINK);
        status.setColor(RGB_COLOR_ORANGE);
        status.setSpeed(LED_SPEED_FAST);
        return;
    }

    //Standard LED signaling patterns
    if(lowBattery){     //Flash the status color quickly if the battery is low
        SetPattern = LED_PATTERN_BLINK;
        SetSpeed = LED_SPEED_NORMAL;
    }
    else if(driveMode == DRIVE_MODE_AUTONOMOUS){    //If we're in autonomous mode, do a fade pattern
        SetPattern = LED_PATTERN_FADE;
        SetSpeed = LED_SPEED_NORMAL;
    }
    else if(driveMode == DRIVE_MODE_SENTRY){    //If we're in sentry, then blink slowly
        SetPattern = LED_PATTERN_BLINK;
        SetSpeed = LED_SPEED_SLOW;
    }
    else{                       //If under manual control, set to solid
        SetSpeed = LED_SPEED_NORMAL;
        SetPattern = LED_PATTERN_SOLID;
    }

    //Standard LED color modes
    statusMode = 0;                 //Set flag counter to 0
    statusMode = LTEAvail;          //Create bit mask with each of the available mode shifted into different bits
    statusMode |= XBeeAvail << 1;
    statusMode |= BLEAvail << 2;
    //Serial.printlnf("Status: %d",statusMode);
    switch (statusMode){
    case 7: //All modes of communication are available
        SetColor = RGB_COLOR_CYAN;
        break;
    case 6: //XBee and BLE are available
        SetColor = RGB_COLOR_YELLOW;
        break;
    case 5: //LTE and BLE are available
        SetColor = RGB_COLOR_WHITE;
        break;
    case 4: //Only BLE available
        SetColor = RGB_COLOR_BLUE;
        break;
    case 3: //XBee and LTE available (typical when out in water)
        SetColor = RGB_COLOR_GREEN;
        break;
    case 2: //Only Xbee available
        SetColor = RGB_COLOR_ORANGE;
        break;
    case 1: //Only LTE is available (recovery mode)
        SetColor = RGB_COLOR_MAGENTA;
        break;
    case 0: //No communication available
        SetColor = RGB_COLOR_RED;
        break;
    default:
        break;
    }
    status.setPattern(SetPattern);
    status.setColor(SetColor);
    status.setSpeed(SetSpeed);    
}

/**
 * @brief Particle function for processing a debug string, for testing things like an emulated GPS point
 * @param cmd Command string received from Particle cloud
 * @returns Always returns 0 (success)
 */
int LTEInputCommand(String cmd){
    char cmdBuf[100];
    cmd.toCharArray(cmdBuf, 100);
    processCommand(cmdBuf, 4,false);
    return 1;
}

/** @brief Function to print a status table showing availability of all communication and hardware systems */
void printStatusTable(){
    // Check if status table printing is enabled
    if(!statusTableEnabled) return;

    // Only print once per second
    static uint32_t lastUpdateTime;
    if(millis() - lastUpdateTime < 500) return;
    lastUpdateTime = millis();

    // Clear the screen and move cursor to top-left
    Serial.print("\033[2J\033[H");
    
    // Print the main system status header
    Serial.println("┌──────────────────────────────────────────────────────────────────────────┐");
    Serial.println("│                         WaterNet23 System Status                        │");
    Serial.println("├──────────────────────────────────────────────────────────────────────────┤");
    Serial.println("│  Component     │  Status  │  Indicator  │                               │");
    Serial.println("├──────────────────────────────────────────────────────────────────────────┤");
    
    // Print each component status
    Serial.printf("│  LTE           │  %s   │     %s      │                               │\n", 
                  LTEAvail ? "ONLINE " : "OFFLINE", 
                  LTEAvail ? "Y" : " ");
    
    Serial.printf("│  XBee          │  %s   │     %s      │                               │\n", 
                  XBeeAvail ? "ONLINE " : "OFFLINE", 
                  XBeeAvail ? "Y" : " ");
    
    Serial.printf("│  BLE           │  %s   │     %s      │                               │\n", 
                  BLEAvail ? "ONLINE " : "OFFLINE", 
                  BLEAvail ? "Y" : " ");
    
    Serial.printf("│  GPS           │  %s   │     %s      │                               │\n", 
                  GPSAvail ? "ONLINE " : "OFFLINE", 
                  GPSAvail ? "Y" : " ");
    
    Serial.printf("│  Compass       │  %s   │     %s      │                               │\n", 
                  CompassAvail ? "ONLINE " : "OFFLINE", 
                  CompassAvail ? "Y" : " ");
    
    Serial.printf("│  SD Card       │  %s   │     %s      │                               │\n", 
                  SDAvail ? "ONLINE " : "OFFLINE", 
                  SDAvail ? "Y" : " ");
    
    Serial.println("├──────────────────────────────────────────────────────────────────────────┤");
    Serial.println("│                            Navigation Data                               │");
    Serial.println("├──────────────────────────────────────────────────────────────────────────┤");
    
    // GPS Position Data (more compact)
    Serial.printf("│  Current: Lat %10.6f  Lon %11.6f                       │\n", latitude, longitude);
    Serial.printf("│  Target:  Lat %10.6f  Lon %11.6f                       │\n", targetLat, targetLon);
    Serial.printf("│  Heading: Compass %6.1f°  Travel %6.1f°  Delta %6.1f°  │\n", compassHeading, travelHeading, headingDelta);
    Serial.printf("|  Motors L:%3d R:%3d                                    │\n", leftMotorSpeed, rightMotorSpeed);

    Serial.println("├──────────────────────────────────────────────────────────────────────────┤");
    Serial.println("│                           System Information                             │");
    Serial.println("├──────────────────────────────────────────────────────────────────────────┤");
    
    // System info (more compact)
    Serial.printf("│  Bot: %2d  Battery: %3d%%  Mode: %d  Uptime: %8lu ms              │\n", 
                  BOTNUM, battPercent, driveMode, millis());
    Serial.printf("│  Last Update: %s                                          │\n", Time.timeStr().c_str());
    
    Serial.println("└──────────────────────────────────────────────────────────────────────────┘");
    Serial.println("═════════════════════════════════════════════════════════════════════════════");
}
