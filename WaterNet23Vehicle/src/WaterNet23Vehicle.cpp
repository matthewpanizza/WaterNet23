/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/Users/mligh/OneDrive/Particle/WaterNet23-GY511/WaterNet23Vehicle/src/WaterNet23Vehicle.ino"
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
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include "LSM303.h"
void setup();
void loop();
void readEEPROM();
void writeEEPROM();
#line 23 "c:/Users/mligh/OneDrive/Particle/WaterNet23-GY511/WaterNet23Vehicle/src/WaterNet23Vehicle.ino"
#define X_AXIS_ACCELERATION 0
//#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
//#include <MicroNMEA.h>                      //http://librarymanager/All#MicroNMEA
#include "SdFat.h"
#include "sdios.h"
#undef min
#undef max
#include <vector>

//////////////////////////////
// BOT CONFIGURATION MACROS //
//////////////////////////////

#define BOTNUM              1           //VERY IMPORTANT - Change for each bot to configure which node in the network this bot is
#define STARTUP_WAIT_PAIR   0           //Set to 1 to wait for controller to connect and discover bots, turns on "advertising" on startup
//#define BLE_DEBUG_ENABLED               //If enabled, will add a BLE characteristic for convenient printing of log messages to a BLE console
#define LEAK_DET_BYPASS     0           //Set to 1 to disable shutdown upon leak detection
#define BATT_TRIG_LEAK      0           //Set to 1 to enable the battery leak cutting off system
//#define VERBOSE

//////////////////////////
// EEPROM Configuration //
//////////////////////////

#define EEPROM_KEY1_LOC         0x00                    //Location of the first key in EEPROM, used to check if the EEPROM is valid
#define EEPROM_KEY2_LOC         0x01                    //Location of the second key in EEPROM, used to check if the EEPROM is valid
#define EEPROM_KEY1             0x23                    //Key for the first byte of the EEPROM, used to check if the EEPROM is valid
#define EEPROM_KEY2             0x129                   //Key for the second byte of the EEPROM, used to check if the EEPROM is valid
#define EEPROM_COMP_CAL_LOC     0x02                    //Location of the compass calibration in EEPROM, used to store the calibration values for the compass


///////////////////////
// Pin Configuration //
///////////////////////

#define ESC_PWM_L           D6          //Left motor ESC output pin
#define ESC_PWM_R           D7          //Right motor ESC output pin
#define SENSE_EN            D2          //Output pin to enable/disable voltage regulator for sensors
#define chipSelect          D8          //Chip select pin for Micro SD Card
#define BATT_ISENSE         A3          //Shunt monitor ADC input for battery supply current
#define SOL_ISENSE          A2          //Shunt monitor ADC input for solar array input current
#define BAT_LEAK_DET        A4          //Digital input for reading battery leak sensor
#define PWR_BUT             A1          //Digital input for reading power button input
#ifdef A6
#define BATT_VSENSE         A6          //Voltage divider ADC input for reading power rail (battery) voltage
#endif
#ifdef D22
#define PWR_EN              D22         //Digital output for latching power mosfet on until shutoff
#endif
#ifdef D23
#define LEAK_DET            D23         //Digital input for on-PCB leak detection trace
#endif

/////////////////////////
// Compass Calibration //
/////////////////////////

#define COMPASS_TYPE            0           //0 = LSM303DLHC, 1 = LIS3MDL

#define COMPASS_TYPE_LSM303     0           //Value for COMPASS_TYPE to indicate LSM303DLHC    
#define COMPASS_TYPE_LIS3MDL    1           //Value for COMPASS_TYPE to indicate LIS3MDL

#define COMP_OFFSET 0                       //Number of degrees to add to the raw compass reading to calibrate it to true north. This is a constant offset, not a full calibration

#define COMP_CAL_AVG_COUNT      5         //Number of samples to average for the compass calibration

////////////////////
// PROGRAM MACROS //
////////////////////

#define SCAN_RESULT_COUNT   20

#define PHADDR              99              //default I2C ID number for EZO pH Circuit.
#define MCOND               100             //default I2C ID number for EZO Mini-Conductivity (0.1)
#define COND                101             //default I2C ID number for EZO Conductivity Circuit. (1.0)
#define TEMPADDR            102             //Default I2C address for temperature sensor
#define DOADDR              97              //Default I2C address for Dissolved Oxygen sensor
#define SENS_POLL_RT        2500            //Number of milliseconds between sensor reads
#define SENS_DATA_DLY       825             //Number of milliseconds between a request to a sensor and actually retrieving the reading

#define SHUTDOWN_HOLD       3000            //Number of milliseconds that the power button must be held to actually shut off bot
#define WATCHDOG_PD         1000           //Watchdog timer period in milliseconds
#define STATUS_PD           10000            //Time between status updates published to CC Hub
#define STOP_RST_TIME       10000           //Time after receiving the last stop command to exit stop mode
#define XBEE_WDOG_AVAIL     5000           //Watchdog interval between XBee messages for availablility check
#define BLE_WDOG_AVAIL      5000           //Watchdog interval between BLE messages for availability check
#define LTE_MAX_STATUS      480             // (Divided by LTE STAT PD) Maximum number of status messages to send over LTE if other methods are unavailable
#define LTE_STAT_PD         4               //Divider for sending status via LTE to reduce data usage
#define XBEE_START_PUB      5000            //Time period between sending "Hello World" messages over XBee during setup
#define MANUAL_RAMP_PD      30             //Time period between motor ramp updates when in manual motor drive mode

#define DEF_FILENAME        "WaterBot"
#define FILE_LABELS         "Time,Latitude,Longitude,Temperature,pH,Dissolved O2,Conductivity 0.1K,Conductivity 1K"
#define BLE_OFFLD_BUF       100
#define CUSTOM_DATA_LEN     8
#define MAX_FILENAME_LEN    32

/////////////////////////
// Power System Macros //
/////////////////////////

#define BAT_MIN             13.2            //Voltage to read 0% battery
#define BAT_MAX             16.4            //Voltage to read 100% battery
#define LOW_BATT_PCT        20              //Voltage to set low battery flag
#define VDIV_MULT           0.004835        //Calculate the ratio for ADC to voltage conversion 3.3V in on ADC = 4095 3.3V on 100kOhm + 20kOhm divider yields (3.3/20000)*120000 = 19.8V in MAX
#define BAT_ISENSE_MULT     33.0            //Calculate the maximum current the shunt can measure for the battery. Rs = 0.001, RL = 100k. Vo = Is * 0.1, max current is 33A
#define SLR_ISENSE_MULT     16.5            //Calculate the maximum current the shunt can measure for the solar array. Rs = 0.010, RL = 20k. Vo = Is * 0.2, max current is 16.5A

////////////////////////
// Motor Drive Macros //
////////////////////////

#define MTR_IDLE_ARM        2000            //Number of milliseconds to hold motors stopped for arming
#define MTR_ST_FWD          100             //Minimum commanded speed for motors going forward
#define MTR_ST_REV          80              //Minimum commanded speed for motors in reverse
#define MTR_TIMEOUT         4000            //Timeout in milliseconds for turning off motors when being manually controlled
#define MTR_RAMP_SPD        3               //Rate to ramp motor speed to target speed (step size for going between a value somewhere between 0 and 180)
#define MTR_RAMP_TIME       50              //Time between ramp iterations
#define MTR_TRAVEL_SPD      0.45             //Percentage maximum travel speed for autonomous movement default
#define MTR_CUTOFF_RAD      1.5             //Radius to consider "arrived" at a target point
#define SENTRY_IDLE_RAD     4.0             //Radius to keep motors off in sentry mode after reaching the cutoff radius
#define GPS_POLL_TIME       990             //Rate to poll the GPS and calculate the distance
#define COMP_POLL_TIME      250             //Rate to poll the Compass and calculate the target heading



SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

/////////////////////////
// Function Prototypes //
/////////////////////////

void processCommand(const char *command, uint8_t mode, bool sendAck);
void cmdLTEHandler(const char *event, const char *data);                    //ISR Function to take in a command string received over Cellular and process it using the proccessCommand dictionary
void setupXBee();
bool setupCompass();
void compassCalibration();
void setupGPS();
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
void updateMotors();
void sendData(const char *dataOut, uint8_t sendMode, bool sendBLE, bool sendXBee, bool sendLTE);
void printBLE(const char *dataOut);
void StatusHandler();
void sensorHandler();
void XBeeHandler();
static void BLEDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void motionHandler();
void wdogHandler();
void dataOffloader();
void buttonTimer();
void buttonHandler();
void logMessage(const char *message);
void LEDHandler();
int LTEInputCommand(String cmd);


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
Timer shutdownTimer(SHUTDOWN_HOLD, buttonTimer);    //Create timer for shutdown, which runs when the button is pressed to calculate if the button has been held for SHUTDOWN_HOLD seconds 



//////////////////////
// Global Variables //
//////////////////////

SFE_UBLOX_GNSS myGPS;                           //GPS Buffer and Objects
//char nmeaBuffer[100];
//MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
//SFE_UBLOX_GPS myGPS;

Adafruit_LIS3MDL lis3mdl;                 //Compass object for LIS3MDL
LSM303 lsm303;                            //Compass object for LSM303DLHC

LEDStatus status;                               //LED Control object

//SD File system object                 
SdFat sd((SPIClass*)&SPI);                      //SD card object, initialized on SPI for the Beta/Alpha PCB, SPI1 on the Bsom breakout board

File myFile;                                    //File for the sensor data
File logFile;                                   //File for messages logged by the program
File logDir;                                    //File directory 

SerialLogHandler logHandler(LOG_LEVEL_INFO);    //Log Configuration

Servo ESCL;                                     //Object for servo esc of left motor 
Servo ESCR;                                     //Object for servo esc of right motor


bool waitForConnection;                                                 //Flag used on startup until the CC acknowledges this bot
float latitude, longitude;                                              //Globals to hold the latitude and longitude read in from the GPS
float compassHeading; 
float travelHeading, targetDelta;                       //Compassheading is the calibrated compass reading relative to north, travel heading is the heading between current point and target point
float targetLat, targetLon;                                             //Globals to hold the latitude and longitude sent from the CC for where the bot should target
float travelDistance;                                                   //Global to hold the distance between the current latitude and longitude and the target latitude and longitude
bool telemetryAvail;                                                    //Boolean global to check if the compass and GPS are available
uint8_t leftMotorSpeed, setLSpeed;                                      //Global for the left motor speed and left motor target speed
uint8_t rightMotorSpeed, setRSpeed;                                     //Global for the right motor speed and right motor target speed
float autoMoveRate = MTR_TRAVEL_SPD;                                    //Set the move rate of the motors, which makes the motor base speed faster/slower
bool pointArrived;                                                      //Global to indicate that the target point has been reached
bool warnedBattLeak, warnedLeak;                                        //Flags to indicate a leak warning has been published to avoid publishing too frequently
uint8_t battPercent;                                                    //Global battery percentage indicator
float battVoltage, battCurrent, solarCurrent;                           //Global battery and solar current read in from power system
bool updateMotorControl;                                                //Flag to update the motor control, which ramps speed slowly
uint8_t driveMode = 0;                                                  //Global mode for drive mode, 0 = manual remote control, 1 = sentry, 2 = autonomous
bool lowBattery;                                                        //Flag to indicate that the battery is low
bool statusReady;                                                       //Flag to indicate that a status is ready
uint8_t requestActive;                                                  //Flag to indicate that a sensor request has been made from the CChub
uint16_t LTEStatusCount;                                                //Counter to determine number of LTE messages that should be sent to limit data usage
uint16_t statusFlags;                                                   //Global status flag
bool LTEAvail, XBeeAvail, BLEAvail, GPSAvail, CompassAvail, SDAvail;    //Flags for communicaton keep-alives/available
bool logSensors, logMessages, dataWait;                                 //Flags for sensor timing/enables
bool offloadMode;                                                       //Flag to indicate that SD card data is being offloaded
bool signalLED;                                                         //Flag to indicate that the CC hub has requested that the LED should be signaling flashing orange
bool shutdownActive;                                                    //Flag to indicate that the button has been pressed down and a shutdown is initiated
bool stopActive;                                                        //Flag to indicate that the CChub has had a stop hit
uint32_t senseTimer, dataTimer, positionTimer, compassTimer;            //Timers for reading from the GPS and compass
uint32_t XBeeRxTime, BLERxTime;                                         //Timers for when the last valid Xbee and BLE message was received
uint32_t motionTime, lastMtrTime, lastTelemTime;                                    //Timers for when the last motor and telemetry commands were received
uint32_t lastStatusTime;                                                //Timer for when the last status control packet was received
uint32_t stopTime;
float sensePH, senseTemp, senseCond, senseMCond, senseDO;               //Global variables for holding sensor data received from last Atlas sensors
char filename[MAX_FILENAME_LEN];                                        //Filename for the file holding sensor data
char filenameMessages[MAX_FILENAME_LEN];                                //Filename for the file holding log messages
double varCompassHead;
double rawHead;
uint32_t BLEdbgTimer;
float last_lat, last_lon;
int compOffset = COMP_OFFSET;                                           //Offset for the compass calibration. Degrees off north to add to the compass reading to calibrate it to true north
bool doCompassCal = false;                                              //Flag to indicate that the compass calibration has been requested

//Dictionary for all bot commands that is called when XBee, BLE, and LTE strings are received. Mode 1 - BLE, Mode 2 - XBEE, Mode 4 - LTE
void processCommand(const char *command, uint8_t mode, bool sendAck){
    //Process if command is addressed to this bot "Bx" or all bots "AB"
    if((command[2] == 'B' && command[3] == BOTNUM+48) || (command[2] == 'A' && command[3] == 'B')){     //Check if the message was addressed to this bot, otherwise do nothing with it
        uint8_t checksum;                   //Integer checksum that is populated from the number at the end of the string
        char dataStr[strlen(command)-8];    //String to hold the data section of the message, cuts off the address, command, and checksum characters
        dataStr[strlen(command)-9] = '\0';  //Put null terminator at end of data string, otherwise string operators will flow into surrounding memory (a bug that cost many hours in debugging)
        char cmdStr[4];                     //String to hold the three command characters, plus a null terminator to enable string compare to find the end of the string
        cmdStr[3] = '\0';                   //Set null at end of command string
        char checkStr[3] = {command[strlen(command)-2], command[strlen(command)-1], '\0'};  //Get checksum string from last two characters
        checksum = (uint8_t)strtol(checkStr, NULL, 16);       //Convert string to number, with base 16 (hex) from string
        #ifdef VERBOSE
        Serial.printlnf("Checksum: %02x, %03d",checksum,checksum);
        #endif
        for(uint8_t i = 4; i < strlen(command)-2;i++){      //Copy in data characters from overall string
            if(i < 7) cmdStr[i-4] = command[i];
            else dataStr[i-7] = command[i];
        }
        if(checksum != strlen(command)-2){      //Check if the received checksum matches the length of the string received
            #ifdef VERBOSE
            Serial.printlnf("String Len: %d, Checksum: %d",strlen(command)-2,checksum); //Print to console
            Serial.println("Warning, checksum does not match");
            #endif
            if(!logFile.isOpen()){  //Print to SD Card
                logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
                logFile.printlnf("[WARN] Message Checksum Does Not Match!: %s",command);
                logFile.close();
            }
            else logFile.printlnf("[WARN] Message Checksum Does Not Match!: %s",command);
            return;     //Don't process the command
        }
        if(!strcmp(cmdStr,"ctl")){      //Control command from CC that contains data about the drive mode, target latitude and longitude, and offloading
            char tLat[10];              //String buffer for latitude, as sscanf doesn't handle floats well
            char tLon[10];              //String buffer for longitude, as sscanf doesn't handle floats well
            sscanf(dataStr,"%s %s %d %d %d",tLat,tLon,&driveMode,&logSensors,&signalLED);    //Target lat, target lon, drive mode, dataRecord, signal
            targetLat = atof(tLat);     //Convert latitude string to float
            targetLon = atof(tLon);     //Convert longitude string to float
            Serial.printlnf("New target GPS, Lat: %f Lon: %f", targetLat, targetLon);
        }
        if(!strcmp(cmdStr,"mtr")){  //Motor Speed Control
            char lSpd[3] = {dataStr[0],dataStr[1],dataStr[2]};  //Get the first three characters of the data for the left target speed
            char rSpd[3] = {dataStr[3],dataStr[4],dataStr[5]};  //Get the second three characters of the data for the right target speed
            setLSpeed = atoi(lSpd);                             //Convert string to integer in global target speed, motor speed is ramped to new target by updateMotors
            setRSpeed = atoi(rSpd);                             //Convert string to integer in global target speed, motor speed is ramped to new target by updateMotors
            Serial.printlnf("Received Motor Command: LSpeed=%d,RSpeed=%d",setLSpeed,setRSpeed);
            //updateMotorControl = true;      //Set flag to indicate to updateMotors that a new speed has been received
            lastMtrTime = millis();         //Update timer for the watchdog that a motor speed was received from CC hub
            driveMode = 0;                  //In case we missed the switch from an autonomous to manual mode, switch to manual mode
        }
        else if(!strcmp(cmdStr,"req")){     //Data Request from CChub to get the bundle of sensor data and transmit it out
            requestActive = mode;           //Set flag, as it's not possible to use 2/3 communication modes in an interrupt handler (which is where processCommand is called from)
        }
        else if(!strcmp(cmdStr,"pts")){     //Command used for debugging, which allows the CChub (or any bluetooth device) to print a string to the console and to the SD card
            Serial.println(dataStr);        //Print to console
            if(!logFile.isOpen()){          //Print to SD card
                logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
                logFile.printlnf("[PUTS] Received String Command: %s",dataStr);
                logFile.close();
            }
            else logFile.printlnf("[PUTS] Received String Command: %s",dataStr);
        }
        else if(!strcmp(cmdStr,"spc")){         //Incoming communication status from CChub, this data is used in addition to control strings to determine which communication methods are available between this bot and CChub
            lastStatusTime = millis();          //Update timer with the current time, and the watchdog will automatically set the flags based on this timer and the current time
        }
        else if(!strcmp(cmdStr,"hwa")){         //Hello-world acknowledge command from the CCHub, which will bring this bot out of pairing mode on startup
            waitForConnection = false;          //Setup loop waits for this to be set true before moving into main loop
        }
        else if(!strcmp(cmdStr,"dmp")){         //Enter SD Card "Dump Mode" for Bluetooth offloading
            offloadMode = true;                 //Set flag for offloading mode, which is checked by the main loop
            status.setPattern(LED_PATTERN_BLINK);   //Set the LED pattern immediately so the user can tell that it has successfully entered offloading mode
            status.setColor(RGB_COLOR_BLUE);
            status.setSpeed(LED_SPEED_FAST);
        }
        else if(!strcmp(cmdStr,"cmp")){         //Command to calibrate the compass, which is used to set the offset for the compass heading
            doCompassCal = true;               //Set flag to indicate that the compass calibration
        }
        else if(!strcmp(cmdStr,"egp")){         //Emulated GPS point for testing purposes. Spoofs the GPS latitude and longitude which allows testing of the distance and bearing functions without hardware
            char tLat[12];                      //Strings for the latitude and longitude, as sscanf cannot handle floats very well, copies string then converts to a float using atof()
            char tLon[12];
            sscanf(dataStr,"%s %s",tLat,tLon);      //Scan in the target latitude and longitude from the data string
            latitude = atof(tLat);              //Convert strings with latitude and longitude to a float variable
            longitude = atof(tLon);
        }
        else if(!strcmp(cmdStr,"stp")){         //Stop Command (Emergency stop for motors)
            driveMode = 0;                      //Set drive mode back to manual mode
            setLSpeed = 90;                     //Stop motors
            setRSpeed = 90;                 
            leftMotorSpeed = 90;                //Immediately stop motors (no ramp)
            rightMotorSpeed = 90;
            ESCL.write(90);                     //Immediately write to the ESC a stopped state
            ESCR.write(90);
            stopTime = millis();
            stopActive = true;                  //Set flag to indicate that stop was hit
        }
    }
}

//ISR Function to take in a command string received over Cellular and process it using the proccessCommand dictionary
void cmdLTEHandler(const char *event, const char *data){
    processCommand(data, 4,false);      //Pass received string directly to the processCommand directory
    if(logMessages){                    //Log message to the SD card for later debugging
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[INFO] Received LTE Message: %s",data);
        logFile.close();
    }
}

//Function that is called by the system once upon startup. Initializes variables used by the system as well as all hardware like the SD card, GPS, sensors, XBee
void setup(){
    status.setPriority(LED_PRIORITY_IMPORTANT);         //First set the LED color to green to indicate to the user that the system has properly started up
    status.setColor(RGB_COLOR_GREEN);
    status.setPattern(LED_PATTERN_SOLID);
    status.setActive(true);                             //Set the LED status to active to change from Particle's LED scheme to the custom patterns
    
    //Pin configuration
    pinMode(SENSE_EN, OUTPUT);                          //Configure the pin for the Atlas sensors as an output and pull low to enable power to the Atlas sensors
    digitalWrite(SENSE_EN,LOW);                     
    pinMode(PWR_BUT, INPUT);                            //Configure power button input as an input, no pull as the resistor divider will handle pin floating
    attachInterrupt(PWR_BUT, buttonHandler, CHANGE);    //Attach the buttonHandler function to trigger whenever the button is pressed or released
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

    Particle.connect();

    readEEPROM();                               //Read the EEPROM to get the compass calibration
    
    uint32_t mtrArmTime = millis();             //Create a timer to make sure the motors are initialized to 90 (stopped) for at least 2 seconds, otherwise ESC will not become armed
    leftMotorSpeed = setLSpeed = 90;            //Set the initial left motor speed of 90, which is stopped. The controller must be held here for 2 seconds to arm the ESC
    rightMotorSpeed = setRSpeed = 90;           //Set the initial right motor speed of 90, which is stopped. The controller must be held here for 2 seconds to arm the ESC
    ESCL.attach(ESC_PWM_L,1000,2000);           //Start PWM-based ESC, with 1ms min pulse width and 2ms max pulse width
    ESCR.attach(ESC_PWM_R,1000,2000);           //Start PWM-based ESC, with 1ms min pulse width and 2ms max pulse width
    ESCL.write(setLSpeed);                      //Set the initial speed of the left motor
    ESCR.write(setRSpeed);                      //Set the initial speed of the right motor
    if(!STARTUP_WAIT_PAIR) delay(2000);         //Delay for 2 seconds to allow motor controllers to arm

    BLE.on();                                   //Turn on Bluetooth
    BLE.setTxPower(8);                          //Max transmitting power
    
    Serial.begin(115200);
    Serial1.begin(9600);                        //Start serial for XBee module
    setupXBee();                                //Setup XBee module
    setupGPS();                                 //Setup GPS module
    
    Particle.variable("Heading", varCompassHead);
    Particle.variable("TargetHead", rawHead);
    //Particle.variable("Distance", travelDistance);
    //Particle.variable("Lat", latitude);
    //Particle.variable("Lon", longitude);

    Particle.subscribe("CCHub", cmdLTEHandler); //Subscribe to LTE data from Central Control Hub
    Particle.function("Input Command", LTEInputCommand);        //Debug function to feed in commands over LTE
    LTEAvail = false;                           //Initialize LTE status indicator to false until we receive a message from CC
    SDAvail = true;                             //SD initialized to true, but is set false when the SD is initialized unsucessfully
    BLEdbgTimer = compassTimer = motionTime = stopTime = positionTimer = lastTelemTime = lastStatusTime = dataTimer = senseTimer = millis();     //Initialize most software timers here to current time
    XBeeRxTime = 0;                             //Initialize timer for checking that XBee is available
    BLERxTime = 0;                              //Initialize timer for checking that BLE is available
    dataWait = false;                           //Set false initially to first request data to sensors before attempting to read data
    logSensors = true;                          //By default, log sensor data to SD card, if SD card is inserted
    logMessages = true;                         //By default, log debug messages to SD card, if SD card is inserted
    offloadMode = false;                        //Set offload to false, otherwise could try to offload with no CC connected
    requestActive = false;                      //Set request to false, otherwise we are trying to send sensor data with no CC connected
    LTEStatusCount = LTE_MAX_STATUS;            //Initialize counter for LTE backup messages. This counter limits the number of LTE messages being sent so we don't burn through the data limit
    telemetryAvail = false;                     //Initially assume GPS and compass are not available until we receive the first point from each
    shutdownActive = false;                     //Initially set the shutdown state to false until the button is pressed and held for 3 seconds, then shut off
    stopActive = false;                         //Initially disable stop, otherwise user might not know that the stop button must be pressed
    warnedBattLeak = false;                     //Initially set false so at least one leak message will be sent in the case of a leak
    warnedLeak = false;                         //Initially set false so at least one leak message will be sent in the case of a leak

    battPercent = 50;                           //Initially set battery reading to 50% until we read the actual voltage so the LED states are not affected 

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

    Wire.begin();
    Wire.setClock(CLOCK_SPEED_400KHZ);

    CompassAvail = setupCompass();

    char timestamp[16];                         //String that holds a timestamp for naming the files generated on the SD card
    snprintf(timestamp,16,"B%d%02d%02d%04d%02d%02d%02d", BOTNUM, Time.month(),Time.day(),Time.year(),Time.hour(),Time.minute(),Time.second());
    strcpy(filename,DEF_FILENAME);              //Copy in all of the necessary elements of the file name
    strcat(filename,timestamp);
    strcpy(filenameMessages,filename);
    strcat(filename,".csv");
    strcat(filenameMessages,"_LOG.txt");

    Serial.println(filename);                   //Print the filenames to the console for debugging
    Serial.println(filenameMessages);

    watchdog.start();                           //Start the timers
    //motionTimer.start();
    ledTimer.start();
    statusPD.start();

    if (!sd.begin(chipSelect, SD_SCK_MHZ(8))) {     //Try to connect to the SD card
        Serial.println("Error: could not connect to SD card!");     //If not, warn the user in the console
        logSensors = false;                         //Set flags that the SD card is not available which will warn the user on the CChub
        logMessages = false;
        SDAvail = false;
    }
    if(logSensors){                                 //Logsensors enables logging of sensor data, if enabled, then create the file on the SD card
        myFile.open(filename, O_RDWR | O_CREAT | O_AT_END);
        myFile.println(FILE_LABELS);
        myFile.close();
    }
    if(logMessages){                                //Logsensors enables logging of messages, if enabled, then create the file on the SD card
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[INFO] WaterBot %d: Started Logging!",BOTNUM);
        logFile.close();
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
}

//Function called by the system that continuously loops as long as the device is on. Interrupts will pause this, execute what they are doing (change flags monitored here) and then return control here
void loop(){
    //Serial.printlnf("Time: %d", millis());
    compassCalibration();    //Check if the compass calibration has been requested, and if so, run the calibration function
    getPositionData();      //Grab position data from GPS and Compass
    readPowerSys();         //Read power from battery and solar panel
    sensorHandler();        //Read and request data from Atlas sensor
    XBeeHandler();          //Check if a string has come in from XBee
    statusUpdate();         //Check if a status update has to be sent out
    //updateMotors();         //Update the motor speeds dependent on the mode
    if(offloadMode) dataOffloader();    //Check if a signal to offload has been received
    sendResponseData();     //Send sensor data if requested from the CC
    varCompassHead = (double)compassHeading;
    //rawHead = (double) targetDelta;
    delay(3);              //Slow down the program a little bit, 10ms per loop
    //Serial.printlnf("Hello World, %d", millis());
}

//Reads from the EEPROM if it is properly formatted. Updates the EEPROM if formatting does not match
void readEEPROM(){
    //Check if the EEPROM has been configured for use by the CAN analyzer. Read memory items if the keys match
    if(EEPROM.read(EEPROM_KEY1_LOC) == EEPROM_KEY1 && EEPROM.read(EEPROM_KEY2_LOC) == EEPROM_KEY2){
        EEPROM.get(EEPROM_COMP_CAL_LOC, compOffset);    //Read the compass calibration
    }
    //Otherwise, write the default values and the keys for the CAN Analyzer so the EEPROM is set up for the next time
    else{
        EEPROM.put(EEPROM_COMP_CAL_LOC, compOffset);    //Write the compass calibration value to the EEPROM
        EEPROM.write(EEPROM_KEY1_LOC, EEPROM_KEY1);
        EEPROM.write(EEPROM_KEY2_LOC, EEPROM_KEY2);
    }
}

void writeEEPROM(){
    EEPROM.put(EEPROM_COMP_CAL_LOC, compOffset);    //Write the compass calibration
}

//Code to initially configure XBee module over serial. Sends newline and 'B' to bypass the microcontroller onboard the XBee module.
void setupXBee(){
    Serial1.printf("\n");    //First character to set Bypass mode
    delay(20);              //Wait some time before sending next character
    Serial1.printf("B");     //Second character to set Bypass mode
    delay(20);
    //Serial1.printf("Hello from Bot %d\n", BOTNUM);   //Send Hello World message!
}

//This function gets called from the SparkFun Ublox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
//void SFE_UBLOX_GPS::processNMEA(char incoming){
  //Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  //nmea.process(incoming);
//}

//Function to initialize the compass (LIS3MDL or LSM303) and set the parameters for the compass
bool setupCompass(){

    if(COMPASS_TYPE == COMPASS_TYPE_LIS3MDL){
        if (! lis3mdl.begin_I2C()) {                // hardware I2C mode, can pass in address & alt Wire
            Serial.println("Failed to find LIS3MDL chip");   //Couldn't connect over I2C, so assume the compass is unavailable. Flag disables Autonomous/Sentry mode
            return false;
        }
        else Serial.println("LIS3MDL Found!");
        lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);
        lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
        lis3mdl.setRange(LIS3MDL_RANGE_8_GAUSS);
        lis3mdl.setIntThreshold(500);
        lis3mdl.configInterrupt(false, false, true, // enable z axis
                              true, // polarity
                              false, // don't latch
                              true); // enabled!
    }
    else if(COMPASS_TYPE == COMPASS_TYPE_LSM303){
        if(!lsm303.init()) return false;
        lsm303.enableDefault();

        /*
        Calibration values; the default values of +/-32767 for each axis
        lead to an assumed magnetometer bias of 0. Use the Calibrate example
        program to determine appropriate values for your particular unit.
        */
        lsm303.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
        lsm303.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
    }
    else{
        return false;
    }
    return true;
}

//I2C setup for NEO-M8U GPS
void setupGPS(){
    GPSAvail = true;
    /*myGPS.begin(Wire);
    if (myGPS.isConnected() == false){
        //Log.warn("Ublox GPS not detected at default I2C address, freezing.");
        GPSAvail = false;
    }*/
    if(myGPS.begin() == false){
        GPSAvail = false;
        Serial.println("Error, Could not initialize GPS");
    }
    myGPS.setI2COutput(COM_TYPE_UBX);
    myGPS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX);
    myGPS.setNavigationFrequency(2);
    Wire.setClock(400000); //Increase I2C clock speed to 400kHz
}

//Checks if the remote control has requested a compass calibration and reads the raw heading to calculate the offset
void compassCalibration(){
    if(doCompassCal){
        float sum = 0;
        for(int i = 0; i < COMP_CAL_AVG_COUNT; i++){
            sum += getRawCompassHeading();    //Get the raw compass heading from the compass
            delay(10);    //Delay for 10ms between readings to allow the compass to stabilize
        }
        float avg = sum / (float)COMP_CAL_AVG_COUNT;    //Average the compass heading over the number of samples
        compOffset = (int) avg;                 //Set the compass offset to the average heading
        writeEEPROM();                     //Write the compass offset to the EEPROM so it is saved for next time
        Serial.printlnf("Compass Calibration: %d",compOffset);    //Print the compass calibration to the console for debugging
        doCompassCal = false;    //Set flag to false so we don't keep calibrating the compass
    }
}

//Function to read power draw of the system and check for leaks. Updates global variables for power system. 
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
        delay(50);                                                          //wait 50ms for data to go out
        if(!LEAK_DET_BYPASS) digitalWrite(PWR_EN,LOW);                      //kill system
        warnedLeak = true;                                                  //Set flag to not spam console in case cutoff doesn't work
    }
    if(!digitalRead(BAT_LEAK_DET) && !warnedBattLeak){                      //BAT_LEAK_DET pin is pulled low when a leak is detected
        char warnChar[12];                                                  //String to hold transmitted string
        if(!LEAK_DET_BYPASS && BATT_TRIG_LEAK) sprintf(warnChar,"B%dCCldb",BOTNUM);
        else sprintf(warnChar,"B%dCCwlb",BOTNUM);                           //Message to warn leak in battery
        sendData(warnChar,0,true,true,true);                                //Send data out over all transmission methods regardless
        delay(50);                                                          //wait 50ms for data to go out
        if(!LEAK_DET_BYPASS && BATT_TRIG_LEAK) digitalWrite(PWR_EN,LOW);    //kill system
        warnedBattLeak = true;
    }
    #endif
    return battPercent;
}

//Convert degrees to radians, used by the compass bearing calculation
float deg2rad(float deg) {
  return deg * (3.14159/180);   //Multiply by Pi/180
}

//Function to take an x and y acceleration from the compass and return a raw value between -180 and +180 degrees
float lis3mdlCompassHeading(float x_accel, float y_accel){
    float rawHeading = atan2(y_accel, x_accel) * 180.0 / M_PI;  //Convert x and y compass acceleration to a heading
    #ifdef VERBOSE
    //Serial.printlnf("Raw Heading: %f", rawHeading);
    #endif
    return rawHeading;   //Call the remap function to get a calibrated heading
}

//function that takes two latitudes and longitudes and calculates the distance (in meters) between them. Used by autonomous system for determining arrival at a point
float calcDistance(float lat1, float lat2, float lon1, float lon2){
    float dLat = deg2rad(lat1-lat2);    //Calculate difference between latitudes
    float dLon = deg2rad(lon1-lon2);    //Calculate difference between longitudes
    float a = sinf(dLat/2) * sinf(dLat/2) + cosf(deg2rad(lat2)) * cosf(deg2rad(lat1)) * sinf(dLon/2) * sinf(dLon/2);    //Formula from a stackexchange post
    float c = 2 * atan2(sqrt(a), sqrt(1.0-a)); 
    return 6371000.0 * c; // Distance in m
}

//Functio that takes compass and target headings from compass and two gps points and calculates which way the bot should rotate in order to get to that target point
float calcDelta(float compassHead, float targetHead){
    //Do math and comparisons to produce an output heading between -180 and 180, where positive values rotate the bot clockwise, and negative counterclockwise
    if(targetHead > 0){ 
        if(compassHead > 0){
            return targetHead - compassHead;
        }
        else{
            float diff = -(180.0 - targetHead);
            if(diff < compassHead) return targetHead - compassHead;
            else return 0 - (180.0 + compassHead) - (180.0 - targetHead);
        }
    }
    else{
        if(compassHead > 0){
            float diff = 180.0 + targetHead;
            if(diff > compassHead) return targetHead - compassHead;
            else return (180.0 - compassHead) + (180.0 + targetHead);
        }
        else{
            return targetHead - compassHead;
        }
    }
}

//Function to get the raw compass heading from the compass module (0-360). This is used for debugging and testing purposes, as well as for the autonomous system to determine which way to turn
float getRawCompassHeading(){
    float rawHeading = 0;     //Create a variable to hold the heading from the compass, regardless
    if(COMPASS_TYPE == COMPASS_TYPE_LIS3MDL){
        lis3mdl.read();                                 // get X Y and Z data at once
        sensors_event_t event;                          //"Event" for compass reading which contains x and y acceleration
        bool CompassAvail = lis3mdl.getEvent(&event);   //Get event data over I2C from compass
        if(CompassAvail) rawHeading = lis3mdlCompassHeading(event.magnetic.x,event.magnetic.y);
        if(rawHead < 0) rawHead += 360;   //If the heading is negative, add 360 to it to get a positive value
    }
    else if(COMPASS_TYPE == COMPASS_TYPE_LSM303){
        lsm303.read();                              //Read the compass data from the LSM303 over I2C
        rawHeading = lsm303.heading();        //Library automatically converts to degrees
    }
    Serial.printlnf("Raw Heading: %0.2f", compassHeading); 
    return rawHeading;   //Return the raw heading from the compass module
}

//Function to get the calibrated compass heading, which is used by the autonomous system to determine which way to turn
float getCalibratedCompassHeading(){
    float cHeading = getRawCompassHeading();   //Get the raw compass heading from the compass module
    cHeading -= compOffset;   //Add the offset to the compass heading to get the calibrated heading
    if(cHeading > 180) cHeading -= 360;
    else if(cHeading < -180) cHeading += 360;
    if(targetLat >= -90 && targetLat <= 90 && targetLon >= -90 && targetLon <= 90){         //Check that the target latitude and longitude are valid
        travelHeading = (atan2(targetLon-longitude, targetLat-latitude) * 180 / M_PI);      //Calculate the heading between the current and target location
        travelDistance = calcDistance(targetLat,latitude,targetLon,longitude);              //Calculate the distance between the current and target location
        targetDelta = calcDelta(cHeading, travelHeading);                             //Calculate delta to control angle of the bot
        lastTelemTime = millis();                                                           //Update telemetry time
        if(CompassAvail) telemetryAvail = true;                                             //If compass and GPS are available, set flag to true
        //rawHead = (double) targetDelta;
        //char tempbuf[200];
        //sprintf(tempbuf,"Lat: %f Lon %f TLa: %f TLo: %f, Compass: %f, Trv hd: %f, Trv Del: %f, Dst: %f, L:%d, R: %d", latitude, longitude, targetLat, targetLon, compassHeading, travelHeading, targetDelta, travelDistance, setLSpeed, setRSpeed);
        //printBLE(tempbuf);
        #ifdef VERBOSE
            Serial.printlnf("Head: %0.2f, Target: %0.2f, Delta: %0.2f", cHeading, travelHeading, targetDelta);
        #endif
    }  
    return cHeading;
}

//Function to read data from the GPS and compass module and then call the distance calculation functions for updating autonomous movement
void getPositionData(){
    if(millis() - positionTimer > GPS_POLL_TIME){       //Use a timer to slow the poll rate on GPS and Compass, as they do not same that quickly
        positionTimer = millis();                       //Reset timer
        if(myGPS.isConnected()){                        //Only read from GPS if it is connected
            latitude = ((float)myGPS.getLatitude())/10000000.0;      //Get latitude and divide by 1000000 to get in degrees
            longitude = ((float)myGPS.getLongitude())/10000000.0;    //Get longitude and divide by 1000000 to get in degrees
            Serial.printlnf("Lat: %0.7f Lon: %0.7f", latitude, longitude);
            GPSAvail = true;
        }
        else GPSAvail = false;                          //Set flag to indicate GPS unavailable if not connected
        //GPSAvail = true;
        //latitude = 35.77185;
        //longitude = -78.67415;
    }
    if(millis() - compassTimer > COMP_POLL_TIME){
        compassTimer = millis();                       //Reset timer
        compassHeading = getCalibratedCompassHeading();   //Get the calibrated compass heading
        #ifdef VERBOSE
            Serial.printlnf("Compass Heading: %0.2f", compassHeading);   //Print the heading to the console for debugging
        #endif
    }
}

//Function to check if response data to a request needs to be sent out
void sendResponseData(){
    if(requestActive){              //If the CC has requested data using the req command
        char responseStr[65];       //Create string to hold sensor data
        memset(responseStr,0,65);   //Empty the string if it had something
        sprintf(responseStr,"B%dCCsns%0.6f %0.6f %d %d %d %d %d ",BOTNUM,latitude,longitude,(int)(senseDO*1000),(int)(sensePH*1000),(int)(senseCond*1000),(int)(senseMCond*1000),(int)(senseTemp*1000));
        sendData(responseStr,requestActive,false,false,false);  //transmit out data over the same mode the request was recived over
        requestActive = 0;          //Set flag back to 0
    }
}

//Function to check if the status is updated based on a flag and then transmit it out to the CChub
void statusUpdate(){
    if(statusReady){        //Check if status flag has been set by timer that calculates system status flags
        #ifdef VERBOSE
        Serial.println("Sending a status update!");     //Log to console (for debug purposes)
        #endif
        char updateStr[55];                             //Create local string to hold status being sent out
        sprintf(updateStr,"B%dABsup%d %d %0.6f %0.6f %d %d ",BOTNUM,battPercent,statusFlags,latitude,longitude,(int)(battVoltage * battCurrent),(int)(battVoltage * solarCurrent));  //Print status flags, battery, latitude and logitude
        if(!BLEAvail && !XBeeAvail && LTEStatusCount && (LTEStatusCount%LTE_STAT_PD == 0)){     //If BLE and XBee are not available, send status over LTE, but only 1 in LTE_STAT_PD updates (to not suck up data)
            sendData(updateStr,0,false,false,true);     //Only send out over LTE
        }
        else{
            if(XBeeAvail || BLEAvail) LTEStatusCount = LTE_MAX_STATUS;  //Otherwise, we're sending updates over BLE or XBee, reset counter for cellular
            sendData(updateStr,0,true,true,false);
        }
        if(!BLEAvail && !XBeeAvail && LTEStatusCount) LTEStatusCount--;            //Decrement a large coounter for the LTE status. This stops sending the status over LTE after a while to not burn up monthly quota. Should be recovering bots if on cell only
        statusReady = false;                            //Clear ready flag
        //sendData("B1CCptsbigbot",0,true,false,false);
    }
}

//Function to calculate what speed the motors should move at based on the current drive mode (manual, sentry, autonomous)
void updateMotors(){
    //if(millis() - motionTime > MTR_RAMP_TIME){
    //    updateMotorControl = true;
    //    motionTime = millis();
    //}
    //if(updateMotorControl){                                 //Flag to initialize a motor update, such that the motor speed is ramped to the target oover time
        if(driveMode == 1 || driveMode == 2){               //Change the value of setLSpeed and setRSpeed here for the autonomous algorithm
            if(travelDistance < MTR_CUTOFF_RAD){            //If the bot is close enough to the center when in autonomous and sentry, then disable motors and float there
                pointArrived = true;                        //Indicate that the bot has arrived at the target point, which acts as a disable until it drifts out of the larger radius
                leftMotorSpeed = setLSpeed = 90;            //Set left and right motor speeds to off
                rightMotorSpeed = setRSpeed = 90;
            }
            else if(travelDistance < SENTRY_IDLE_RAD){      //Check if the bot is inside of the larger radius of approaching the target point, start slowing motors here
                if(pointArrived){                           //If we had already arrived at the target point, then use this larger radius as a deadzone so we don't have rapid on/off on the small radius border
                    setLSpeed = 90;                         //Keep motors off here
                    setRSpeed = 90;
                }
                else{                                       //If we haven't arrived at the point, continue the autonomous movement, but start slowing the motors as we get closer so we don't go beyond due to p=m*v
                    int Rset = (90 + (90 * autoMoveRate) + (targetDelta * autoMoveRate / 2.0)) * (travelDistance/SENTRY_IDLE_RAD);    //Take the base 90 (stopped speed), add the delta for how much the heading is off, and slow with distance
                    int Lset = (90 + (90 * autoMoveRate) - (targetDelta * autoMoveRate / 2.0)) * (travelDistance/SENTRY_IDLE_RAD);
                    if(Lset < 0) setLSpeed = 0;             //Cap the speed between 0 and 180
                    else if(Lset > 180) setLSpeed = 180;
                    else Lset = setLSpeed;
                    if(Rset < 0) setRSpeed = 0;
                    else if(Rset > 180) setRSpeed = 180;
                    else Rset = setRSpeed;
                }
            }
            else{                                           //Otherwise, we are outside the radius of both circles
                pointArrived = false;                       //Set flag back to false so we have to travel to the inner circle, also happens usually when a new point is specified
                int Rset = 90 + (90 * autoMoveRate) + (targetDelta * autoMoveRate / 2); //Take the base 90 (stopped speed), add the delta for how much the heading is off, and the base move rate multiplier
                int Lset = 90 + (90 * autoMoveRate) - (targetDelta * autoMoveRate / 2); 
                if(Lset < 0) setLSpeed = 0;                 //Cap speed between 0 and 180
                else if(Lset > 180) setLSpeed = 180;
                else setLSpeed = Lset;
                if(Rset < 0) setRSpeed = 0;
                else if(Rset > 180) setRSpeed = 180;
                else setRSpeed = Rset;
            }
        }

        if(setLSpeed > 90 && setLSpeed <= MTR_ST_FWD) setLSpeed = MTR_ST_FWD; //Push motor speed out of deadzone to make sure the motors actually respond to non-90 inputs
        if(setRSpeed > 90 && setRSpeed <= MTR_ST_FWD) setRSpeed = MTR_ST_FWD;
        if(setLSpeed < 90 && setLSpeed >= MTR_ST_REV) setLSpeed = MTR_ST_REV;
        if(setRSpeed < 90 && setRSpeed >= MTR_ST_REV) setRSpeed = MTR_ST_REV;

        if(leftMotorSpeed < setLSpeed){                                                     //If the acutal motor (leftMotorSpeed) speed is less than the target motor speed (setLSpeed), then ramp the acutal motor speed to reach target
            if(setLSpeed - leftMotorSpeed > MTR_RAMP_SPD) leftMotorSpeed += MTR_RAMP_SPD;   //If we're off by more than one step size, then increment by one step
            else leftMotorSpeed = setLSpeed;                                                //Otherwise, we're less than one step, so finish step function
        }
        else if(leftMotorSpeed > setLSpeed){                                                //If the acutal motor (leftMotorSpeed) speed is greater than the target motor speed (setLSpeed), then ramp the acutal motor speed to reach target
            if(leftMotorSpeed - setLSpeed > MTR_RAMP_SPD) leftMotorSpeed -= MTR_RAMP_SPD;   //If we're off by more than one step size, then decrement by one step
            else leftMotorSpeed = setLSpeed;                                                //Otherwise, we're less than one step, so finish step function
        }
        if(rightMotorSpeed < setRSpeed){                                                    //If the acutal motor (rightMotorSpeed) speed is greater than the target motor speed (setRSpeed), then ramp the acutal motor speed to reach target
            if(setRSpeed - rightMotorSpeed > MTR_RAMP_SPD) rightMotorSpeed += MTR_RAMP_SPD; //If we're off by more than one step size, then increment by one step
            else rightMotorSpeed = setRSpeed;                                               //Otherwise, we're less than one step, so finish step function
        }
        else if(rightMotorSpeed > setRSpeed){                                               //If the acutal motor (rightMotorSpeed) speed is greater than the target motor speed (setRSpeed), then ramp the acutal motor speed to reach target
            if(rightMotorSpeed - setRSpeed > MTR_RAMP_SPD) rightMotorSpeed -= MTR_RAMP_SPD; //If we're off by more than one step size, then decrement by one step
            else rightMotorSpeed = setRSpeed;                                               //Otherwise, we're less than one step, so finish step function
        }
        //Serial.printlnf("Lspd: %d Rspd: %d HDelt: %d Hdist: %0.2f MR: %0.2f", leftMotorSpeed, rightMotorSpeed, (int)targetDelta, travelDistance, autoMoveRate);
        if(!stopActive){                    //If there has not been a stop command, then update the ESC
            ESCL.write(180-leftMotorSpeed);
            ESCR.write(rightMotorSpeed);
            //Serial.printlnf("Update motor speed (%dms): %d %d", millis(), setRSpeed, setLSpeed);
        }
        updateMotorControl = false;        //Set the flag to false
    //}
}

//Majoy function for sending a string of data out over BLE, XBee or LTE. Automatically calculates the checksum from the given string
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

//Function used for BLE debugging that allows printing debug messages to a remote device
void printBLE(const char *dataOut){
    #ifdef BLE_DEBUG_ENABLED                                        //Only functional if BLE debugging is enabled, disable to reduce overhead in final build
        if(millis() - BLEdbgTimer < 1000) return;
        BLEdbgTimer = millis();
        uint8_t txBuf_tmp[strlen(dataOut)];                         //Convert input string to byte array to transmit out of BLE
        memcpy(txBuf_tmp,dataOut,strlen(dataOut));                  //Copy character array elements into byte array
        bledbgCharacteristic.setValue(txBuf_tmp, strlen(dataOut));  //Transmit out byte array
    #endif
}

//ISR Function to calculate bitmasked status number and signal to the main loop that the status is ready
void StatusHandler(){
    statusFlags = 0;                    //Reset status flags to 0, then add up the individual flags
    statusFlags = LTEAvail;             //Bit 0 indicates LTE is available
    statusFlags |= XBeeAvail << 1;      //Bit 1 indicates XBee is available
    statusFlags |= BLEAvail << 2;       //Bit 2 indicates BLE is available
    statusFlags |= offloadMode << 3;    //Bit 3 indicates bot is currently offloading
    statusFlags |= driveMode << 4;      //Bit 4 indicates the current drive mode
    statusFlags |= lowBattery << 6;     //Bit 6 indicates that the battery is low
    statusFlags |= logSensors << 7;     //Bit 7 indicates that the Atlas sensors are being logged to the SD card
    statusFlags |= GPSAvail << 8;       //Bit 8 indicates neo-m8u GPS is available and receiving non-null data
    statusFlags |= CompassAvail << 9;   //Bit 9 indicates the LIS3MDL compass is connected and providing dat
    statusFlags |= SDAvail << 10;       //Bit 10 indicates the SD card is functional and can record data
    statusReady = true;                 //Set flag true, so the main loop will transmit out status to CChub
    //Serial.println("Sending a status update!");
}

//Function to read and request data from the Atlas scientific sensors over I2C. Uses millis() timer to ensure at least 850ms between request and read (required for Atlas sensors)
void sensorHandler(){
    if(dataTimer < millis() && dataWait){       //Check if the timer for waiting after a data request has expired
        if(Wire.requestFrom(PHADDR, 20, 1)){    //Request 20 bytes from the PH sensor
            byte code = Wire.read();            //the first byte is the response code, we read this separately.
            char tempSense[20];                 //Temporary string to hold string returned by the sensor
            int c = 0;                          //Index variable for the temporary string
            while(Wire.available()){            // slave may send less than requested
                tempSense[c++] = Wire.read();   //Read each of the bytes returned by the sensor into a string

            }
            sensePH = atof(tempSense);          //Convert the string to a float and store it in the global pH variable
        }
        //Serial.printlnf("pH: %f", sensePH);
        if(Wire.requestFrom(MCOND, 20, 1)){
            byte code = Wire.read();            //the first byte is the response code, we read this separately.
            char mcondSense[20];
            int c = 0;
            while(Wire.available()){   // slave may send less than requested
                mcondSense[c++] = Wire.read();

            }
            senseMCond = atof(mcondSense);
        }
        //Serial.printlnf("MiniCond: %f",senseMCond);
        if(Wire.requestFrom(COND, 20, 1)){
            byte code = Wire.read();            //the first byte is the response code, we read this separately.
            char condSense[20];
            int c = 0;
            while(Wire.available()){   // slave may send less than requested
                condSense[c++] = Wire.read();

            }
            senseCond = atof(condSense);
        }
        //Serial.printlnf("Conductivity: %f",senseCond);
        if(Wire.requestFrom(TEMPADDR, 20, 1)){
            byte code = Wire.read();             //the first byte is the response code, we read this separately.
            char addrSense[20];
            int c = 0;
            while(Wire.available()){   // slave may send less than requested
                addrSense[c++] = Wire.read();

            }
            senseTemp = atof(addrSense);
        }
        if(Wire.requestFrom(DOADDR, 20, 1)){
            byte code = Wire.read();             //the first byte is the response code, we read this separately.
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
            if(!myFile.isOpen()){               //Print out each of the global sensor values
                myFile.open(filename, O_RDWR | O_CREAT | O_AT_END);
                myFile.printlnf("%s,%f,%f,%f,%f,%f,%f,%f",timestamp,latitude,longitude,senseTemp,sensePH,senseDO,senseMCond,senseCond);
                myFile.close();
            } 
            else{
                myFile.printlnf("%s,%f,%f,%f,%f,%f,%f,%f",timestamp,latitude,longitude,senseTemp,sensePH,senseDO,senseMCond,senseCond);
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

//Function to check the UART serial buffer from XBee and then send any received commands to the processCommand function
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
        processCommand(buffer,2,true);                  //Process the command received over Xbee using the dictionary
        if(buffer[0] == 'B' || buffer[0] == 'C') XBeeRxTime = millis(); //If the first characters were from another bot or from the CC, then assume Xbee is working, so update it's watchdog counter
        if(logMessages){
            if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
            logFile.printlnf("[INFO] Received XBee Message: %s",data);
            logFile.close();
        }
    }
}

//ISR function triggered whenever data is received over BLE. Converts to a string and then sends data to processCommand dictionary
static void BLEDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context){
    char btBuf[len+1];                                              //Create character array for the received command
    for (size_t ii = 0; ii < len; ii++) btBuf[ii] = data[ii];       //Convert byte array into character array
    if(btBuf[len-1] != '\0') btBuf[len] = '\0';                     //Make sure there is a null character at the end (another bug that cost many hours and seeing random data from surrounding memory)
    else btBuf[len-1] = '\0';
    #ifdef VERBOSE
    Serial.println("New BT Command:");
    Serial.println(btBuf);                                          //Print out command for debugging purposes
    #endif
    processCommand(btBuf,1,true);                                   //Process the command received over BLE using the dictionary
    if(btBuf[0] == 'A' || btBuf[0] == 'C') BLERxTime = millis();    //If the first characters were from another bot or from the CC, then assume Xbee is working, so update it's watchdog counter
    if(logMessages){
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[INFO] Received BLE Message: %s",btBuf);
        logFile.close();
    }
}

//ISR timer to check if strings have been received from the CCHub, and will cut off motors if an update has not been received recently
void motionHandler(){
    //If the bot is operating in manual RC mode, then check that a mtr message has been received from the CC recently, otherwise cut off to prevent driving into oblivion
    if(driveMode == 0 && setLSpeed != 90 && setRSpeed != 90 && millis() - lastMtrTime > MTR_TIMEOUT){
        setLSpeed = 90;
        setRSpeed = 90;
        leftMotorSpeed = 90;
        rightMotorSpeed = 90;
        updateMotorControl = true;
        ESCL.write(setLSpeed);
        ESCR.write(setRSpeed);
        Serial.printlnf("Warning, motor command has not been received in over %dms, cutting motors", MTR_TIMEOUT);
    }
    //If we're in an autonomous mode, also check that telemetry is available, otherwise, return to manual RC mode
    if(!telemetryAvail && driveMode != 0 && millis() - lastTelemTime > MTR_TIMEOUT){
        driveMode = 0;
        telemetryAvail = false;
        pointArrived = false;
        setLSpeed = 90;
        setRSpeed = 90;
        leftMotorSpeed = 90;
        rightMotorSpeed = 90;
        updateMotorControl = true;
        ESCL.write(setLSpeed);
        ESCR.write(setRSpeed);
        Serial.printlnf("Warning, GPS or Compass data not available for greater than %dms, exiting autonomous mode", MTR_TIMEOUT);
    }
}

//ISR timer to check if messages over communication modes have been received. updates flags used to determine which methods to send data over
void wdogHandler(){
    if(Particle.connected()) LTEAvail = true;   //If particle cloud is connected, assume that LTE is available
    else if(LTEAvail){
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[WARN] LTE Messages have not been received in %ds, assuming XBee is unavailable",(XBEE_WDOG_AVAIL/1000));
        LTEAvail = false;
    }
    if(millis()-XBeeRxTime > XBEE_WDOG_AVAIL || !XBeeRxTime){   //If the time since the last XBee message is too long, print warning and set status flag to false
        if(XBeeAvail){
            if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
            logFile.printlnf("[WARN] XBee Messages have not been received in %ds, assuming XBee is unavailable",(XBEE_WDOG_AVAIL/1000));
        }
        XBeeAvail = false;
    }
    else XBeeAvail = true;
    if(millis()-BLERxTime > BLE_WDOG_AVAIL || !BLERxTime){      //If the time since the last BLE message is too long, print warning and set status flag to false
        if(BLEAvail && BLERxTime){
            if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
            logFile.printlnf("[WARN] BLE Messages have not been received in %ds, assuming BLE is unavailable",(BLE_WDOG_AVAIL/1000));
        }
        if(BLE.connected() && XBeeAvail) BLEAvail = true;
        else BLEAvail = false;
    }
    else BLEAvail = true;
    if(stopActive && millis() - stopActive > STOP_RST_TIME) stopActive = false;                          //Set stop to false in case the CChub somehow crashed (though we have already entered a "float" mode where drivemode = 0)
}

//Function to pause operation and copy data off of SD card over bluetooth to the CChub
void dataOffloader(){
    Serial.println("Entering Data Offloader Mode");
    myFile.close();
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
    while (myFile.openNext(&logDir, O_RDONLY) && BLE.connected()) {
        char namebuf[MAX_FILENAME_LEN];
        myFile.getName(namebuf,MAX_FILENAME_LEN);
        Serial.printlnf("Checking if file %s is a .csv or .txt...", namebuf);
        if(!strstr(strlwr(namebuf + (strlen(namebuf) - 4)), ".csv")){
            myFile.close();
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
            while(myFile.available()){
                char lineBuffer[BLE_OFFLD_BUF];
                memset(lineBuffer,0,BLE_OFFLD_BUF);
                //myFile.readBytes(lineBuffer,BLE_OFFLD_BUF);
                myFile.readBytesUntil('\r',lineBuffer,BLE_OFFLD_BUF);
                offloadCharacteristic.setValue(lineBuffer);
                //Serial.println(lineBuffer);
            }
            interrupts();
            memset(fileCode,0,8+MAX_FILENAME_LEN);
            strcpy(fileCode,"filecomp");
            //memcpy(codeBuf,fileCode,8+MAX_FILENAME_LEN);
            offloadCharacteristic.setValue(fileCode);
            delay(150);
            
            myFile.close();
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

//Timer that activates whenever the power button is pressed
void buttonTimer(){
    #ifdef PWR_EN
    if(digitalRead(PWR_BUT)) digitalWrite(PWR_EN, LOW); //Turn off system
    #endif
    shutdownTimer.stopFromISR();
}

//ISR triggered when button is pressed or released
void buttonHandler(){
    if(digitalRead(PWR_BUT)){               //If the interrupt was triggered by the button being pressed
        shutdownTimer.startFromISR();       //Start the shutdown timer, which will check if the button has been held for long enough
        shutdownActive = true;              //Set flag to true to make LED flash and indicate shutdown initiate
        //char teststr[15];
        //sprintf(teststr,"B%dCCptsShutdown",BOTNUM);
        //sendData(teststr,0,false,true,false);
    }
    else{                                   //If the interrupt was triggered by the button being released
        shutdownTimer.stopFromISR();        //Stop the ISR which will shut off the bot
        shutdownActive = false;             //Clear shutdown flag to make LED stop blinking
    }
}

//ISR timer to update the color and pattern of the LED based on the status of the system
void LEDHandler(){
    uint32_t SetColor;
    LEDPattern SetPattern;
    LEDSpeed SetSpeed;
    uint8_t statusMode;
    //Special LED Modes
    /*if(shutdownActive){     //The user is holding down the power off button
        status.setPattern(LED_PATTERN_BLINK);
        status.setColor(RGB_COLOR_GREEN);
        status.setSpeed(LED_SPEED_FAST);
        return;   
    }*/
    if(stopActive){         //The user has pressed the stop button on the CChub
        status.setPattern(LED_PATTERN_BLINK);
        status.setColor(RGB_COLOR_YELLOW);
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
    else if(driveMode == 2){    //If we're in autonomous mode, do a fade pattern
        SetPattern = LED_PATTERN_FADE;
        SetSpeed = LED_SPEED_NORMAL;
    }
    else if(driveMode == 1){    //If we're in sentry, then blink slowly
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

//Function to log a string to the SD card log file
void logMessage(const char *message){
    if(!logFile.isOpen()){
        logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.println(message);
        logFile.close();
    }
    else logFile.println(message);
}

//Particle function for processing a debug string, for testing things like an emulated GPS point
int LTEInputCommand(String cmd){
    char cmdBuf[100];
    cmd.toCharArray(cmdBuf, 100);
    processCommand(cmdBuf, 4,false);
    if(logMessages){
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[INFO] Received LTE Message: %s",cmdBuf);
        logFile.close();
    }
    return 1;
}
