/*
 * Project WaterNet23CCHub
 * Description: Code for the Central Control hub responsible for orchestrating commands to Water Bots
 * Author: WaterNet23 - Matthew Panizza, Jake Meckley Cynthia Rios, Harrison Strag
 * Date: 9/2/22
 */


#include "application.h"
#include "SdFat.h"                              //Library for SD Card
#undef min
#undef max
#include <vector>                               //Standard C++ library for vector data type
#include <Adafruit_GFX.h>                       //Library for displaying text/shapes on standard LCDs
#include <Adafruit_SH110X.h>                    //Library to use Adafruit GFX library with this specific 128x64 OLED

// Pin Definitions

#define OLED_RESET              A0              //Pin used to reset the OLED screen on startup
#define JOYH_ADC                A2              //Horizontal Joystick
#define JOYV_ADC                A3              //Vertical Joystick ADC pin
#define JOY_BTN                 D23             //Joystick button press
#define L_DPAD                  A4              //Left DPAD button
#define R_DPAD                  A1              //Right DPAD button
#define U_DPAD                  A5              //Up DPAD button
#define D_DPAD                  D7              //Down DPAD button
#define E_DPAD                  D22             //"Enter" DPAD button
#define chipSelect              D8              //SD Card chip select pin
#define STOP_BTN                A7              //Stop button to disable all bot motors

//Program Parameters

#define DEF_FILENAME            "CCHUB"
#define BLE_OFFLD_BUF           100             //Maximum number of characters to fetch for each read from the SD card when offloading
#define SCAN_RESULT_COUNT       20              //Maximum devices to discover per scan
#define CUSTOM_DATA_LEN         8               //Number of bytes to include in the advertising packet for BLE
#define MAX_FILENAME_LEN        30              //Maximum number of characters in the filename for log files
#define MAX_ERR_BUF_SIZE        15              //Buffer size for error-return string
#define MAX_LTE_STATUSES        300             //Maximum number of statuses to send over LTE when XBee and BLE are unavailable

//Timing Control

#define XBEE_BLE_MAX_TIMEOUT    36
#define BLE_MAX_CONN_TIME       200             //20 second max time to successfully pair to bot
#define BLE_SCAN_PERIOD         1000            //Time betweeen BLE scans
#define LTE_BKP_Time            100             //Send LTE request after 10 seconds if not connected to any bot
#define LTE_CTL_PERIOD          29000           //Minimum time between sending LTE control packets periodically per bot
#define MTR_LTE_PERIOD          3000            //Minimum time between sending mtr command over LTE
#define MTR_UPDATE_TIME         250             //Frequency to send manual motor control packet in milliseconds
#define CONTROL_PUB_TIME        5000            //Number of milliseconds between sending control packets to bots
#define STOP_PUB_TIME           5000            //Time between sending stop messages when active
#define WB_MOD_UPDATE_TIME      60000           //Timeout for when status update packets will modify the class, prevents immediate overwrite when changing control variables

//Menu Parameters
#define i2c_Address             0x3c            //initialize with the I2C addr 0x3C Typically eBay OLED's
#define MAX_MENU_ITEMS          9               //Maximum number of settings per bot displayed on the menu (must go to where MenuItems.push_back() and count this)
#define DEBOUNCE_MS             150             //Number of milliseconds to disable button effects after pressing a button
#define OLED_MAX_X              128             //Number of pixels in the X direction
#define OLED_MAX_Y              64              //Number of pixels in the Y direction

//Control Parameters
#define JOY_DEADZONE        35                  //ADC reading offset in the middle to read no change in the the position
#define JOY_MID             2048                //ADC reading when the joystick is centered
#define JOY_MAX             4094                //ADC reading when the joystick is fully forward
#define JOY_MIN             1                   //ADC reading when the joystick is fully backward
#define LTE_MIN_DIFF        3                   //Minimum difference in motor speed to send an update over LTE

//Drive mode parameters
#define DRIVE_MODE_MANUAL      0               //Drive mode where the user has full control of the motors
#define DRIVE_MODE_SENTRY      1               //Drive mode where the bot autonomously navigates to a fixed waypoint and holds one spot
#define DRIVE_MODE_AUTONOMOUS  2               //Drive mode where the bot autonomously navigates through a list of waypoints

//Development Parameters
//#define VERBOSE

// This example does not require the cloud so you can run it in manual mode or
// normal cloud-connected mode
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

// These UUIDs were defined by Nordic Semiconductor and are now the defacto standard for
// UART-like services over BLE. Many apps support the UUIDs now, like the Adafruit Bluefruit app.
const BleUuid serviceUuid("b4206910-dc4b-5743-c8b1-92d0e75182b0");
const BleUuid rxUuid("b4206912-dc4b-5743-c8b1-92d0e75182b0");
const BleUuid txUuid("b4206913-dc4b-5743-c8b1-92d0e75182b0");
const BleUuid offldUuid("b4206914-dc4b-5743-c8b1-92d0e75182b0");

BleScanResult scanResults[SCAN_RESULT_COUNT];

BleCharacteristic peerTxCharacteristic;         //Characteristic to receive data from the bot
BleCharacteristic peerRxCharacteristic;         //Characteristic to send data too the bot
BleCharacteristic peerOffloadCharacteristic;    //Characteristic to receive SD card data from bot
BlePeerDevice peer;                             //Bluetooth peer device that is currently connected

//OLED Object for the onboard menu screen
Adafruit_SH1107 oled = Adafruit_SH1107(64, 128, &Wire, OLED_RESET);

//SD File system object
SdFat sd((SPIClass*)&SPI1);

File myFile;                                    //File object to create files when doing offloading
File logFile;                                   //File object for logging debugging messages
File logDir;                                    //File object to point at root directory of the SD card

unsigned long lastScan = 0;                     //Timer variable for timing bluetooth scans
bool offloadingMode;                            //Flag to indicate that the CC is in receiving mode for offloading
bool offloadingDone;                            //Flag set until the sending bot finishes transmission
char offloadFilename[MAX_FILENAME_LEN];         //Filename of the file being offloaded
char filenameMessages[MAX_FILENAME_LEN];        //Filename of the file for debug messages
bool logMessages;                               //Default flag for if debug messages should be logged to the SD card
bool startConnect;                              //Flag used for finding bluetooth devices when pairing, set true once a device was found
bool postStatus;                                //Flag set true when status should be posted to the bots
bool publishedFirstStatus = false;              //Flag set true when the first status has been published to the bots
uint32_t controlUpdateID;                       //Id to publish the next status update to. Updates go in a circle between all discovered bots
bool LTEStopSent;                               //Only send stop command over LTE when first pressed instead of periodically publishing
uint32_t stopTime;                              //Timer for periodically publishing stop command when a stop is active over XBee and BLE
uint32_t controlUpdateTime;                     //Timer for periodically sending control packet to bots     
uint32_t rcTime;                                //Time between sending mtr commands to the currently controlled bot
uint16_t LTEStatuses = MAX_LTE_STATUSES;        //Counter for number of statuses that have been sent over LTE. Stops sending status over LTE after running out
bool stopActive;                                //Flag set active when the stop button has been pressed, and is cleared after pressing again
uint8_t BLEBotNum;                              //Bot id of the bot currently connected to over BLE
bool ctlSpeedDiff = false;                      //Flag set true when the motor has a significant enough speed change to warrant sending a new speed immediately
uint8_t LSpeed, RSpeed;
bool LTEConnected = false;                      //Flag set true if the user has allowed connection to LTE
int targetTableBot = -1;                        //When printing out a table to the serial console, the bot number to target for the table
uint32_t tableUpdateTime = 0;                   //Timer for periodically printing the status table

//Menu variables
uint8_t botSelect = 0;                          //Which bot in the menu is currently selected
bool redrawMenu = true;                         //Flag to indicate to the main loop to redraw the screen elements when there is an update
bool selectingBots = true;                      //Flag set based on if either bots are selected, or if we are modify the value of a selected menu item
uint8_t menuItem = 0;                           //Variable to hold which setting is currently being selected
bool modifiedValue = false;                     //Flag set true when a value is modified when scrolling through items
bool updateControl = false;                     //Flag set true when the a value has been modified which causes a control packet to be transmitted out
uint32_t debounceTime;                          //Timer shared between all buttons for not registering button interrupts to not cause bouncing

//Class used widely throughout the program to handle data sent and received by the waterbots. Represents the controllable/status attributes of the actual bots, each given a dedicated object
class WaterBot{
    public:
    bool updatedStatus = false;      //Flag to indicate that a new status was received and should have an update sent to the raspberry pi
    bool updatedControl = true;     //Flag to indicate that the menu buttons have modified an attribute of this bot that requires a control packet to be sent
    uint8_t botNum;                 //Bot number unique to each bot and can range from 0 to 9. Identified in each message
    uint16_t battPercent = 0;       //Battery percentage read by each bot, reported over status update packet and displayed on menu
    bool LTEInitialStatus = false;
    bool BLEAvail = false;          //Flag if BLE communication is functional between CC and bot, will affect method of communication choice
    bool LTEAvail = false;          //Flag if LTE communication is functional between CC and bot, will affect method of communication choice
    bool XBeeAvail = false;         //Flag if XBee communication is functional between CC and bot, will affect method of communication choice
    bool GPSAvail = true;           //Flag if the GPS module is available on this bot, will pop up warning if set back to false by a bot status update
    bool CompassAvail = true;       //Flag if the Compass module is available on this bot, will pop up warning if set back to false by a bot status update
    bool SDAvail = true;            //Flag if the SD Card is available on this bot, will pop up warning if set back to false by a bot status update
    uint16_t driveMode = 0;         //Var to hold which drive mode this bot is in (0 = remote, 1 = sentry, 2 = autonomous)
    bool signal = false;            //Flag to turn on or off a flashing indicator, which will flash the onboard LED on this bot for physical identification
    bool lowBatt = false;           //Flag indicating if this bot's power system is indicating low battery, will prompt a warning on the onboard menu
    bool warnedLowBatt = false;     //Flag to indicate that there has been a low-battery pop up for this bot, which is so the user is not spammed constantly with warnings
    bool warnedSDCard = false;      //Flag to indicate that there has been a SD card failure pop up for this bot, which is so the user is not spammed constantly with warnings
    bool warnedTelem = false;       //Flag to indicate that there has been a telemetry failure pop up for this bot, which is so the user is not spammed constantly with warnings
    bool dataRecording = true;      //Flag to indicate that sensor data is being recorded to the SD card
    bool offloading = false;        //Flag to indicate that SD card data is being offloaded
    int leftMotorSpeed = 0;         //Current left motor speed from 0-180 (90 = idle)
    int rightMotorSpeed = 0;        //Current right motor speed from 0-180 (90 = idle)
    bool waypointArrived = false;   //Flag set true when the bot has arrived at a waypoint, use this to advance to the next waypoint
    uint16_t botWaypointIndex = 0;  //Index of the current waypoint the bot is navigating to in autonomous mode
    uint16_t waypointIndex = 0;          //Index of the current waypoint the bot is navigating to in autonomous mode
    std::vector<float> waypointsLat; //Vector of latitudes for waypoints in autonomous mode
    std::vector<float> waypointsLon; //Vector of longitudes for waypoints in autonomous mode
    float lastWaypointLat = -999.0f; //Latitude of the last waypoint reached, used to prevent repeated waypoint arrival messages
    float lastWaypointLon = -999.0f; //Longitude of the last waypoint reached, used to prevent repeated waypoint arrival messages
    float GPSLat = 0.0;             //Current GPS latitude sampled from onboard Ublox module
    float GPSLon= 0.0;              //Current GPS longitude sampled from onboard Ublox module
    uint16_t CompassHeading = 0.0;     //Current filtered compass heading from onboard module
    uint16_t targetHeading = 0.0;   //Target heading for autonomous navigation mode, calculated from current GPS position and target GPS position. Received from vehicle    
    bool requestCompCalibration = false; //Flag set true when the bot is requesting a compass calibration, which will pop up a warning on the menu
    bool calRequestAcknowledged = false; //Flag set true when the user has acknowledged the compass calibration request by clearing pop-up
    uint8_t reqActive = 0;         //Flag set true when a request should be made to get sensor data
    float pH = 0.0;                 //pH sensor reading, populated when a sensor request ("sns" command) is made
    float temp = 0.0;               //Water temperature sensor reading, populated when a sensor request ("sns" command) is made
    float DO = 0.0;                 //Dissolved Oxygen sensor reading, populated when a sensor request ("sns" command) is made
    float Cond = 0.0;               //Conductivity sensor reading, populated when a sensor request ("sns" command) is made
    float MCond = 0.0;              //Mini-conductivity sensor reading, populated when a sensor request ("sns" command) is made
    bool motorTareReceived = false; //Flag set true when a motor tare value has been received from the bot, which will allow editing on the menu
    uint16_t lastMotorTare = 100;   //Last motor tare value sent to the bot, used to prevent sending duplicate values
    uint16_t motorTare = 100;       //Motor tare 0-200, default of 100 which is no tare.
    uint16_t panelPower = 0;        //Solar panel power (in Watts) sampled from the onboard shunt and voltage divider
    uint16_t battPower = 0;         //Battery power draw (in Watts) sampled from the onboard shunt and voltage divider
    uint32_t publishTime = 0;       //A timer used to handle a data "hazard", which prevents the bot from updating this class with variables controllable in both locations for a short period of time, to not have a "loop"
    uint32_t LTELastStatTime = 0;   //A timer used to hold the last time a periodic "ctl" command was sent to the bot to limit the rate of LTE publishes
    uint32_t LastMtrTime = 0;       //A timer used to hold the last time a "mtr" command was sent to limit the rate of mtr published
};

class PairBot{
    public:
    uint8_t botNum;
    int rssi;
};

class MenuItem{                                 //Class for displaying menu on the mini OLED. Can have different label types. Takes a pointer to a variable type in the WaterBot class
    public:
        std::vector<String> labels;             //Vector of strings for custom labels (such as "Rem", "Sen", and "Aut") for the drive mode menu item. Indexed by the current value at the variable (i.e. a value of "2" at the method pointer displays the string at vector location 2)
        void init(uint16_t inStep, uint16_t minV, uint16_t maxV, bool switchOnOff, const char * itemString){    //Constructor for the menu item, requires setting key properties of the item
            minVal = minV;                      //Minimum value to decrement to when pressing the left button
            maxVal = maxV;                      //Maximum value to increment to when pressing the right button
            stepSize = inStep;                  //Number of values to step up or down by when pressing left or right button
            onOffSetting = switchOnOff;         //Set true if this setting is an On/Off setting (like "Signal"). Causes the string "On" and "Off" to be displayed instead of a digit
            strcpy(itemName,itemString);        //Copy in the string displayed as the label of the menu item liike "Signal" or "Battery"
        }
        uint16_t WaterBot::*MethodPointer;      //Pointer to one of the members of the WaterBot class that this menu item modifies. Modifies a 16-bit unsigned integer type. See menu creation for example
        bool WaterBot::*MethodPointerBool;      //Pointer to one of the members of the WaterBot class that this menu item modifies. Modifies a boolean type. See menu creation for example
        uint16_t stepSize;                      //Internal step size variable, amount to step up/down by when pressing left or right button
        bool onOffSetting = false;              //Set true if this setting is an On/Off setting (like "Signal"). Causes the string "On" and "Off" to be displayed instead of a digit
        bool customLabel = false;               //If this flag is set true, then the labels in the "labels" vector are used instead of a numeric digits or the On/Off label
        bool statOnly = false;                  //If this flag is true, then this field cannot be modified by the buttons. Used to display statistics only, like the battery percentage which comes from the bot
        uint16_t minVal;                        //Minimum value to decrement to when pressing the left button
        uint16_t maxVal;                        //Maximum value to increment to when pressing the right button
        char itemName[10];                      //Copy in the string displayed as the label of the menu item liike "Signal" or "Battery"
};

class MenuPopUp{                                //Class for displaying a pop up on the mini OLED. Has various lines for displaying text to the user with warnings
    public:
        char primaryLine[10];                   //Primary line is the main header displayed on the first line, with a maximum of 9 characters. Typically is the "WARNING" or "HELLO". Used strcpy to set the label
        char secondaryLine[30];                 //Second line on the screen, holds 29 characters. Used strcpy to set the label
        char tertiaryLine [30];                 //Third line on the screen, holds 29 characters. Used strcpy to set the label
        uint8_t primaryStart = 0;               //Horizontal pixel to start displaying this line at. Used to center the text in the box
        uint8_t secondaryStart = 0;             //Horizontal pixel to start displaying this line at. Used to center the text in the box
        uint8_t tertiaryStart = 0;              //Horizontal pixel to start displaying this line at. Used to center the text in the box
};

WaterBot *ControlledBot;                        //Pointer to the bot that is currently selected in the bot selection pane
std::vector<WaterBot> WaterBots;                //Main vector to hold discovered water bots. Used extensively - sends status updates to the bots based on the parameters set here. 
std::vector<WaterBot> PairBots;                 //Vector used to scan for bots over all communication modes upon startup.
std::vector<PairBot> BLEPair;                   //Vector used to scan for bots over BLE upon startup. Helps to find the closest bot to the controller to connect to
std::vector<MenuPopUp> PopUps;                  //Vector to hold pop-ups so they act as a queue. Pressing enter pops the most recently displayed pop up if there is one active

Timer at1(5000,actionTimer5);                   //Timer used to set flags, currently used to periodically request sensor data

MenuItem * SelectedItem;                        //Pointer to the currently selected menu item so the left and right button handlers can update the integer/boolean pointed to by this menu item
std::vector<MenuItem> MenuItems;                //Vector of menu items in the system. Push items into this vector on startup and they will be automatically displayed and be modifiable


//Command table structure for cleaner command processing
typedef void (*CommandHandler)(const char* dataStr, uint8_t rxBotID, uint8_t mode);

struct CommandEntry {
    const char* command;
    CommandHandler handler;
};

//Function prototypes
void BLEScan(int BotNumber = -1);
void XBeeHandler();
void XBeeLTEPairSet();
void dataLTEHandler(const char *event, const char *data);
void createMenu();
void startupPair();
void rHandler(void);
void printStatusTable();
void handleStatusUpdateCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode);
void handleSensorCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode);
void handleHelloWorldCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode);
void handlePutStringCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode);
void handleLeakDetectionCommand(const char *cmdStr, uint8_t rxBotID, uint8_t mode);
void handleLeakWarningCommand(const char *cmdStr, uint8_t rxBotID, uint8_t mode);

// RPi command helpers (cbp, ctl, tbl)
void handleRPiChecksumBypassCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode);
void handleRPiControlCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode);
void handleRPiAddWaypointCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode);
void handleRPiTableCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode);
void printHelpMenu();

// Global flag controlling whether RPi checksum is enforced (toggled by cbp)
static bool rpiChecksumBypassed = true;

/**
 * @brief LTE data handler for incoming messages from water bots
 * @param event Event name (unused)
 * @param data Command string received from bot over LTE
 * 
 * Interrupt handler called anytime a message is received from a Bot/CCHUB over LTE.
 * Processes the command and optionally logs it to SD card.
 */
void dataLTEHandler(const char *event, const char *data){           //Interrupt handler called anytime a message is received from a bot over LTE.
    processCommand(data, 4,false);                                  //Send the command to the dictionary function for processing
    if(logMessages){                                                //Log the incoming message to the SD card if enabled
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[INFO] Received LTE Message: %s",data);   //Print the message as an info function
        logFile.close();
    }
}

/**
 * @brief Initialize LTE cellular connection to Particle cloud
 * 
 * Waits for the LTE connection to the Particle cloud. Shows an animation on LCD 
 * to indicate status. User can override by pressing up+down buttons simultaneously.
 */
void initializeCellularConnection(){
    Particle.connect();                                         //Initiate connection to the cloud
    oled.clearDisplay();                                        //Empty the display for the new startup item
    oled.setCursor(0,0);                                        //Scanning text put on first line
    oled.print("Connecting ");                                  //Show connecting text on first line
    oled.setCursor(0,16);                                       //Move to next line
    oled.print("to LTE");                                       //Show secondary line
    oled.display();                                             //Update the LCD
    uint8_t loadAnim = 0;
    while(!Particle.connected()){                               //While we are still connecting to LTE, show animation
        for(int i = 0; i < loadAnim; i++){                      //Show a set of LTE signal bars while connecting
            oled.fillRect(90 + (i*4), 28, 2, -(i*2) - 2, SH110X_WHITE);
        }
        oled.display();                                         //Display the animated circle
        loadAnim++;                                             //Increment the counter so the next loop displays the circle
        if(loadAnim > 5){                                       //Loop back to center for animation
            loadAnim = 0;
            oled.fillRect(90, 28, 20, -12, 0);
        }
        delay(150);

        //Override for continuing to connect to LTE.
        if(digitalRead(U_DPAD) == HIGH && digitalRead(D_DPAD) == HIGH){
            LTEConnected = false;
            return;
        }
    }
}

/**
 * @brief Startup pairing routine to discover and connect to water bots
 * 
 * Function to run on startup to display how many bots have been discovered and 
 * wait until at least one is discovered. Shows animated scanning indicator and
 * bot count on OLED display.
 */
void startupPair(){                                             //Function to run on startup to display how many bots have been discovered and wait until at least on is discovered
    startConnect = false;                                       //Flag set true while scanning for bots, cleared upon bluetooth connection or a 
    oled.clearDisplay();                                        //Empty the display for the new startup item
    oled.setCursor(0,0);                                        //Scanning text put on first line
    oled.print("Scanning ");
    oled.fillCircle(115,7,3,SH110X_WHITE);                      //Fill initial circle for animation
    oled.display();
    uint8_t loadAnim = 0;                                       //Animation counter for displaying the radius of the circle
    while(!startConnect){                                       //Loop until bots are found. Wait for user to press the enter button
        oled.setCursor(0,16);                                   //Set location for rectangle
        oled.fillRect(72,16,35,15,0);
        oled.printlnf("Bots: %d",WaterBots.size());             //Notify user of how many bots have been discovered
        #ifdef VERBOSE
            Serial.printlnf("Array size: %d",BLEPair.size());
        #endif
        oled.drawCircle(115,7,loadAnim,SH110X_WHITE);           //Draw the circle with varying circle radius for animation
        if(loadAnim-1) oled.fillCircle(115,7,loadAnim-1,0);     //Erase the previous circle
        oled.display();                                         //Display the animated circle
        loadAnim++;                                             //Increment the counter so the next loop displays the circle
        if(loadAnim > 7){                                       //Loop back to center for animation
            loadAnim = 0;
            oled.drawCircle(115,7,7,0);                         //Erase last circle
        }
        //BLEScan(-2);
        XBeeHandler();                                          //Check XBee radio for hello world message received from bot
        XBeeLTEPairSet();                                       //Add XBee and LTE discovered bots to the Pair Bot vector
        
        delay(50);
        if(digitalRead(E_DPAD) == HIGH && WaterBots.size() > 0){    //When the enter button is pressed, check if there are any discovered bots. Continue to main program if there are discovered bots
            botSelect = WaterBots.at(0).botNum;                     //Update bot select to select the first-found bot on the list
            oled.clearDisplay();
            oled.display();
            selectingBots = true;                                   //By default, we are choosing bots, not modifying a menu item
        }
        
    }
}

/**
 * @brief Send acknowledgment to bots discovered via XBee or LTE
 * 
 * Function to send hello-world acknowledge string to bots when a hello world 
 * message has been received. This stops the bot from periodically sending 
 * "Hello World" messages.
 */
void XBeeLTEPairSet(){                                                          //Function to send hello-world acknowledge string to bots when a hello world message has been received
    for(uint8_t i = 0; i < PairBots.size(); i++){                                  //Loop over discovered bots, send the message, then pop it from the vector, since it will already be in the main WaterBots vector
        char replyStr[10];
        sprintf(replyStr,"CCB%dhwa",PairBots.back().botNum);                    //Bot will stop periodically sending "Hello World" after receiving the acknowledge
        sendData(replyStr,0,true,PairBots.back().XBeeAvail,PairBots.back().LTEAvail);
        PairBots.pop_back();
    }
}

/**
 * @brief Arduino setup function - initializes hardware and communication
 * 
 * Configures GPIO pins, interrupts, timers, Bluetooth, LTE, SD card, OLED display,
 * and establishes communication with water bots. Runs once at startup.
 */
void setup() {

    pinMode(E_DPAD,INPUT_PULLDOWN);                 //Enter Button - Configure each of the buttons used on the controller to be pulldowns, as they are pulled to 3.3V when pressed
    pinMode(U_DPAD,INPUT_PULLDOWN);                 //Up Button - Configure each of the buttons used on the controller to be pulldowns, as they are pulled to 3.3V when pressed
    pinMode(D_DPAD,INPUT_PULLDOWN);                 //Down Button - Configure each of the buttons used on the controller to be pulldowns, as they are pulled to 3.3V when pressed
    pinMode(L_DPAD,INPUT_PULLDOWN);                 //Left Button - Configure each of the buttons used on the controller to be pulldowns, as they are pulled to 3.3V when pressed
    pinMode(R_DPAD,INPUT_PULLDOWN);                 //Right Button - Configure each of the buttons used on the controller to be pulldowns, as they are pulled to 3.3V when pressed
    pinMode(JOY_BTN,INPUT_PULLDOWN);                //Joystick click Button - Configure each of the buttons used on the controller to be pulldowns, as they are pulled to 3.3V when pressed
    pinMode(STOP_BTN, INPUT_PULLDOWN);              //Stop Button - Configure each of the buttons used on the controller to be pulldowns, as they are pulled to 3.3V when pressed
    
    attachInterrupt(E_DPAD,entHandler,RISING);      //Create an interrupt triggered whenever the enter button is pressed
    attachInterrupt(U_DPAD,uHandler,RISING);        //Create an interrupt triggered whenever the up button is pressed
    attachInterrupt(D_DPAD,dHandler,RISING);        //Create an interrupt triggered whenever the down button is pressed
    attachInterrupt(L_DPAD,lHandler,RISING);        //Create an interrupt triggered whenever the left button is pressed
    attachInterrupt(R_DPAD,rHandler,RISING);        //Create an interrupt triggered whenever the right button is pressed
    attachInterrupt(JOY_BTN,jHandler,RISING);       //Create an interrupt triggered whenever the joystick button is pressed
    attachInterrupt(STOP_BTN,sHandler,RISING);      //Create an interrupt triggered whenever the stop button is pressed

    delay(5);

    debounceTime = millis();                        //Initialize timers to current startup time so they are accuracte when the program starts
    controlUpdateTime = millis();
    rcTime = millis();
    tableUpdateTime = millis();                     //Initialize table update timer
    controlUpdateID = -1;

    Serial.begin(115200);                           //Set debug output serial port to use 115200 baud
    Serial1.begin(115200);                            //Start serial for XBee module, using 9600 baud for long range
    setupXBee();                                    //Setup XBee by sending bypass characters for XBee modules which have the integrated controller

	BLE.on();                                       //Make sure bluetooth is on for the controller
    BLE.setScanTimeout(50);                         //100ms scan
    BLE.setTxPower(8);                              //Use highest power to get longest range

    peerTxCharacteristic.onDataReceived(BLEDataReceived, &peerTxCharacteristic);        //Create bluetooth characteristic which triggers the BLEDataReceived function whenever the connected bot publishes a command over BLE
    peerOffloadCharacteristic.onDataReceived(offloadDataReceived, &peerOffloadCharacteristic);

    offloadingMode = false;                         //Initialize flags for offloading so we don't initially start offloading data from the SD card
    offloadingDone = false;

    logMessages = true;                             //Default settings for certain actions - enable SD card logging by default
    postStatus = false;                             //Don't post status until after the first timer expires so we have time to sample the system
    stopActive = false;                             //Flag set true whenever the stop button is hit, cleared when hit again. By default we should not stop the bot motors, or the user could be confused
    LTEStopSent = false;                            //Flag set true when a stop command has been sent already, limits publishing rate
    
    stopTime = 0;                                   //Timer for periodically sending stop signal when stop is active

    char timestamp[16];                             //string for holding the filename timestamp
    snprintf(timestamp,16,"%02d%02d%04d%02d%02d%02d",Time.month(),Time.day(),Time.year(),Time.hour(),Time.minute(),Time.second());
    strcpy(filenameMessages,DEF_FILENAME);          //Create the filename using the current time as well as the label in the macro
    strcat(filenameMessages,timestamp);
    strcat(filenameMessages,"_LOG.txt");

    createMenu();                                   //Function to initialize the menu items by creating menu objects and pushing them to the menu item vector

    delay(250);

    oled.begin(i2c_Address, true);                  // Address 0x3C default for display
    oled.clearDisplay();                            //Clear display in case it had something from a reset
    oled.display();                                 //Apply changes to display
    oled.setRotation(1);                            //Standard rotation
    
    oled.setTextSize(2);                            //Set text size of the "Starting" prompt
    oled.setTextColor(SH110X_WHITE);                //Set white color with black background
    oled.setCursor(0,0);                            //Cursor position for "Starting" prompt
    oled.print(" Starting ");                       //Print label to tell user the system is starting
    oled.display();

    if(digitalRead(U_DPAD) == LOW || digitalRead(D_DPAD) == LOW){
        LTEConnected = true;
        initializeCellularConnection();
    }

    Particle.subscribe("Bot1dat",dataLTEHandler);   //Subscribe to LTE event that comes from bots. They will publish when sending LTE data and this will trigger the dataLTEHandler function
    Particle.function("Input Command", LTEInputCommand);

    if (!sd.begin(chipSelect, SD_SCK_MHZ(8))) {     //Begin communication with the SD card at 8MHz, using chipSelect as the GPIO for selecting this SPI device
        #ifdef VERBOSE
        Serial.println("Error: could not connect to SD card!");
        #endif
        logMessages = false;
    }

    MenuPopUp m;                                        //Create initial pop-up to greet user
    sprintf(m.primaryLine, "Hello!");                   //Display hello as the main text
    sprintf(m.secondaryLine, "Scanning for Bots");      //Second line for information
    sprintf(m.tertiaryLine, "OK when bots ready");      //Third line for information
    m.primaryStart = 32;                                //Values to center the text found by experimentation
    m.secondaryStart = 12;
    m.tertiaryStart = 10;
    PopUps.push_back(m);

    //startupPair();                                      //Not significantly tested - Disable if using an emulated bot id or if the program is crashing on startup
    //delay(3000);

    at1.start();                                        //Start the timer used for requesting sensor data

    WaterBotSim(1);

    
}

/**
 * @brief Main program loop - handles communication and control
 * 
 * Continuously processes status updates, menu interface, bot control, motor commands,
 * BLE scanning, XBee communication, RPi interface, and various maintenance tasks.
 */
void loop() {
    if(postStatus){                                     //Check if the timer has indicated that a status update should be sent
        sendData("CCABspc",0,false,true,false);         //Call send data to send data over XBee, which helps to check if XBee is available
        postStatus = false;                             //Set flag false until time expires again
    }       
    updateMenu();                                       //Function to update all redrawing of the menu, triggered whenever buttons are pressed or an interrupt modifies something displayed on the menu
    updateBotControl();                                 //Function to send out the control packet to each of the bots periodically. The control packet will set items like the drive mode and also change the LED state
    manualMotorControl(botSelect);                      //Function to control motors from the joystick if the selected bot is in manual motor control mode
    if(!BLE.connected()) {                              //If bluetooth is not connected, then periodically scan for bots so at least one may have bluetooth
    	if (millis() - lastScan >= BLE_SCAN_PERIOD) {   //Check timer between scans to not scan too often
    		lastScan = millis();
    		BLEScan(-1);                                //Scan for any bot
    	}

    }
    for(WaterBot &wb: WaterBots){                       //Loop over all bots and check for offloading or sensor requests
        if(wb.offloading){                              //Check if the user requested an offload from the menu items
            DataOffloader(wb.botNum);                   //Offload from this bot
            wb.offloading = false;                      //Set flag back to false for this bot to not immediately offload again
        }
        if(wb.reqActive > 3){                           //Check if we should re-request the live sensor data from this bot
            char tempBuf[10];
            sprintf(tempBuf,"CCB%dsns",wb.reqActive);   //Send sense command which makes the request
            sendData(tempBuf,0,!(wb.XBeeAvail), true, false);
            wb.reqActive = 0;                           //Set the request active to false so it only happens at the timer intervals
        }
    }
    XBeeHandler();                                      //Check XBee serial buffer and process serial data to the command dictionary if a command was received
    RPiHandler();                                       //Check USB serial to see if Raspberry Pi or computer has sent a new control packet which contains the drive mode and target latitude and longitude
    XBeeLTEPairSet();                                   //Call pair function to see if any bots have come online after the initial pair sequence
    RPiStatusUpdate();                                  //Periodically send a status update for all of the bots to the Raspberry Pi so the user interface is populated with recent data and status
    printStatusTable();                                 //Periodically print status table for the target bot
    calibrateCompass();                                 //Check if the compass calibration request has been set by a the menu interface, and if so, pop up an info screen to tell the user to calibrate the compass
    sendTareData();                                     //Check if a new motor tare value has been set by the menu interface, and if so, send it to the bot
    if(stopActive){                                     //If the user has pressed the stop button and has not yet cleared it, periodically publish the stop button in case the bot missed the previous message
        if(millis() - stopTime > STOP_PUB_TIME){        //Check timer to publish periodically, stops sending periodically after being cleared by hitting stop again
            stopTime = millis();
            sendData("CCABstp",0,true,true,!LTEStopSent); 
            LTEStopSent = true;                         //Only send stop over LTE one time so we don't burn through cell data
        }
    }
    delay(10);
}

/**
 * @brief Log a message to the SD card
 * @param message The message string to log
 * 
 * Function to take a string and log it to the SD card by opening/closing the file.
 * Handles file management automatically.
 */
void logMessage(const char *message){           //Function to take a string and log it to the SD card by opening/closing the file
    if(!logFile.isOpen()){                      //if the file is not open, open it and then close it afterwards to ensure changes take place
        logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.println(message);
        logFile.close();
    }
    else logFile.println(message);
}

/**
 * @brief Display a single menu item on the OLED screen
 * @param id Menu item ID in the MenuItems vector
 * @param highlighted Whether this item should be highlighted (selected)
 * @param selected Whether the value should be highlighted (being modified)
 * @param x X coordinate for display
 * @param y Y coordinate for display 
 * @param wb WaterBot object to get data from
 * 
 * Abstraction function to print a single menu item with data from a given waterbot,
 * at the provided x and y coordinates. Handles highlighting and value display.
 */
void printMenuItem(uint8_t id, bool highlighted, bool selected, uint16_t x, uint16_t y, WaterBot wb){
    if(highlighted){                                            //Check if this menu item should be highlighted
        oled.fillRect(x,y,OLED_MAX_X - 40,16,1);                //Draw white rectangle, then write black text to show "highlighted" state
        oled.setCursor(x+1,y+1);                                //Set cursor for text
        oled.setTextSize(2);                                    //Set size to 2 for standard menu items
        oled.setTextColor(0);                                   //Set to black text
        oled.print(MenuItems.at(id).itemName);                  //Print out the name associated with the menu item at the "id" location in the menu vector
        if(selected){                                           //If this item is selected, then draw box to indicate to the user
            oled.fillRect(OLED_MAX_X - 40,y,OLED_MAX_X-1,16,1); //Fill area with white rectangle 
            oled.setCursor(OLED_MAX_X - 39,y+1);                //Move cursor to draw text in this box
            oled.setTextColor(0);                               //Make sure the text is set to black
            if(MenuItems.at(id).onOffSetting){                  //Determine where to source the label for the value, when true, display "On" or "Off"
                if(wb.*(MenuItems.at(id).MethodPointerBool))  oled.printf("On");    //Check if the value this menu item modifies for this waterbot is true, print "On" if true, "Off" if false
                else oled.printf("Off");
            }
            else if(MenuItems.at(id).customLabel){              //Otherwise, check if we are using custom labels, like those for the drive mode. Source the label text from the Menu item label vector
                oled.printf(MenuItems.at(id).labels.at(wb.*MenuItems.at(id).MethodPointer));
            }
            else oled.printf("%d",wb.*(MenuItems.at(id).MethodPointer));    //Otherwise, just print the number in the unsigned integer location
        }
        else{
            oled.fillRect(OLED_MAX_X - 40,y,OLED_MAX_X-1,16,0);             //Draw black rectangle to erase text, then write white text to show "unhighlighted" state
            oled.setCursor(OLED_MAX_X - 39,y+1);
            oled.setTextColor(1);                                           //Set to white text
            if(MenuItems.at(id).onOffSetting){                              //Determine where to source the label for the value, when true, display "On" or "Off"
                if(wb.*(MenuItems.at(id).MethodPointerBool))  oled.printf("On");    //Check if the value this menu item modifies for this waterbot is true, print "On" if true, "Off" if false
                else oled.printf("Off");
            }
            else if(MenuItems.at(id).customLabel){                          //Otherwise, check if we are using custom labels, like those for the drive mode. Source the label text from the Menu item label vector
                oled.printf(MenuItems.at(id).labels.at(wb.*MenuItems.at(id).MethodPointer));
            }
            else oled.printf("%d",wb.*(MenuItems.at(id).MethodPointer));    //Otherwise, just print the number in the unsigned integer location
        }
        #ifdef VERBOSE
        Serial.printlnf("Printed Highlighted Menu item with name: %s",MenuItems.at(id).itemName);
        #endif
    }
    else{                                                       //This item is not highlighted, cannot have the value highlighted either
        oled.fillRect(x,y,OLED_MAX_X - 40,16,0);                //Draw black rectangle to erase any previous contents
        oled.setCursor(x+1,y+1);                                //Move cursor for where to display label text
        oled.setTextSize(2);                                    //Set standard size for text of menu
        oled.setTextColor(1);                                   //Use white text for non-highlighted menu items
        oled.print(MenuItems.at(id).itemName);                  //Print out the name associated with the menu item at the "id" location in the menu vector
        oled.fillRect(OLED_MAX_X - 40,y,OLED_MAX_X-1,16,0);
        oled.setCursor(OLED_MAX_X - 39,y+1);
        oled.setTextColor(1);
        if(MenuItems.at(id).onOffSetting){                      //Determine where to source the label for the value, when true, display "On" or "Off"
            if(wb.*(MenuItems.at(id).MethodPointerBool))  oled.printf("On");    //Check if the value this menu item modifies for this waterbot is true, print "On" if true, "Off" if false
            else oled.printf("Off");
        }
        else if(MenuItems.at(id).customLabel){                  //Otherwise, check if we are using custom labels, like those for the drive mode. Source the label text from the Menu item label vector
            oled.printf(MenuItems.at(id).labels.at(wb.*MenuItems.at(id).MethodPointer));
        }
        else oled.printf("%d",wb.*(MenuItems.at(id).MethodPointer));    //Otherwise, just print the number in the unsigned integer location
    }
    MenuItems.at(id);
}

/**
 * @brief Update the OLED display with the current menu state
 * 
 * Redraws the OLED menu display including pop-ups, menu items, and status.
 * Handles both pop-up display mode and normal menu navigation mode.
 * Only redraws when redrawMenu flag is set for efficiency.
 */
void updateMenu(){
    if(redrawMenu){
        oled.fillRect(0,0,OLED_MAX_X,OLED_MAX_Y,0);                 //Erase full screen
        if(PopUps.size() != 0){                                     //Check if there is a queue of pop-ups to be displayed, print out the most-recent (back of vector) if any
            oled.drawRect(1,1,126,62,1);                            //Draw outer rectangle of warning box
            oled.drawRect(2,2,124,60,1);                            //Draw inner rectangle of warning box
            oled.setTextColor(1);                                   //Set text to white for all text inside the box
            oled.setCursor(PopUps.back().primaryStart,4);           //Move cursor to the location for the main warning line of the top pop-up
            oled.setTextSize(2);                                    //Use larger text for main warning
            oled.printf(PopUps.back().primaryLine);                 //Print main warning text from the top pop-up string
            oled.setCursor(PopUps.back().secondaryStart,22);        //Move cursor for second text line
            oled.setTextSize(1);                                    //Use smaller text for next two lines
            oled.printf(PopUps.back().secondaryLine);               //Print out second warning text from top pop-up string
            oled.setCursor(PopUps.back().tertiaryStart,32);         //Move cursor for third line
            oled.setTextSize(1);                                    //Continue using smaller text
            oled.printf(PopUps.back().tertiaryLine);                //Print third line
            oled.setCursor(48,45);                                  //Move cursor for displaying "OK" prompt
            oled.setTextSize(2);                                    //Use large text for "OK" prompt
            oled.fillRect(45,44,32,16,1);                           //Create a white rectangle to highlight the "OK" text
            oled.setTextColor(0);                                   //Set text color to black for printing in the white rectangle to display "OK" as highlighted
            oled.printf("OK");                                      //Print "OK" to indicate the user should hit enter to dismiss this prompt
            oled.display();                                         //Commit update to OLED screen
            redrawMenu = false;                                     //Set flag to false so it isn't redrawn every time
        }
        else{                                                       //No pop-ups to display, print the main menu interface  
            uint8_t menuSelect = 0;                                 //Counter used to find which index in the Water Bot vector the currently selected bot is at
            for(uint8_t i = 0; i < WaterBots.size(); i++){          //Loop over all of the water bots that have been discovered and check if they are the selected one
                if(WaterBots.at(i).botNum == botSelect){            //If it is the selected one, print it with a highlighted rectangle
                    oled.setCursor(5+18*i,4);                       //Set cursor to offset based on the item location in the list. Bots displayed in order they were discovered
                    oled.setTextSize(1);                            //Use small text for number inside of bot box
                    oled.setTextColor(0);                           //Set text to black on selected box since printing in white rectangle
                    oled.fillRect(1+i*18,1,14,14,1);                //Fill rectangle with white background for selected bot
                    oled.printf("%d",WaterBots.at(i).botNum);       //Then print the bot ID inside the box in black text
                    menuSelect = i;                                 //Get the vector location of the selected bot to get data to print more easily later
                }
                else{                                               //If it is not the selected one, print with an empty rectangle
                    oled.setCursor(5+18*i,4);                       //Set cursor to offset based on the item location in the list. Bots displayed in order they were discovered
                    oled.setTextSize(1);                            //Use small text for bot ID printed in box
                    oled.setTextColor(1);                           //Set white text for unselected bot as the background is black
                    oled.drawRect(1+i*18,1,14,14,1);                //Only draw outside of rectangle as white since this bot is not selected
                    oled.printf("%d",WaterBots.at(i).botNum);       //Print out the bot number in the box
                }
            }
            if(menuItem == 0){      //If we are selecting the first menu item in the list, print it first (highlighted), then print the next two unhighlighted
                //Serial.println("Menu item 0");
                if(MenuItems.size() != 0) printMenuItem(0,true,!selectingBots,0,16,WaterBots.at(menuSelect));   //Print top menu item as highlighted
                uint8_t loopIter = MenuItems.size();                                                            //Get size of menu items (check if there are < 2)
                if(loopIter > 2) loopIter = 2;                                                                  //Limit to max 3 printouts beyond first item
                for(int mi = 1; mi <= loopIter; mi++){                                                          //Print out the items below this
                    //Serial.printlnf("Menu item %d", mi);
                    printMenuItem(mi,false,!selectingBots,0,16+(16*mi),WaterBots.at(menuSelect));
                }
            }
            else if(menuItem == MenuItems.size() -1){  //If we are selecting the last menu item in the list, print it last (highlighted), then print the previous two unhighlighted
                //Serial.printlnf("Menu item %d", menuItem);
                printMenuItem(menuItem,true,!selectingBots,0,48,WaterBots.at(menuSelect));      //Print bottom menu item as highlighted
                //Serial.printlnf("Menu item %d", menuItem-1);
                printMenuItem(menuItem-1,false,!selectingBots,0,32,WaterBots.at(menuSelect));   //Print middle menu item as unhighlighted
                //Serial.printlnf("Menu item %d", menuItem-2);
                printMenuItem(menuItem-2,false,!selectingBots,0,16,WaterBots.at(menuSelect));   //Print top menu item as unhighlighted
            }
            else{                                   //Otherwise, print the selected item in the middle of the list (highlighted) with one unhighlighted item before and after
                //Serial.printlnf("Menu item %d", menuItem+1);
                printMenuItem(menuItem+1,false,!selectingBots,0,48,WaterBots.at(menuSelect));   //Print bottom menu item as unhighlighted
                //Serial.printlnf("Menu item %d", menuItem);
                printMenuItem(menuItem,true,!selectingBots,0,32,WaterBots.at(menuSelect));      //Print middle menu item as highlighted
                //Serial.printlnf("Menu item %d", menuItem-1);
                printMenuItem(menuItem-1,false,!selectingBots,0,16,WaterBots.at(menuSelect));   //Print top menu item as unhighlighted
            }
            oled.display();         //Commit drawn menu to the OLED
            redrawMenu = false;     //Clear flag so it isn't redrawn every time
        }
    }
}

/**
 * @brief Compute target latitude and longitude for a water bot
 * @param wb WaterBot object containing waypoint data
 * @param lat Reference to store computed target latitude
 * @param lon Reference to store computed target longitude
 * 
 * Helper function to calculate the current target coordinates based on
 * the bot's waypoint list and current waypoint index. Returns -999.0f
 * for invalid coordinates.
 */
static void computeTargetLatLon(const WaterBot &wb, float &lat, float &lon) {
    lat = -999.0f;
    lon = -999.0f;
    if (wb.waypointsLat.size() > wb.waypointIndex && wb.waypointsLon.size() > wb.waypointIndex) {
        lat = wb.waypointsLat.at(wb.waypointIndex);
        lon = wb.waypointsLon.at(wb.waypointIndex);
    }
}

/**
 * @brief Build a control packet string for transmission to a water bot
 * @param wb WaterBot object containing control parameters
 * @param outBuf Output buffer to store the formatted control packet
 * @param outLen Maximum length of the output buffer
 * 
 * Constructs a control packet in the format "CCBXctlLAT LON MODE RECORD SIGNAL"
 * where X is the bot number. Uses current waypoint coordinates and bot settings.
 */
static void buildControlPacket(const WaterBot &wb, char *outBuf, size_t outLen) {
    float lat, lon;
    computeTargetLatLon(wb, lat, lon);
    snprintf(outBuf, outLen, "CCB%dctl%0.6f %0.6f %d %d %d",
             wb.botNum, lat, lon, wb.driveMode, wb.dataRecording, wb.signal);
}

/**
 * @brief Update and send control packets to all water bots
 * 
 * Periodically sends control commands to all discovered water bots.
 * Handles waypoint progression for autonomous bots and transmits
 * control packets via the appropriate communication method (BLE/XBee).
 */
void updateBotControl(){                    //Function to send control packet to bot periodically and when updated by user. 
    for(WaterBot &wb: WaterBots){           //Loop over all discovered water bots and check for waypoint update
        if(wb.waypointArrived && wb.waypointIndex == wb.botWaypointIndex && wb.waypointIndex < wb.waypointsLat.size()-1 && wb.driveMode == DRIVE_MODE_AUTONOMOUS){ //Check if the bot has arrived at its current waypoint and there are more waypoints to go to
            wb.waypointIndex++;          //Increment waypoint index to move to next waypoint if the bot has arrived at the current waypoint
        }
    }
    if(updateControl){                      //Check flag that is set when menu items are modified
        updateControl = false;              //Clear flag to not constantly publish updates
        //ControlledBot = nullptr;
        for(WaterBot &wb: WaterBots){       //Loop over discovered water bots and check if this bot has had one of its assets changed
            //if(wb.botNum == botSelect) ControlledBot =  &wb;
            if(wb.updatedControl){          //Flag represents a change of at least one modified asset - re-send control packet
                wb.updatedControl = false;  //Clear flags for this bot
                wb.publishTime = millis();  //Inidicate the last time this bot had its control packet sent
                char statusStr[42];         //String to hold the control packet
                buildControlPacket(wb, statusStr, sizeof(statusStr)); //Build control packet string
                #ifdef VERBOSE
                Serial.printlnf("Control Packet: %s",statusStr);
                #endif
                bool rpiLTEStatus = false;      //Flag to determine if LTE status should be sent
                if(!wb.XBeeAvail && !wb.BLEAvail && LTEStatuses && wb.LTEInitialStatus){    //Don't publish LTE when XBee or BLE are available
                    rpiLTEStatus = true;
                    LTEStatuses--;              //Counter to limit number of LTE messages sent per power-up of this bot. Limits LTE usage over long periods
                }
                sendData(statusStr,0,!(wb.XBeeAvail),true,rpiLTEStatus);    //Send out data over the available method
                wb.LTEInitialStatus = false;    //Flag set true when a status has been changed - don't send status if something hasn't been changed
            }
        }
    }
    if(millis() - controlUpdateTime > CONTROL_PUB_TIME){        //Periodically send out the control packet - fixes problem when packets are missed
        controlUpdateTime = millis();                           //Update timer for sending status update
        if(!publishedFirstStatus){                              //Check if this is the first status update
            if(WaterBots.size() != 0) publishedFirstStatus = true;      //Make sure some bots have been discovered before trying to send periodic status updates
            else return;                                        //Do nothing if no bots have been discovered
        }
        if(controlUpdateID > WaterBots.size()-1) controlUpdateID = 0;   //Go around the vector of water bots circularly so each one is published at some point.
        WaterBot wb = WaterBots.at(controlUpdateID);                    //Get a copy of the water bot in the vector that is at the circular pointer location
        char statusStr[42];                                             //Create a status string
        buildControlPacket(wb, statusStr, sizeof(statusStr));           //Populate string with command packet
        bool sendLTEStat = false;                               //Determine if we should send LTE on this go-around, limits periodic publishing over LTE to reduce usage
        if(!wb.XBeeAvail && !wb.BLEAvail && (millis() - wb.LTELastStatTime > LTE_CTL_PERIOD)){  //Determine if LTE should be used based on the availability of other communication modes
            sendLTEStat = true;                                 //Set flag true if we should use LTE
            WaterBots.at(controlUpdateID).LTELastStatTime = millis();   //Set the timer for this water bot to indicate the last time LTE status was sent for it
        }
        sendData(statusStr,0,false,true,sendLTEStat);           //Send out the data over the determined communication mode
        if(controlUpdateID < WaterBots.size()-1) controlUpdateID++; //Advance the pointer for the circular buffer so the next bot in the vector is sent next time
        else controlUpdateID = 0;
    }
    static uint8_t lastBotCount = 0;                            //Static variable to hold the last number of bots discovered so we can check if the number of bots has changed
    if(lastBotCount != WaterBots.size()){                       //Check if the number of bots has changed since last time
        lastBotCount = WaterBots.size();                        //Update the last bot count to the current number of bots
        char publishRateStr[20];                                //Create a string to hold the publish rate command
        sprintf(publishRateStr,"CCABsdp%d", (int)lastBotCount); //Create the publish rate command string the publish rate will be one second delay for each bot in the system
        sendData(publishRateStr,0,true,true,true);              //Send out the publish rate command over XBee and LTE if available
    }
}

/**
 * @brief Handle status update commands from water bots
 * @param dataStr Data string containing status information
 * @param rxBotID Bot ID that sent the status update
 * @param mode Communication mode (BLE/XBee/LTE)
 * 
 * Processes status update messages from water bots including battery level,
 * GPS coordinates, motor speeds, power readings, and various status flags.
 * Updates the corresponding WaterBot object with received data.
 */
void handleStatusUpdateCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode) {
    for(WaterBot &w: WaterBots){                            //Loop over available bots in the Water Bot vector
        if(rxBotID == w.botNum){                            //Find the bot in the vector that matches the source of the status update
            //Serial.printlnf("Status update: %s", dataStr);
            unsigned int battpct;                                //Create local variables that the string will be parsed into
            unsigned int statflags;
            char testLat[12];                               //Have to use char arrays to copy in floats because C++ is not always smart
            char testLon[12];
            int panelPwr, battPwr;
            int lSpeed, rSpeed;
            unsigned int rxMotorTare;
            unsigned int compassHeading, targetHeading;
            unsigned int currentWaypointIndex;
            sscanf(dataStr,"%u %u %s %s %d %d %u %u %d %d %u %u",
                &battpct,
                &statflags,
                testLat,testLon, 
                &battPwr, &panelPwr, 
                &compassHeading, &targetHeading, 
                &lSpeed, &rSpeed, &rxMotorTare,
                &currentWaypointIndex);                     //Parse the status update string into the local variables
            w.battPercent = battpct;                        //Copy in battery percent from the status update
            w.LTEAvail = statflags & 1;                     //Statflags is a bit-masked number to transmit multiple booleans using an integer. Bit 0 in the number represents if LTE is available
            w.XBeeAvail = (statflags >> 1) & 1;             //Bit 1 represents if XBee is available
            w.BLEAvail = (statflags >> 2) & 1;              //Bit 2 represents if BLE is available
            w.lowBatt = (statflags >> 6) & 1;               //Bit 6 represents if this bot has low battery
            w.GPSAvail = (statflags >> 8) & 1;              //Bit 8 represents if the GPS module is functional
            w.CompassAvail = (statflags >> 9) & 1;          //Bit 9 represents if the Compass module is functional
            w.SDAvail = (statflags >> 10) & 1;              //Bit 10 represents if the SD card is functional
            w.waypointArrived = (statflags >> 11) & 1;      //Bit 11 represents if the bot has arrived at its waypoint
            w.botWaypointIndex = currentWaypointIndex;      //Copy in the current waypoint index from the status update
            w.GPSLat = atof(testLat);                       //Convert the decimal in string form to a floating point number for latitude
            w.GPSLon = atof(testLon);                       //Convert the decimal in string form to a floating point number for longitude
            w.panelPower = panelPwr;                        //Copy in the solar panel power measurement
            w.battPower = battPwr;                          //Copy in the battery power measurement
            w.updatedStatus = true;                         //Indicate that this bot has received new data for printing to menu
            w.CompassHeading = compassHeading;              //Copy in the compass heading from the status update
            w.targetHeading = targetHeading;                //Copy in the target heading from the status update
            w.leftMotorSpeed = lSpeed;                      //Copy in the left motor speed from the status
            w.rightMotorSpeed = rSpeed;                     //Copy in the right motor speed from the status
            if(rxMotorTare >= 0 && rxMotorTare <= 200) {
                w.motorTare = rxMotorTare;                  //Copy in the motor tare value from the status update, but only if it is a valid number
                if(!w.motorTareReceived){                   //If this is the first valid motor tare value received, update the last tare value so we don't immediately retransmit
                    w.lastMotorTare = rxMotorTare;
                }
                w.motorTareReceived = true;                 //Set the flag indicating a valid motor tare value has been received
            }

            if(millis() - w.publishTime > WB_MOD_UPDATE_TIME){      //Check the last time a status change was published to this bot, this prevents a status update from reverting a change the user made due to latency, ignore a status update for modifiable fields for some time
                w.offloading = (statflags >> 3) & 1;                //Copy in offloading, drive mode and sensor data recording if it hasn't been sent out too recently
                w.driveMode = (statflags >> 4) & 3;
                w.dataRecording = (statflags >> 7) & 1;
            }
            if(w.lowBatt && !w.warnedLowBatt){              //Check the status of the power system for low battery, warn the user if this bot's battery is low
                w.warnedLowBatt = true;                     //Set this flag after the user has been warned so they don't get spammed every status update
                MenuPopUp m;                                //Create a pop-up for the low battery warning
                sprintf(m.primaryLine,"Warning");         //Populate strings of the low battery warning with the bot number
                sprintf(m.secondaryLine,"Bot %d", w.botNum);
                sprintf(m.tertiaryLine, "Low Battery: %d",w.battPercent);
                m.primaryStart = 20;                        //Calculated offsets so the strings are centered in the box - determined from experimentation
                m.secondaryStart = 40;
                m.tertiaryStart = 20;
                PopUps.push_back(m);                        //Push the pop-up item onto the vector so the menu updater prints it
                redrawMenu = true;                          //Set flag so the menu item updater draws it in
                
            }
            if(!w.SDAvail && !w.warnedSDCard){              //Check if the SD card had a warning
                w.warnedSDCard = true;                      //Set this flag after the user has been warned so they don't get spammed every status update
                MenuPopUp m;                                //Create a pop-up for the SD card warning
                sprintf(m.primaryLine,"Warning");         //Populate strings of the SD card warning
                sprintf(m.secondaryLine,"Bot %d", w.botNum);
                sprintf(m.tertiaryLine, "SD Card Failed");
                m.primaryStart = 20;                        //Calculated offsets so the text is centered in the box - determined by experimentation
                m.secondaryStart = 40;
                m.tertiaryStart = 20;
                PopUps.push_back(m);                        //Push the pop-up item onto the vector so the menu updater prints it
                redrawMenu = true;                          //Set flag so the menu item updater draws it in
            }
            if((!w.CompassAvail || !w.GPSAvail) && !w.warnedTelem){     //Check if the compass or GPS had an error
                MenuPopUp m;                                            //Create a pop-up for them - see comments for SD card and battery warnings above for how the details of creating a pop-up
                w.warnedTelem = true;
                sprintf(m.primaryLine,"Warning");
                sprintf(m.secondaryLine,"Bot %d", w.botNum);
                sprintf(m.tertiaryLine, "GPS/Compass Error");
                m.primaryStart = 20;
                m.secondaryStart = 40;
                m.tertiaryStart = 10;
                PopUps.push_back(m);
                redrawMenu = true;
            }
            if(botSelect == w.botNum) redrawMenu = true;        //Redraw the menu if the selected bot got the status update because it's telemetry got refreshed
            logMessage("Status Update!");
            /*Serial.println("##########################");
            Serial.println("##     STATUS UPDATE    ##");
            Serial.printlnf("##       Bot #: %1d      ##",w.botNum);
            Serial.printlnf("##      Batt %: %03d     ##",w.battPercent);
            Serial.println("##    LTE  BLE  XBee    ##");
            Serial.printlnf("##     %d    %d     %d     ##",w.LTEAvail,w.BLEAvail,w.XBeeAvail);
            Serial.println("##  Latitude Longitude  ##");
            Serial.printlnf("## %.6f %.6f ##",w.GPSLat,w.GPSLon);
            Serial.println("##########################");*/
        }
    }
}

/**
 * @brief Handle sensor data commands from water bots
 * @param dataStr Data string containing sensor readings and GPS coordinates
 * @param rxBotID Bot ID that sent the sensor data
 * @param mode Communication mode (BLE/XBee/LTE)
 * 
 * Processes sensor data messages including GPS coordinates, dissolved oxygen,
 * pH, conductivity, and temperature readings. Updates the corresponding
 * WaterBot object.
 */
void handleSensorCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode) {
    char GPSLatstr[12];         //Use strings for GPS latitude and longitude because scanf doesn't like scanning in floating point numbers
    char GPSLonstr[12];
    uint32_t do_in,pH_in,cond_in,mcond_in,temp_in;      //Variables to hold decimal-shifted sensor readings
    sscanf(dataStr,"%s %s %lu %lu %lu %lu %lu",GPSLatstr,GPSLonstr,&do_in,&pH_in,&cond_in,&mcond_in,&temp_in);       //Scan in the sensor readings and the GPS locations
    
    // Find the target WaterBot using the rxBotID
    for(WaterBot &w: WaterBots){
        if(rxBotID == w.botNum){
            w.DO = ((float)do_in)/1000.0;               //Update the bot object with the sensor readings - divide by 1000 to shift back the decimal
            w.pH = ((float)pH_in)/1000.0;
            w.Cond = ((float)cond_in)/1000.0;
            w.MCond = ((float)mcond_in)/1000.0;
            w.temp = ((float)temp_in)/1000.0;
            #ifdef VERBOSE
            Serial.printlnf("Bot #: %d Temp: %f", w.botNum, w.temp);
            #endif
            break;  // Found the bot, no need to continue looping
        }
    }
}

/**
 * @brief Handle hello world commands from water bots during initial discovery
 * @param dataStr Data string containing hello world message
 * @param rxBotID Bot ID that sent the hello world message
 * @param mode Communication mode (BLE/XBee/LTE)
 * 
 * Processes hello world messages from bots during startup/discovery phase.
 * Creates new WaterBot objects for newly discovered bots and sends
 * acknowledgment responses to establish communication.
 */
void handleHelloWorldCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode) {
    bool newBot = true;          //Assumming this is probably a new bot since it should be on startup
    for(WaterBot w: WaterBots){  //Just in case, check if this bot was already discovered, as the bot could have restarted somehow
        if(rxBotID == w.botNum) newBot = false;     //Don't create a new bot if it was already discovered
    }
    if(newBot){
        #ifdef VERBOSE
        Serial.println("Found a new water bot ID");
        #endif
        WaterBot newWaterbot;                           //Create a new object for the discovered water bot
        if(mode == 1) newWaterbot.BLEAvail = true;      //Check which mode of communication the hello world message was received over, assume that mode is available
        else if(mode == 2) newWaterbot.XBeeAvail = true;
        else if(mode == 3) newWaterbot.LTEAvail = true;
        newWaterbot.botNum = rxBotID;                   //Copy in the bot number for display on the list
        WaterBots.push_back(newWaterbot);               //Push this bot onto the main vector of bots since it's now discovered and set-up
        PairBots.push_back(newWaterbot);
        redrawMenu = true;                              //Redraw the menu since the top list of discovered bots will now have this one to show
    }
}

/**
 * @brief Handle debug print commands from water bots
 * @param dataStr Data string containing the message to log
 * @param rxBotID Bot ID that sent the string message
 * @param mode Communication mode (BLE/XBee/LTE)
 * 
 * Processes debug print commands from bots which are used for general
 * text message logging. Logs the received messages to the SD card
 * for debugging and monitoring purposes.
 */
void handlePutStringCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode) {
    if(!logFile.isOpen()){
        logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[PUTS] Received String Command: %s",dataStr);
        logFile.close();
    }
    else logFile.printlnf("[PUTS] Received String Command: %s",dataStr);
}

/**
 * @brief Handle leak detection commands with automatic shutoff
 * @param cmdStr Command string containing leak detection data
 * @param rxBotID Bot ID that detected the leak
 * @param mode Communication mode (BLE/XBee/LTE)
 * 
 * Processes leak detection alerts from water bots. Creates warning
 * pop-ups on the display and automatically initiates emergency
 * shutdown procedures for the affected bot.
 */
void handleLeakDetectionCommand(const char *cmdStr, uint8_t rxBotID, uint8_t mode) {
    MenuPopUp m;                                            //Create pop-up item to warn the user
    sprintf(m.primaryLine,"Warning");                     //Print warning strings into the pop-up item
    sprintf(m.secondaryLine,"Bot %d", rxBotID);
    sprintf(m.tertiaryLine, "Leak shutoff");
    m.primaryStart = 20;                                    //Calculated offsets to center the text in the box
    m.secondaryStart = 40;
    m.tertiaryStart = 30;
    PopUps.push_back(m);                                    //Push onto the queue of warnings to be displayed
    redrawMenu = true;                                      //Indicate to the menu drawer that we should update now
}

/**
 * @brief Handle leak warning commands without automatic shutoff
 * @param cmdStr Command string containing leak warning data
 * @param rxBotID Bot ID that detected the leak
 * @param mode Communication mode (BLE/XBee/LTE)
 * 
 * Processes leak warning alerts from water bots. Creates warning
 * pop-ups on the display to notify operators of potential leaks
 * without initiating automatic shutdown procedures.
 */
void handleLeakWarningCommand(const char *cmdStr, uint8_t rxBotID, uint8_t mode) {
    MenuPopUp m;                                            //Create pop-up item to warn the user
    sprintf(m.primaryLine,"Warning");                     //Print warning strings into the pop-up item
    sprintf(m.secondaryLine,"Bot %d", rxBotID);
    sprintf(m.tertiaryLine, "Leak detected");
    m.primaryStart = 20;                                    //Calculated offsets to center the text in the box
    m.secondaryStart = 40;
    m.tertiaryStart = 25;
    PopUps.push_back(m);                                    //Push onto the queue of warnings to be displayed
    redrawMenu = true;                                      //Indicate to the menu drawer that we should update now
}

/**
 * @brief Print a formatted status table for the target water bot
 * 
 * Displays a detailed status table over serial communication showing
 * comprehensive information about the target bot including GPS coordinates,
 * sensor readings, motor speeds, battery status, and operational flags.
 * Updates once per second when enabled.
 */
void printStatusTable() {
    if(targetTableBot < 0) return;                         //Don't print if targetTableBot is negative
    
    if(millis() - tableUpdateTime >= 1000) {               //Print table once per second
        tableUpdateTime = millis();                        //Update timer
        
        //Find the target bot in the WaterBots vector
        WaterBot* targetBot = nullptr;
        for(WaterBot &wb: WaterBots) {
            if(wb.botNum == targetTableBot) {
                targetBot = &wb;
                break;
            }
        }
        
        if(targetBot == nullptr) return;                   //Target bot not found, don't print
        
        float lat = -999.0f, lon = -999.0f;
        if(targetBot->waypointsLat.size() > targetBot->waypointIndex && targetBot->waypointsLon.size() > targetBot->waypointIndex) {
            lat = targetBot->waypointsLat.at(targetBot->waypointIndex);
            lon = targetBot->waypointsLon.at(targetBot->waypointIndex);
        }

        //Print status table header
        Serial.println("##########################");
        Serial.println("##   CCHUB STATUS TABLE ##");
        Serial.printlnf("##       Bot #: %1d      ##", targetBot->botNum);
        Serial.printlnf("##      Batt %%: %03d     ##", targetBot->battPercent);
        Serial.println("##    LTE  BLE  XBee    ##");
        Serial.printlnf("##     %d    %d     %d     ##", targetBot->LTEAvail, targetBot->BLEAvail, targetBot->XBeeAvail);
        Serial.println("##  Latitude Longitude  ##");
        Serial.printlnf("## %8.6f %8.6f ##", targetBot->GPSLat, targetBot->GPSLon);
        Serial.println("##   Target Lat/Lon     ##");
        Serial.printlnf("## %8.6f %8.6f ##", lat, lon);
        Serial.println("## Drive Mode | Record  ##");
        Serial.printlnf("##     %d      |   %d    ##", targetBot->driveMode, targetBot->dataRecording);
        Serial.println("##   Motor Speeds L/R   ##");
        Serial.printlnf("##    %3d     |  %3d   ##", targetBot->leftMotorSpeed, targetBot->rightMotorSpeed);
        Serial.println("##     Compass/Target   ##");
        Serial.printlnf("##    %3d°    | %3d°   ##", targetBot->CompassHeading, targetBot->targetHeading);
        Serial.println("##      Water Quality   ##");
        Serial.printlnf("## pH:%4.1f  DO:%4.1f  ##", targetBot->pH, targetBot->DO);
        Serial.printlnf("## T:%4.1f°C Cond:%4.0f ##", targetBot->temp, targetBot->Cond);
        Serial.println("##    Panel | Battery   ##");
        Serial.printlnf("##   %4dW  |  %4dW   ##", targetBot->panelPower, targetBot->battPower);
        Serial.println("##  GPS | Compass | SD  ##");
        Serial.printlnf("##   %d  |    %d    | %d  ##", targetBot->GPSAvail, targetBot->CompassAvail, targetBot->SDAvail);
        Serial.println("##########################");
        Serial.println();
    }
}

// Command lookup table for cleaner command processing
const CommandEntry commandTable[] = {
    {"sup", handleStatusUpdateCommand},                 // Status update command
    {"sns", handleSensorCommand},                       // Sensor reading command
    {"hwd", handleHelloWorldCommand},                   // Hello World command
    {"pts", handlePutStringCommand},                    // Put string command
    {"ldt", handleLeakDetectionCommand},                // Leak detection with shutoff
    {"ldb", handleLeakDetectionCommand},                // Battery leak detection with shutoff
    {"wld", handleLeakWarningCommand},                  // Leak warning without shutoff
    {"wlb", handleLeakWarningCommand}                   // Battery leak warning without shutoff
};
const int commandTableSize = sizeof(commandTable) / sizeof(CommandEntry);

// RPi command lookup table for cleaner RPi command processing
const CommandEntry rpiCommandTable[] = {
    {"cbp", handleRPiChecksumBypassCommand}, // Toggle checksum bypass
    {"ctl", handleRPiControlCommand},        // Control packet from Raspberry Pi/computer
    {"wyp", handleRPiAddWaypointCommand},     // Add waypoint command
    {"tbl", handleRPiTableCommand}           // Enable/disable status table printing
};
const int rpiCommandTableSize = sizeof(rpiCommandTable) / sizeof(CommandEntry);

/**
 * @brief Process received commands and route them to appropriate handlers
 * @param command Complete command string received from a bot
 * @param mode Communication mode the command was received on
 * @param sendAck Whether to send acknowledgment response
 * 
 * Main command processing function that validates checksums, parses command
 * structure, and routes commands to appropriate handler functions based on
 * command lookup tables. Handles both bot and RPI command dictionaries.
 */
void processCommand(const char *command, uint8_t mode, bool sendAck){
    //Process if command is addressed to this bot "Bx" or all bots "AB"
    if((command[2] == 'A' && command[3] == 'B') || (command[2] == 'C' && command[3] == 'C')){   //Check if the message was addressed to the CChub or all bots (as a broadcast)
        char rxIDBuf[1];                                            //Get the id of the bot that sent this message to find the bot in the WaterBots vector
        rxIDBuf[0] = command[1];                                    //Convert the source bot to a single character
        uint8_t rxBotID = atoi(rxIDBuf);                            //Convert the character to an integer for comparison
        if(rxBotID > 9) return;                                     //Only accept bot ids between 0 and 9
        uint8_t checksum;                                           //Value of the checksum received, a hexadecimal (base 16) number at the end of the string
        char dataStr[strlen(command)-8];                            //Create array to hold the data portion of the received message
        dataStr[strlen(command)-9] = '\0';                          //Set null character to allow use of string compare functions (causes a nasty bug of flowing to surrounding memory otherwise)
        char cmdStr[4];                                             //String to hold 3-byte command, compared in dictionary
        cmdStr[3] = '\0';                                           //Set null character to allow use of string compare functions (causes a nasty bug of flowing to surrounding memory otherwise)
        char checkStr[3];                                           //2-byte checksum which is compared to the string length
        checkStr[0] = command[strlen(command)-2];                   //Copy in the last 2 characters from the end of the message
        checkStr[1] = command[strlen(command)-1];                   //Copy in the last 2 characters from the end of the message
        checkStr[2] = '\0';                                         //Null terminator for string compare
        checksum = (uint8_t)strtol(checkStr, NULL, 16);             //Convert to number base 16
        #ifdef VERBOSE
        Serial.printlnf("Checksum: %02x, %03d",checksum,checksum);
        #endif
        for(uint8_t i = 4; i < strlen(command)-2;i++){              //Copy in the data portion to the data string
            if(i < 7) cmdStr[i-4] = command[i];
            else dataStr[i-7] = command[i];
        }
        if(checksum != strlen(command)-2){                          //Compare the checksum to the string length and return if they don't match (reject this message, wait for the next periodic broadcast)
            #ifdef VERBOSE
            Serial.printlnf("String Len: %d, Checksum: %d",strlen(command)-2,checksum);
            Serial.println("Warning, checksum does not match");
            #endif
            logMessage("[WARN] Warning, checksum does not match!"); //Log warning to SD card
            return;
        }
        bool newBot = true;                                         //Flag set true if a new bot id has been discovered from the command
        int index = 0;                                              //Index in the vector for the water bot this message was received from
        for(WaterBot w: WaterBots){                                 //Loop over the discovered Water Bots in the vector
            if(rxBotID == w.botNum){                                //Check if the received ID matches this one in the vector, if it does, then it's not a new bot - get the pointer to it
                newBot = false;
            }
            index++;
        }
        if(newBot){                                                 //If we've received a new ID in the command source register, create a new bot object for it and push it into the main vector
            WaterBot newWaterbot;                                   //Create new object to put in vector    
            newWaterbot.botNum = rxBotID;                           //Update the object with the new bot's ID 
            WaterBots.push_back(newWaterbot);                       //Push it into the vector of Water Bots
            redrawMenu = true;                                      //Set the menu redraw flag so the list of available bots will show this one on the next refresh
        }       
        
        // Route commands to appropriate helper functions using command table
        bool commandFound = false;
        for (int i = 0; i < commandTableSize; i++) {
            if (!strcmp(cmdStr, commandTable[i].command)) {
                // Call the appropriate handler function
                commandTable[i].handler(dataStr, rxBotID, mode);
                commandFound = true;
                break;
            }
        }
        
        // Log unknown commands for debugging
        if (!commandFound) {
            #ifdef VERBOSE
            Serial.printlnf("Unknown command received: %s", cmdStr);
            #endif
            char logMsg[50];
            sprintf(logMsg, "[WARN] Unknown command: %s from bot %d", cmdStr, rxBotID);
            logMessage(logMsg);
        }
    }
}

/**
 * @brief Process commands received from Raspberry Pi or computer via USB
 * @param command Complete command string received from RPi
 * @param mode Communication mode the command was received on
 * 
 * Processes commands from the Raspberry Pi control system. Validates
 * checksums, parses command structure, and routes to appropriate RPi
 * command handlers based on the RPi command lookup table.
 */
void processRPiCommand(const char *command, uint8_t mode){
    if(command[0] == 'R' && command[1] == 'P'){         //Check that we have received a Pi message and not some kind of garbage
        #ifdef VERBOSE
        Serial.println("Received Pi command");
        #endif
        uint8_t checksum;                               //Checksum contained at the end of the received command
        char dataStr[strlen(command)-8];                //String to hold the data portion of the command
        dataStr[strlen(command)-9] = '\0';              //Put null terminator so string functions don't flow into surrounding memory - a time-costly bug
        char cmdStr[4];                                 //String to contain the three-byte string char
        cmdStr[3] = '\0';                               //Termination character to use string compare properly
        char checkStr[3];                               //String to extract checksum
        checkStr[0] = command[strlen(command)-2];       //Get last two characters
        checkStr[1] = command[strlen(command)-1];
        checkStr[2] = '\0';                             
        checksum = (uint8_t)strtol(checkStr, NULL, 16);       // number base 16
        #ifdef VERBOSE
        Serial.printlnf("Checksum: %02x, %03d, Checkstr: %s",checksum,checksum,command);
        #endif
        uint8_t terminationCharacters = rpiChecksumBypassed ? 0 : 2; // Adjust for checksum characters if not bypassed
        for(uint8_t i = 4; i < strlen(command) - terminationCharacters;i++){  //Copy in data characters to data string
            if(i < 7) cmdStr[i-4] = command[i];
            else dataStr[i-7] = command[i];
        }

        // Handle cbp command before checksum enforcement
        if(!strcmp(cmdStr,"cbp")){
            handleRPiChecksumBypassCommand(dataStr, 0, mode);
            return;
        }

        // Enforce checksum unless bypassed via cbp
        if(checksum != strlen(command)-2 && !rpiChecksumBypassed){              //Compare checksum, reject command if the checksum does not match
            #ifdef VERBOSE
            Serial.printlnf("String Len: %d, Checksum: %d",strlen(command)-2,checksum);
            #endif
            if(!logFile.isOpen()){                      //Log warning to SD card when non-matching strings are caught
                logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
                logFile.printlnf("[WARN] RPi Message Checksum Does Not Match!: %s",command);
                logFile.close();
            }
            else logFile.printlnf("[WARN] RPi Message Checksum Does Not Match!: %s",command);
            int expectedChecksum = strlen(command) - 2;
            Serial.printlnf("Warning, checksum does not match. Command ignored. Expected checksum of '%02x' at end", expectedChecksum);
            return;               //Only return if checksum bypass is not enabled
        }

        // Route commands to appropriate helper functions using RPi command table
        bool commandFound = false;
        for (int i = 0; i < rpiCommandTableSize; i++) {
            if (!strcmp(cmdStr, rpiCommandTable[i].command)) {
                rpiCommandTable[i].handler(dataStr, 0, mode);
                commandFound = true;
                break;
            }
        }
        
        // Log unknown commands for debugging
        if (!commandFound) {
            #ifdef VERBOSE
            Serial.printlnf("Unknown RPi command received: %s", cmdStr);
            #endif
            char logMsg[50];
            sprintf(logMsg, "[WARN] Unknown RPi command: %s", cmdStr);
            logMessage(logMsg);
        }
    }
}

/**
 * @brief Handle RPi checksum bypass toggle command
 * @param dataStr Data string (unused for this command)
 * @param rxBotID Bot ID (unused for this command)
 * @param mode Communication mode
 * 
 * Toggles the checksum bypass flag for RPi communications. When enabled,
 * allows commands from the Raspberry Pi to bypass checksum validation
 * for debugging purposes.
 */
void handleRPiChecksumBypassCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode){
    rpiChecksumBypassed = !rpiChecksumBypassed;
    if(rpiChecksumBypassed) Serial.println("Warning, checksum bypass enabled");
    else Serial.println("Checksum enforced");
}

/**
 * @brief Handle control commands from Raspberry Pi
 * @param dataStr Data string containing control parameters
 * @param rxBotID Bot ID (unused for RPi commands)
 * @param mode Communication mode
 * 
 * Processes control commands from the Raspberry Pi including target
 * coordinates, drive mode, recording settings, and other operational
 * parameters. Updates the specified bot's control state accordingly.
 */
void handleRPiControlCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode){
    //RPCCctl B0 34.123456 -118.123456 0 0 0 0
    char idStr[10] = {0};
    unsigned int offloading = 0, drivemode = 0, recording = 0, signal = 0;
    int parsed = sscanf(dataStr, "%9s %u %u %u %u", idStr, &drivemode, &offloading, &recording, &signal);
    if (parsed != 7) {
        Serial.println("Error: invalid ctl payload from RPi");
        return;
    }
    // idStr is expected as "B#"; fall back to numeric if not in that form
    uint8_t targetBot = 0;
    if (idStr[0] == 'B' || idStr[0] == 'b') targetBot = (uint8_t)atoi(&idStr[1]);
    else targetBot = (uint8_t)atoi(idStr);

    #ifdef VERBOSE
    Serial.printlnf("Got a command packet from Pi for Bot %d", targetBot);
    #endif

    for(WaterBot &wb: WaterBots){               //Loop over discovered water bots and look for the target water bot this control packet addresses
        if(wb.botNum == targetBot){             //Found the one we are targeting
            wb.driveMode = drivemode;           //Update control fields
            wb.offloading = offloading;         //Request SD card data offload
            wb.dataRecording = recording;       //Enable/disable data recording
            wb.signal = signal;                 //Signal LED
            wb.LTEInitialStatus = true;         //Flag for sending over LTE since the status has changed
            if(botSelect == wb.botNum) redrawMenu = true;   //Redraw the menu if this is the displayed bot
            wb.updatedControl = true;           //Trigger control packet send to the modified bot
            updateControl = true;               //Signal to control publisher that a bot has been updated
            // Print out the new control values for this vehicle
            Serial.printlnf(
                "CTL updated B%u: drive=%u offload=%u record=%u signal=%u",
                wb.botNum, wb.driveMode, wb.offloading, wb.dataRecording, wb.signal
            );
            return;
        }
    }
}

/**
 * @brief Handle add waypoint commands from Raspberry Pi
 * @param dataStr Data string containing bot ID and waypoint coordinates
 * @param rxBotID Bot ID (unused for RPi commands)
 * @param mode Communication mode
 * 
 * Processes waypoint addition commands from the Raspberry Pi in the format
 * "B# lat lon" where B# is the target bot identifier and lat/lon are the
 * GPS coordinates. Validates coordinate ranges and adds the waypoint to
 * the specified bot's waypoint list for autonomous navigation.
 */
void handleRPiAddWaypointCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode){
    char idStr[10] = {0};
    char GPSLatstr[12] = {0};
    char GPSLonstr[12] = {0};
    int parsed = sscanf(dataStr, "%9s %11s %11s", idStr, GPSLatstr, GPSLonstr);
    if (parsed != 3) {
        Serial.println("Error: invalid add waypoint payload from RPi");
        return;
    }
    // idStr is expected as "B#"; fall back to numeric if not in that form
    uint8_t targetBot = 0;
    if (idStr[0] == 'B' || idStr[0] == 'b') targetBot = (uint8_t)atoi(&idStr[1]);
    else targetBot = (uint8_t)atoi(idStr);

    float lat = atof(GPSLatstr);
    float lon = atof(GPSLonstr);
    if(lat < -90.0f || lat > 90.0f || lon < -180.0f || lon > 180.0f){
        Serial.println("Error: invalid latitude or longitude");
        return;
    }

    #ifdef VERBOSE
    Serial.printlnf("Got an add waypoint command from Pi for Bot %d", targetBot);
    #endif

    for(WaterBot &wb: WaterBots){               //Loop over discovered water bots and look for the target water bot this control packet addresses
        if(wb.botNum == targetBot){             //Found the one we are targeting
            wb.waypointsLat.push_back(lat);     //Add the new waypoint to the list
            wb.waypointsLon.push_back(lon);
            Serial.printlnf("Added waypoint to B%u: lat=%f lon=%f", wb.botNum, lat, lon);
            return;
        }
    }
}

/**
 * @brief Handle status table control commands from Raspberry Pi
 * @param dataStr Data string containing bot ID for table printing
 * @param rxBotID Bot ID (unused for RPi commands)
 * @param mode Communication mode
 * 
 * Processes table control commands from the Raspberry Pi to enable or
 * disable periodic status table printing to the serial console. A positive
 * bot ID enables table printing for that specific bot, while a negative
 * value disables table printing entirely. The status table includes
 * comprehensive telemetry data updated once per second.
 */
void handleRPiTableCommand(const char *dataStr, uint8_t rxBotID, uint8_t mode){
    sscanf(dataStr, "%d", &targetTableBot);
    if(targetTableBot >= 0){
        Serial.printlnf("Enabling table printing for bot %d", targetTableBot);
    }
    else{
        Serial.printlnf("Disabling table printing");
        targetTableBot = -1;
    }
}

/**
 * @brief Initialize XBee radio modules for communication
 * 
 * Sends bypass characters to XBee modules with integrated microcontrollers
 * to enable transparent communication mode. This setup is performed during
 * system startup to configure the XBee radios properly.
 */
void setupXBee(){           //Function to send the bypass characters to XBee modules that have the integrated microcontrollers - done on startup
    Serial1.printf("\n");    //First character to set Bypass mode
    delay(20);              //Wait some time before sending next character
    Serial1.printf("B");     //Second character to set Bypass mode
    delay(20);
}

/**
 * @brief Scan for and connect to water bots via Bluetooth Low Energy
 * @param BotNumber Specific bot number to connect to, or scan all if negative
 * 
 * Performs BLE scanning to discover nearby water bots advertising their
 * services. Can either scan for all available bots or connect to a specific
 * bot number. Handles BLE connection establishment and service discovery.
 */
void BLEScan(int BotNumber){        //Function to scan for nearby bots advertising over BLE or connect to a specific one.
    size_t count = BLE.scan(scanResults, SCAN_RESULT_COUNT);                //Get number of found BLE devices
	if (count > 0) {                                                        //Check if any were found
		for (uint8_t ii = 0; ii < count; ii++) {                            //Loop over discovered bots
			BleUuid foundServiceUuid;                                       //Check the UUID of the advertisement (Unique ID provided to each bot - see if it matches the water bot service)
			size_t svcCount = scanResults[ii].advertisingData().serviceUUID(&foundServiceUuid, 1);      //Get the number of bluetooth services this device has, makes it easier to filter out devices
            if (svcCount > 0 && foundServiceUuid == serviceUuid) {          //If this fevice has services and has a UUID that matches the water bot service, look at it's custom data to see what bot ID it is
                uint8_t BLECustomData[CUSTOM_DATA_LEN];                     //Array to hold the bytes of custom data
                scanResults[ii].advertisingData().customData(BLECustomData,CUSTOM_DATA_LEN);    //Retrieve the custom data from the advertisement
                if(BotNumber == -2){                                        //If -2 was fed as an argument to the function, we are just scanning for nearby bots and looking for the nearest one
                    #ifdef VERBOSE
                    Serial.printlnf("Found Bot #: %d %d %d %d %d %d %d %d, services: %d",BLECustomData[0],BLECustomData[1],BLECustomData[2],BLECustomData[3],BLECustomData[4],BLECustomData[5],BLECustomData[6],BLECustomData[7], svcCount);
                    #endif
                    bool newBot = true;                                     //Check if we've scanned this one before, if not add it to the pair list
                    PairBot *existingBot;
                    for(PairBot p: BLEPair){                                //Create a new pair item if we haven't seen this bot before
                        if(BLECustomData[0] == p.botNum){
                            newBot = false;
                            existingBot = &p;
                            
                        } 
                    }
                    if(newBot){
                        
                        PairBot NewBot;
                        NewBot.rssi = scanResults[ii].rssi();               //Store the signal strength of this bot relative to the CChub to determine which one in the list is the closest
                        NewBot.botNum = BLECustomData[0];                   //Copy in the Bot ID to keep track of who is closest 
                        BLEPair.push_back(NewBot);                          //add to the pairing vector
                        #ifdef VERBOSE
                        Serial.printlnf("Found new bot: %d", BLECustomData[0],BLEPair.size());
                        #endif
                    }
                    else{
                        existingBot->rssi = (scanResults[ii].rssi() + existingBot->rssi) >> 1;      //Update the signal strength if we've seen this one before
                    }
                }
                if(BotNumber == -1 || BotNumber == BLECustomData[0]){   //Check if a particular bot number was specified, or -1 if we should automatically connect to the closest one
                    peer = BLE.connect(scanResults[ii].address());      //Try to connect to the one we just scanned in
				    if (peer.connected()) {                             //If we connected properly
                        startConnect = true;                            //Indicate to the setup function (that may be waiting) that we have discovered a bot over BLE
                        uint8_t bufName[BLE_MAX_ADV_DATA_LEN];          //Buffer to hold the advertising data for retrieving the Bot ID of the scanned bot
                        scanResults[ii].advertisingData().customData(bufName, BLE_MAX_ADV_DATA_LEN);        //Get the custom data and put it in the buffer
					    peer.getCharacteristicByUUID(peerTxCharacteristic, txUuid);             //Get all of the service characteristics - tx is the characteristic the bot transmits data from 
					    peer.getCharacteristicByUUID(peerRxCharacteristic, rxUuid);             //Characteristic the bot received data in on
                        peer.getCharacteristicByUUID(peerOffloadCharacteristic, offldUuid);     //Characteristic the bot sends strings from the SD card on
                        #ifdef VERBOSE
						Serial.printlnf("Connected to Bot %d",bufName[0]);                      //Log that we have connected to a particular bot over BLE
                        #endif
                        bool newBot = true;                             //Flag to check if we have seen this bot before and should add to the main WaterBots vector
                        WaterBot newWaterbot;                           //Create a new water bot object with the bot id seen in the advertisement packet
                        newWaterbot.BLEAvail = true;                    //Assuming BLE is available since we just connected to it
                        newWaterbot.botNum = bufName[0];                //Copy bot ID
                        PairBots.push_back(newWaterbot);                //Push into the pairing vector
                        for(WaterBot &w: WaterBots){                    //Loop over the water bots already discovered
                            if(bufName[0] == w.botNum){                 //Check if this water bot already has been discovered by some other means (XBee, LTE), don't add another copy if we already have one
                                newBot = false;                         //Don't add a new bot
                                w.BLEAvail = true;                      //Indicate in the existing bot that BLE is available
                                BLEBotNum = w.botNum;
                            }
                        }
                        if(newBot){                                     //If we need to create a new bot, then do so. Copy in the bot ID
                            #ifdef VERBOSE
                            Serial.println("Found a new water bot ID");
                            #endif
                            WaterBots.push_back(newWaterbot);
                            BLEBotNum = newWaterbot.botNum;
                            redrawMenu = true;                          //Indicate to menu function to redraw so the bot pops up in the list of discovered bots
                        }
                    }
                    break;
                }
			}
		}
	}
}

/**
 * @brief Copy SD card data from a water bot to the CCHub via BLE
 * @param bot_id The ID of the water bot to offload data from
 * 
 * Initiates a data offloading session to transfer all .txt and .csv files
 * from the specified water bot's SD card to the CCHub's SD card via BLE.
 * Handles BLE connection management, switching between bots if necessary,
 * and provides user feedback through display pop-ups. The function will
 * timeout after 15 seconds if unable to establish BLE connection.
 */
void DataOffloader(uint8_t bot_id){         //Function to copy all .txt and .csv files from the bot SD card and place them on the CCHub SD card
    uint8_t OffloadingBot = bot_id;         //Get the ID of the bot we want to offload from
    if (!logDir.open("/")) {                //Open the root directory of the SD card on the controller here
        offloadingDone = true;              //If we couldn't open the root directory, then break from the offload operation
        MenuPopUp m;                        //Create a pop-up to warn the user that the SD card is not functional (or at least cannot access root)
        sprintf(m.primaryLine,"Warning"); //Print warning strings to the po-up item
        sprintf(m.secondaryLine,"CCHub");
        sprintf(m.tertiaryLine, "SD Card Failed");
        m.primaryStart = 20;                //Calculated offsets to center the text in the warning box
        m.secondaryStart = 60;
        m.tertiaryStart = 20;
        PopUps.push_back(m);                //Push the warning onto the pop-up queue so it is displayed on top
        redrawMenu = true;                  //Indicate to the main loop draw function that there is new data to be displayed
        #ifdef VERBOSE
        Serial.println("Error, could not open root directory on SD Card. Is it inserted?");
        #endif
        return;                             //Return as we can do nothing more without the SD card
    }
    uint32_t startScanTime = millis();      //Capture the current time to make sure we don't get stuck scanning forever, time out if we can't connect to the target over BLE
    #ifdef VERBOSE
    Serial.printlnf("Requested SD Card Data from Bot %d Over BLE\n",BLEBotNum);
    #endif
    if(BLEBotNum != bot_id){                //In the case of multiple bots, it's possible to not be connected to the target over BLE. Disconnect from the current guy in that case and scan for the target guy
        #ifdef VERBOSE
        Serial.printlnf("Currently connected to Bot %s, need to connect to Bot %d",BLEBotNum,OffloadingBot);
        #endif
        offloadingDone = true;              
        MenuPopUp m;                        //Inidicate with a pop-up to the user that there will be a delay, as we need to switch which bot is connected over BLE
        sprintf(m.primaryLine,"Info");    //Print warning string to the pop-up
        sprintf(m.secondaryLine,"Not connected to BLE");
        sprintf(m.tertiaryLine, "Switching BLE conn");
        m.primaryStart = 30;                //Offsets for centering text in the pop-up box
        m.secondaryStart = 5;
        m.tertiaryStart = 10;
        PopUps.push_back(m);                //Push onto the pop-up queue
        redrawMenu = true;
        BLE.disconnect(peer);               //Disconnect from the currently connected guy
        while(!BLE.connected() && millis() - startScanTime < 15000){        //Try to connect to the target bot for offloading, but time out after 15 seconds. Probably out further than the bluetooth range or the bot has been powered off
            BLEScan(bot_id);                //Scan for that particular bot ID
            delay(50);                      //Short pause between scans
        }
        
        char OffloadCommand[10];            //String to hold commands needed for orchestrating offloading operation
        snprintf(OffloadCommand,10,"CCB%ddmp",OffloadingBot);       //Create start command for offloading
        sendData(OffloadCommand,0,true,false,false);                //Send it to the now-connected bot over BLE to have it enter offloading mode
        #ifdef VERBOSE
        if(BLE.connected()) Serial.printlnf("Successfully connected to Bot %d", BLEBotNum);
        #endif
    }   
    else{                                   //We're already connected to the target, so we can just generate the offload command string and send it
        char OffloadCommand[10];
        snprintf(OffloadCommand,10,"CCB%ddmp",OffloadingBot); 
        sendData(OffloadCommand,0,true,false,false);        //Send the offloading command to the bot to start the process
    }
    #ifdef VERBOSE
    Serial.printlnf("Starting file transfer from Bot %d",BLEBotNum);
    #endif
    if(millis() - startScanTime > 15000){   //Check the timeout for if we actually connected to the target, if we didn't connect then cancel, as we can't reach the target
        if(logDir.isOpen()) logDir.close(); //Close the root directory if it's open as cleanup
        return;
    }
    if(BLE.connected()) offloadingDone = false; //Set flag to show that offloading is not complete. Will sit in a loop until the target indicates that it's done copying. Everything else handled in the offload received interrupt
    //offloadingDone = false;
    while(!offloadingDone) delay(100);          //Wait for the bot to indicate it's done with the copy
    #ifdef VERBOSE
    Serial.printlnf("Finished transferring file from Bot %d",BLEBotNum);
    #endif
    if(logDir.isOpen()) logDir.close();         //Release the root directory of the SD card
    offloadingMode = false;                     //Set flag to false to show we're done offloading (at least for this bot)5
}

/**
 * @brief Send status updates from water bots to Raspberry Pi
 * 
 * Monitors all discovered water bots for status updates and formats
 * comprehensive status information for transmission to the Raspberry Pi.
 * Creates bit-masked status flags containing communication availability,
 * operational modes, battery status, and sensor functionality. Clears
 * the update flag for each bot after processing to prevent redundant
 * transmissions.
 */
void RPiStatusUpdate(){                         //Function to check if any water bots have received a status update and pusblish an update to the Raspberry Pi so the website is updated
    for(WaterBot &wb: WaterBots){               //Loop over discovered water bots
        if(wb.updatedStatus){                   //Check if their internal flag for a new status update has been set
            uint16_t statusFlags;               //Bit-mask integer for data compression
            statusFlags = 0;                    //Clear the flag
            statusFlags = wb.LTEAvail;          //Do math to put flags in their corresponding positions in the bitmask (i.e. LTE is bit 0, XBee availability is bit 1, etc.)
            statusFlags |= wb.XBeeAvail << 1;
            statusFlags |= wb.BLEAvail << 2;
            statusFlags |= wb.offloading << 3;
            statusFlags |= wb.driveMode << 4;
            statusFlags |= wb.lowBatt << 6;
            statusFlags |= wb.dataRecording << 7;
            statusFlags |= wb.GPSAvail << 8;
            statusFlags |= wb.CompassAvail << 9;
            statusFlags |= wb.SDAvail << 10;
            //Serial.printlnf("CCRPsupB%d %d %0.6f %0.6f %d %d %d",wb.botNum, wb.battPercent, wb.GPSLat, wb.GPSLon, statusFlags,wb.battPower, wb.panelPower); //Print out status string to the Raspberry Pi over the USB port
            wb.updatedStatus = false;           //Indicate this bot is up to date, until the next status update is received
        }
    }
}

/**
 * @brief Handle incoming commands from Raspberry Pi via USB serial
 * 
 * Continuously monitors the USB serial buffer for incoming commands from the
 * Raspberry Pi control system. Reads complete lines terminated by newline
 * characters, processes them through the RPi command dictionary, and logs
 * all received messages to the SD card for debugging purposes.
 */
void RPiHandler(){                                          //Function to check if any strings have been received from the Raspberry Pi, if so, then process them using the dictionary
    while(Serial.available()){                              //Check if the serial buffer has anythingin it
            String data = Serial.readStringUntil('\n');     //Read until the newline character, which is at the end of every transmission
            #ifdef VERBOSE
            Serial.println(data);
            #endif
            char buffer[data.length() + 2];                 //Create a buffer for the data and give space for null character
            data.toCharArray(buffer,data.length()+1);       //Copy the characters from the received string type to a character buffer for feeding into command processor
            buffer[data.length() + 1] = 0;                  //Put null character so wild memory stuff does not happen (I've never seen emoji's printed to a console before this bug).
            //if(data.length() > 1 && data.charAt(data.length()-1) == '\r') buffer[data.length()-1] = 0;
            //if(data.length() > 1 && data.charAt(data.length()-1) == '\n') buffer[data.length()-1] = 0;
            processRPiCommand(buffer,3);                    //Process the command received from the Pi
            if(logMessages){
                if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
                logFile.printlnf("[INFO] Received Raspberry Pi Message: %s",data.c_str());
                logFile.close();
        }
    }
}

/**
 * @brief Handle incoming messages from XBee radio communication
 * 
 * Processes messages received from water bots via XBee radio communication.
 * Reads from Serial1 buffer, parses commands, and routes them to appropriate
 * command handlers. Logs all received messages for debugging.
 */
void XBeeHandler(){                                         //Function similar to the Pi handler, but reads from the serial buffer attached to the XBee radio
    while(Serial1.available()){                             //Check if a string has been received from XBee
        String data = Serial1.readStringUntil('\n');        //Read up to end of the string which always hass null terminator character
        char buffer[data.length()];                         //Create a buffer for copying data from the string data type
        for(uint16_t i = 0 ; i < data.length(); i++) buffer[i] = data.charAt(i);        //Copy over the characters from the string type
        if(data.length() > 1 && data.charAt(data.length()-1) == '\r') buffer[data.length()-1] = 0;      //Set the last character to a null to allow sting processing functions to work
        #ifdef VERBOSE
        Serial.println("New XBee Command:");
        Serial.println(data); 
        #endif
        processCommand(buffer,2,true);                      //Process the command using the dictionary
        if(logMessages){
            if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
            logFile.printlnf("[INFO] Received XBee Message: %s",data.c_str());
            logFile.close();
        }
    }
}

/**
 * @brief Read joystick input and send manual motor control commands
 * @param commandedBot Bot number to send motor commands to
 * 
 * Reads analog joystick inputs, converts them to differential motor speeds
 * for tank-style steering, and transmits motor control commands to the
 * specified water bot. Includes deadband filtering and change detection.
 */
void manualMotorControl(uint8_t commandedBot){              //Function to read the joystick on the CCHub, convert to left and right motor speeds, and transmit out the motor command
    static uint8_t lastLSpeed;                              //Variable to hold the last speed transmitted out for each motor, used to check if there is a significant change from the last sample
    static uint8_t lastRSpeed;

    char mtrStr[18];                                        //Temporary string to hold the motor command
    int VRead, HRead, VSet, HSet;                           //Variables used to read the horizontal and vertical joystick and convert to fwd/backward values
    VRead = 4095-analogRead(JOYV_ADC);                      //Read the vertical joystick, invert the direction
    HRead = analogRead(JOYH_ADC);                           //Read horizontal joystick
    if(VRead < JOY_MID - JOY_DEADZONE){                     //Implement a deadzone so there is no response when the stick is only barely bumped
        VSet = -90 * (VRead - (JOY_MID - JOY_DEADZONE))/(JOY_MIN - (JOY_MID - JOY_DEADZONE));   //Calculate a value between 0 and -90 when going down
        if(VSet < -90) VSet = -90;                                                              //Place caps at lower end of the joystick
    }
    else if(VRead > JOY_MID + JOY_DEADZONE){
        VSet = 90 * (VRead - (JOY_MID + JOY_DEADZONE))/(JOY_MAX - (JOY_MID + JOY_DEADZONE));    //Calculate a value between 0 and 90 when going up
        if(VSet > 90) VSet = 90;                                                                //Place caps at upper end of joystick
    }
    else{           //If we're in the deadzone, set to 0 (idle)
        VSet = 0;
    }
    if(HRead < JOY_MID - JOY_DEADZONE){                    //Implement the same deadzoning and ramping as the vertical joystick on the horizontal joystick
        HSet = -90 * (HRead - (JOY_MID - JOY_DEADZONE))/(JOY_MIN - (JOY_MID - JOY_DEADZONE));
        if(HSet < -90) HSet = -90;
    }
    else if(HRead > JOY_MID + JOY_DEADZONE){
        HSet = 90 * (HRead - (JOY_MID + JOY_DEADZONE))/(JOY_MAX - (JOY_MID + JOY_DEADZONE));
        if(HSet > 90) HSet = 90;
    }
    else{
        HSet = 0;
    }
    
    LSpeed = 90 + VSet/2;
    if(VSet > 0){                   //Determine quandrants for doing math and affecting the motor behavior
        if(HSet > 0){               //Top right quadrant
            if(HSet > VSet){        //Below the y = x line on the graph //H = 90, V = 0 -> LS = 135, RS = 45, H = 90, V = 90 -> LS = 180, RS = 135
                LSpeed = 90 + HSet/2 + VSet/2;
                RSpeed = 90 - HSet/2 + VSet;
            }
            else{                   //Above the y = x line on the graph
                LSpeed = 90 + VSet;
                RSpeed = 90 - HSet/2 + VSet;
            }
        }
        else{                       //Top left quadrant
            if((0-HSet) > VSet){    //Below the y = -x line on the graph
                RSpeed = 90 - HSet/2 + VSet/2;
                LSpeed = 90 + HSet/2 + VSet;
            }
            else{                   //Above the y = -x line on the graph
                RSpeed = 90 + VSet;
                LSpeed = 90 + HSet/2 + VSet;
            }
        }
    }
    else{                           //In the lower half of the graph
        if(HSet > 0){               //In the bottom right quadrant
            if(HSet > (0-VSet)){        // H = 90, V = 0 -> LS = 135, RS = 45, H = 90, V = -90 -> LS = 0; RS = 45
                LSpeed = (90 + HSet/2 + VSet*1.5 );
                RSpeed = (90 - HSet/2);      
            }
            else{                  
                LSpeed = 90 + VSet;
                RSpeed = 90 + HSet/2 + VSet;
            }
        }
        else{                        //In the bottom left quadrant
            if((0-HSet) > (0-VSet)){    //H = -90, V = 0 -> LS = 45, RS = 135, H = -90, V = -90 -> LS = 45, RS = 0
                RSpeed = 90 - HSet/2 + (VSet*1.5);
                LSpeed = 90 + HSet/2;
            }
            else{
                RSpeed = 90 + VSet;
                LSpeed = 90 - HSet/2 + VSet;
            }
        }
    }

    //The checks below determine if there was a significant change in the joystick position since the last sample. If there was, then publish out the data a little more frequently
    if(lastLSpeed - LSpeed > LTE_MIN_DIFF || LSpeed - lastLSpeed > LTE_MIN_DIFF){
        lastLSpeed = LSpeed;
        ctlSpeedDiff = true;        //Set this flag true if data should be sent out sooner - due to a large difference since the last position
    }
    if(lastRSpeed - RSpeed > LTE_MIN_DIFF || RSpeed - lastRSpeed > LTE_MIN_DIFF){
        lastRSpeed = RSpeed;
        ctlSpeedDiff = true;
    }
    if(lastLSpeed != LSpeed && LSpeed == 90){
        lastLSpeed = LSpeed;
        ctlSpeedDiff = true;
    }
    if(lastRSpeed != RSpeed && RSpeed == 90){
        lastRSpeed = RSpeed;
        ctlSpeedDiff = true;
    }

    //Check if there is a stop active or if it's been too recent since the last publish, if so then do nothing, otherwise publish out over available comm method
    if(!stopActive && ((millis() - rcTime > MTR_UPDATE_TIME) || (ctlSpeedDiff && millis() - rcTime > 75))){
        ctlSpeedDiff = false;
        rcTime = millis();                      //Capture last time a motor command was sent to limit traffic
        for(WaterBot wb: WaterBots){            //Loop over water bots in vector and check if this one is selected and being controlled
            if(wb.driveMode == 0 && wb.botNum == botSelect){
                snprintf(mtrStr, 16, "CCB%dmtr%03d%03d",commandedBot, LSpeed, RSpeed);        //Craft the command string to be transmitted out
                bool sendMTRLTE = false;                                                //Flag for limiting publish rate over LTE
                if(!wb.XBeeAvail && !wb.BLEAvail && ctlSpeedDiff && (millis() - wb.LastMtrTime > MTR_LTE_PERIOD)){      //Check if XBee or BLE are available before trying to publish over LTE
                    wb.LastMtrTime = millis();  //Update timer for LTE publishes
                    sendMTRLTE = true;
                }
                sendData(mtrStr,0,!(wb.XBeeAvail), true, sendMTRLTE);       //Send data with priority of XBee, then Bluetooth, then LTE
                //Serial.printlnf("Time :%d, Speed: %d %d", millis(), LSpeed, RSpeed);
            }
        }
    }
}

/**
 * @brief Initiate compass calibration for water bots
 * 
 * Sends compass calibration commands to water bots that have requested
 * calibration. Creates informational pop-ups to guide the user through
 * the calibration process and communicates with the bot's compass system.
 */
void calibrateCompass(){
    for(WaterBot &wb: WaterBots){            //Loop over water bots in vector and check if this one is selected and being controlled
        if(wb.requestCompCalibration){
            wb.requestCompCalibration = false;          //Set the flag to false so we don't keep sending this command over and over again
            MenuPopUp m;                                //Create a pop-up for the low battery warning
            sprintf(m.primaryLine,"Compass");         //Populate strings of the compass calibration mode with the bot number
            sprintf(m.secondaryLine,"Turn B%d facing North", wb.botNum);
            sprintf(m.tertiaryLine, "Press OK to calibrate");
            m.primaryStart = 20;                        //Calculated offsets so the strings are centered in the box - determined from experimentation
            m.secondaryStart = 5;
            m.tertiaryStart = 5;
            PopUps.push_back(m);                        //Push the pop-up item onto the vector so the menu updater prints it
            redrawMenu = true;                          //Set flag so the menu item updater draws it in
            wb.calRequestAcknowledged = true;           //Indicate that the pop-up has been shown to the user and they can acknowledge it
        }

        //If the user has requested calibration, after they clear the pop-up, send the command to the bot to start calibration
        if(wb.calRequestAcknowledged && PopUps.empty()){                //Check if the user has acknowledged the calibration request, if so then send the command to the bot
            wb.calRequestAcknowledged = false;          //Reset the flag to false so we don't keep sending this command over and over again
            char calStr[10];                            //String to hold the calibration command
            sprintf(calStr,"CCB%dcmp",wb.botNum);       //Create the command string to send out
            sendData(calStr,0,(!wb.XBeeAvail),true,(!wb.BLEAvail && !wb.XBeeAvail));        //Send the command out over BLE to the bot
            #ifdef VERBOSE
            Serial.printlnf("Sending compass calibration command to Bot %d",wb.botNum);
            #endif
        }
    }
}

/**
 * @brief Send motor tare commands to water bots with updated settings
 * 
 * Monitors all discovered water bots for changes in motor tare values
 * set through the menu interface. When a change is detected, formats
 * and transmits the new motor tare command to the appropriate bot via
 * the best available communication method (prioritizing XBee/BLE over LTE).
 * Updates the last sent tare value to prevent redundant transmissions.
 */
void sendTareData(){
    for(WaterBot &wb: WaterBots){
        if(wb.motorTare != wb.lastMotorTare){
            wb.lastMotorTare = wb.motorTare;            //Update the last motor tare value
            char tareStr[15];                           //String to hold the motor tare command
            sprintf(tareStr,"CCB%dtar%03d",wb.botNum,wb.motorTare);   //Create the command string to send out
            sendData(tareStr,0,(!wb.XBeeAvail),true,(!wb.BLEAvail && !wb.XBeeAvail));        //Send the command out to the bot
            #ifdef VERBOSE
            Serial.printlnf("Sending motor tare command to Bot %d, tare: %d",wb.botNum, wb.motorTare);
            #endif
        }
    }
}

/**
 * @brief BLE interrupt handler for incoming command data from water bots
 * @param data Pointer to received byte array data
 * @param len Length of received data in bytes
 * @param peer BLE peer device that sent the data
 * @param context User context pointer (unused)
 * 
 * Interrupt service routine triggered when command data is received from
 * a connected water bot via BLE. Converts the byte array to a null-terminated
 * string, processes the command through the main command dictionary, and
 * logs the message to SD card if logging is enabled.
 */
static void BLEDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    char btBuf[len+1];      //Received as a byte array over BLE, take and convert to a char array for string operations
    for (size_t ii = 0; ii < len; ii++) btBuf[ii] = data[ii];   //Loop and copy each character
    if(btBuf[len-1] != '\0') btBuf[len] = '\0';                 //Put null terminator at end to ensure no memory overflow to surrounding regions (trust me, you'll see funky stuff come from random memory)
    else btBuf[len-1] = '\0';
    #ifdef VERBOSE
    Serial.print("New BT Command: ");
    Serial.println(btBuf);
    #endif
    processCommand(btBuf,1,true);                               //Send command to the dictionary for processing
    if(logMessages){
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[INFO] Received BLE Message: %s",btBuf);
        logFile.close();
    }
}

/**
 * @brief BLE interrupt handler for SD card file offloading operations
 * @param data Pointer to received byte array data containing file data or commands
 * @param len Length of received data in bytes
 * @param peer BLE peer device that sent the data
 * @param context User context pointer (unused)
 * 
 * Interrupt service routine for processing file transfer data during SD card
 * offloading operations. Handles three types of commands: "filename" to create
 * new files, "filecomp" to signal end of current file, and "filedone" to
 * complete the transfer session. Raw data between commands is written directly
 * to the currently open file on the CCHub's SD card.
 */
void offloadDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    char fileCommand[8 + MAX_FILENAME_LEN];
    memset(fileCommand,0,8 + MAX_FILENAME_LEN);
    memcpy(fileCommand,data,8);
    
    if(fileCommand[0] == 'f'){      //Check if we received a file string or if we received a command for making a file or quitting
        //Serial.printlnf("Found an 'f' command %s",fileCommand);
        if(!strcmp(fileCommand,"filename")){        //Filename command creates a new file named with the characters following in the data string
            if(myFile.isOpen()) myFile.close();     //Close the current file
            memcpy(fileCommand,data,8 + MAX_FILENAME_LEN);      //Copy int he new filename from the received data
            memset(offloadFilename,0,MAX_FILENAME_LEN);         //String that contains the properly shifted name
            strncpy(offloadFilename,fileCommand+8,MAX_FILENAME_LEN);
            if(sd.exists(offloadFilename)){         //Try to open a file with the received name
                #ifdef VERBOSE
                Serial.printlnf("File '%s' already exists, deleting and overwriting",offloadFilename);
                #endif
                sd.remove(offloadFilename);
            }
            #ifdef VERBOSE
            Serial.printlnf("Starting offload of file: %s",offloadFilename);
            #endif
            myFile.open(offloadFilename, O_RDWR | O_CREAT | O_AT_END);
            return;
        }
        else if(!strcmp(fileCommand,"filecomp")){       //Command to indicate that we have reached the end of the currently copyin file
            #ifdef VERBOSE
            Serial.printlnf("Reached end of file: %s",offloadFilename);
            #endif
            if(myFile.isOpen()) myFile.close();
            return;
        }
        else if(!strcmp(fileCommand,"filedone")){       //Command to indicate that we are done copying all files
            #ifdef VERBOSE
            Serial.println("Received done command");
            #endif
            offloadingDone = true;                      //Set flag to break the loop in the offloading function
            if(myFile.isOpen()) myFile.close();
            return;
        }
    }
    //If not a command, then we have received raw data from the SD card
    char dataStr[len];      
    memcpy(dataStr,data,len);       //Copy that data from the byte array
    myFile.print(dataStr);          //Print it to the file that is now on the SD card here
    #ifdef VERBOSE
    Serial.println(dataStr);
    #endif
}

/**
 * @brief Send data over multiple communication channels
 * @param dataOut Data string to transmit
 * @param sendMode Communication mode (1=BLE, 2=XBee, 4=LTE)
 * @param sendBLE Flag to enable BLE transmission
 * @param sendXBee Flag to enable XBee transmission  
 * @param sendLTE Flag to enable LTE transmission
 * 
 * Main communication function that sends data over BLE, XBee, and/or LTE
 * channels. Automatically appends checksum and handles channel-specific
 * formatting requirements.
 */
void sendData(const char *dataOut, uint8_t sendMode, bool sendBLE, bool sendXBee, bool sendLTE){
    char outStr[strlen(dataOut)+2];                     //Get the length of the inputted string and extend it by 2 for the checksum characters
    sprintf(outStr,"%s%02x",dataOut,strlen(dataOut));   //Copy the characters into the string along with the checksum
    #ifdef VERBOSE
    Serial.println(outStr);                             
    #endif
    if(sendLTE || sendMode == 4){                       //Check to send out over LTE
        Particle.publish("CCHub", outStr, PRIVATE);     //Publish event to send out over LTE, the bot is subscribed to the "CCHub" event
        sendLTE = false;                            
    }
    if((sendBLE || sendMode == 1) && BLE.connected()){  //Check to send out over BLE, only do so if BLE is connected
        uint8_t txBuf_tmp[strlen(outStr)];              //BLE requires a uint8_t (byte) array instead of chars, so copy to a byte array
        memcpy(txBuf_tmp,outStr,strlen(outStr));        //Use memcpy to make it ez
        peerRxCharacteristic.setValue(txBuf_tmp, strlen(outStr));   //Set the value for the tx characteristic to write to the bot
    }
    if(sendXBee || sendMode == 2){                      //Check to send out over XBee
        Serial1.println(outStr);                        //Print string on serial1 for XBee transmission
    }
}

/**
 * @brief Timer callback function for periodic status operations
 * 
 * Called periodically by a timer to trigger status updates and maintain
 * communication with water bots. Sets flags for status posting and
 * increments request activity counters for all bots.
 */
void actionTimer5(){
    postStatus = true;
    for(WaterBot &w: WaterBots){
        w.reqActive++;
    }
    //if(!BLE.connected)
}

/**
 * @brief Simulate water bots for testing menu functionality
 * @param count Number of simulated bots to create
 * 
 * Creates simulated water bot objects for testing the menu interface
 * without requiring actual physical bots. Useful for development and
 * debugging of the user interface components.
 */
void WaterBotSim(uint8_t count){        //Function to simulate bot squares on the menu and test how the menu functions with multiple bots. count indicates the number to simulate
    if(count + WaterBots.size() > 10) count = 10-WaterBots.size();      //If we have existing bots already, then only simulate so many to fill up to 10 in the vector
    uint8_t botloop = count+WaterBots.size();                           //Loop over and check for duplicates
    for(uint8_t temp = 0; temp < botloop; temp++){
        int dupeBot = false;
        for(WaterBot wb: WaterBots){
            if(wb.botNum == temp) dupeBot = true;
        }
        if(dupeBot) continue;                                           //If we find a real bot ID that equals the simulated one, then skip this id
        WaterBot simBot;                                                //Otherwise, create a simulated bot to go in the main vector
        simBot.driveMode = 0;                                           //Assume remote control to see behavior of switching between bots
        simBot.botNum = temp;                                           //Assing sequential bot ID from the simulator
        simBot.BLEAvail = false;                                        //Assume BLE is not available
        simBot.XBeeAvail = true;                                        //Assume XBee is available
        simBot.LTEAvail = false;
        simBot.SDAvail = true;
        simBot.battPercent = random(100);                               //Create a random battery percentage, to see if the low battery pop up happens
        WaterBots.push_back(simBot);                                    //Add the simulated bot to the main vector
    }

}

/**
 * @brief Create and initialize all menu items for the OLED interface
 * 
 * Initializes all menu items that appear in the OLED menu system including
 * bot control parameters, status displays, and configuration options.
 * Sets up method pointers to link menu items to WaterBot class variables.
 */
void createMenu(){
    MenuItem dataRecord;                                    //Data recording item, used to control if sensors should be recording to SD card
    dataRecord.init(1,0,1,true,"Record");                   //Label printed to oled is "Record" with "On" "Off" labels
    dataRecord.MethodPointerBool = &WaterBot::dataRecording;    //Modifies the dataRecording variable selected waterbot in the waterbot class when the left or right buttons are pressed

    MenuItem battStat;                                      //Statistic menu item to display the estimated battery percentage of this bot received from a status update
    battStat.init(1,0,100,false,"Battery");                 //Print out battery label when showing this item, with a max value of 100, displaying as a raw number
    battStat.statOnly = true;                               //Only a statistic, don't allow modification from the left or right buttons
    battStat.MethodPointer = &WaterBot::battPercent;        //Get the data from the battery percentage variable of the selected bot

    MenuItem offloadItem;                                   //Item for choosing to offload data from the SD card
    offloadItem.init(1,0,1,true,"Offload");                 //Print offloading to the menu for this item
    offloadItem.MethodPointerBool = &WaterBot::offloading;  //Modify the offloading variable in the water bot object when pressing left or right button

    MenuItem sentryToggle;                                  //Item for choosing which drive mode this bot is in
    sentryToggle.init(1,0,2,false,"Sentry");                //Print out Sentry when displaying this menu item, with a minimum value of 0 (Remote Control), and a max of 2 (Autonomous)
    sentryToggle.customLabel = true;                        //Using custom string labels so it switches between "Rem" "Sen" and "Aut"
    sentryToggle.statOnly = false;                          //Not a statistic only, modifies the drive mode
    sentryToggle.labels.push_back("Rem");                   //Copy in the custom data labels to display in the data section
    sentryToggle.labels.push_back("Sen");
    sentryToggle.labels.push_back("Aut");
    sentryToggle.MethodPointer = &WaterBot::driveMode;      //Modify the driveMode variable in the wateer bot object for the selected bot when pressing left or right button

    MenuItem signalToggle;                                  //Item for signaling the onboard LED of the waterbot
    signalToggle.init(1,0,1,true,"Signal");                 //Print signal when displaying this menu item, with "On" "Off" labels
    signalToggle.statOnly = false;                          //This is a modifiable field
    signalToggle.MethodPointerBool = &WaterBot::signal;     //Modify the signal variable in the water bot object for the selected bot when pressing the left or right object

    MenuItem solStat;                                       //Create item for the solar power statistic
    solStat.init(1,0,999,false,"SolPwr");                   //Maximum value of 999 watts
    solStat.statOnly = true;                                //Only a statistic, not modifiable
    solStat.MethodPointer = &WaterBot::panelPower;          //Source data from the panelPower variable for the selected bot

    MenuItem battPwr;                                       //Create item for the battery power statistic
    battPwr.init(1,0,999,false,"BatPwr");                   //Maximum of 999 watts
    battPwr.statOnly = true;                                //Only a statistic, not modifiable
    battPwr.MethodPointer = &WaterBot::battPower;           //Source data from the battPower variable for the selected bot

    MenuItem motorTareItem;                                 //Create item for the motor tare value
    motorTareItem.init(1,0,200,false,"MtrTare");             //Range 0-200
    motorTareItem.statOnly = false;                         //This is a modifiable field
    motorTareItem.MethodPointer = &WaterBot::motorTare;     //Source data from the motorTare variable for the selected bot

    MenuItem compHead;                                       //Create item for the compass heading
    compHead.init(1,0,359,false,"Compass");                  //Range 0-359
    compHead.statOnly = true;                                //Only a statistic, not modifiable
    compHead.MethodPointer = &WaterBot::CompassHeading;      //Source data from the battPower variable for the selected bot

    MenuItem compassCal;                                                //Create item for the compass calibration mode
    compassCal.init(1,0,1,true,"CalCmp?");                              //True or false
    compassCal.statOnly = false;                                        //This is a modifiable field
    compassCal.MethodPointerBool = &WaterBot::requestCompCalibration;   //Source data from the battPower variable for the selected bot

    MenuItems.push_back(dataRecord);                        //Push all the generated menu items into the menu items vector and they will be automatically printed by the menu function
    MenuItems.push_back(battStat);
    MenuItems.push_back(sentryToggle);
    MenuItems.push_back(offloadItem);
    MenuItems.push_back(signalToggle);
    MenuItems.push_back(solStat);
    MenuItems.push_back(battPwr);
    MenuItems.push_back(motorTareItem);
    MenuItems.push_back(compHead);
    MenuItems.push_back(compassCal);

    SelectedItem = &MenuItems.at(menuItem);                 //Select the first menu item
}

/**
 * @brief Interrupt handler for the Enter button press
 * 
 * Handles Enter button presses for menu navigation. Toggles between
 * bot selection and menu item value modification modes. Also dismisses
 * pop-up messages when present. Includes debouncing logic.
 */
void entHandler(){
    redrawMenu = true;                  //Always redraw to fix any bugs of not updating the screen when modifying values
    if(millis()-debounceTime < DEBOUNCE_MS) return;     //Debounce the button to ensure only one trigger per press
    #ifdef VERBOSE  
    Serial.println("Enter trigger");
    #endif
    debounceTime = millis();                            //Reset debounce timer
    if(PopUps.size() != 0){                             //Dismiss a pop up if there is one
        PopUps.pop_back();
        return;
    }
    selectingBots = !selectingBots;                     //Switch between selecting bots or selecting menu items
    if(modifiedValue){                                  //Check if the left or right buttons were pressed to modify menu values, if so then signal to send out status updates immediately
        updateControl = true;
        modifiedValue = false;
    }
}

/**
 * @brief Interrupt handler for the Right button press
 * 
 * Handles Right button presses for navigation. Used to switch between bots
 * when in bot selection mode, or to increment/modify menu item values
 * when in value modification mode. Includes debouncing logic.
 */
void rHandler(){
    redrawMenu = true;                                          //Set redraw flag always so the display is updated with new highlighted item
    if(millis()-debounceTime < DEBOUNCE_MS) return;             //debounce this button, to make sure only one trigger is registered per press
    debounceTime = millis();    
    #ifdef VERBOSE
    Serial.println("Right trigger");
    #endif
    if(selectingBots){                                          //If enter was hit over a menu item, the item will be open for modification, and this flag indicates true when not modifying items (i.e. selecting which bot to access)
        if(botSelect != WaterBots.back().botNum){               //Check if we are not at the leftmost bot in the list, otherwise we shouldn't try selecting a bot that doesn't exist
            bool findCurrent = false;                           //Grab the first item in the waterbot list to make sure we don't accidentally access something null
            for(WaterBot &ws: WaterBots){                       //Loop over the list of bots discovered
                if(findCurrent){                                //Funky algorithm to find the bot next to the current bot in the list
                    botSelect = ws.botNum;
                    ControlledBot = &ws;
                    break;
                }
                if(ws.botNum == botSelect) findCurrent = true;
            }
        }
    }
    else{                                                       //If enter was hit over a menu item, the item will be open for modification, and this flag indicates false when modifying items (i.e. editing the value of a menu item)
        for(WaterBot &ws: WaterBots){                           //Loop over all bots in the list of discovered bots
            if(ws.botNum == botSelect){                         //If we find the one we're looking for in the list
                MenuItem *curItem = SelectedItem;               //Take the current selected menu item
                #ifdef VERBOSE
                Serial.println(curItem->itemName);
                #endif
                if(curItem == nullptr) return;                  //Make sure that there is a menu item currently selected
                if(curItem->statOnly) return;                   //Some menu items are a status-only display and can't be modified, so do nothing
                if(curItem->onOffSetting){                      //If this is an on/off setting, then it must be a boolean, so set it true
                    #ifdef VERBOSE
                    Serial.println("Modified an On/Off Control");
                    Serial.printlnf("Bot: %d, Modified ",ws.botNum);
                    #endif
                    ws.*(curItem->MethodPointerBool) = true;
                }
                else{
                    
                    if(ws.*(curItem->MethodPointer) < curItem->maxVal) ws.*(curItem->MethodPointer) += curItem->stepSize; //Increment the counter in the waterbot class that this menu item modifies, using the pointer to a element of the waterbot class (see the menuitem class for details)
                }
                modifiedValue = true;                           //Indicate to the main loop that some bot has a modified a value, so send out a new control packet
                ws.updatedControl = true;                       //Indicate that this bot has a modified value
            }
            //index++;
        }
    }
}

/**
 * @brief Interrupt handler for the Left button press
 * 
 * Handles Left button presses for navigation. Used to switch between bots
 * when in bot selection mode, or to decrement/modify menu item values
 * when in value modification mode. Includes debouncing logic.
 */
void lHandler(){
    redrawMenu = true;                                      //Set redraw flag always so the display is updated with new highlighted item
    if(millis()-debounceTime < DEBOUNCE_MS) return;         //debounce this button, to make sure only one trigger is registered per press
    #ifdef VERBOSE
    Serial.println("Right trigger");
    #endif
    debounceTime = millis();
    if(selectingBots){                                      //If enter was hit over a menu item, the item will be open for modification, and this flag indicates true when not modifying items (i.e. selecting which bot to access)
        if(botSelect != WaterBots.front().botNum){          //Check if we are not at the leftmost bot in the list, otherwise we shouldn't try selecting a bot that doesn't exist
            uint8_t newBotNum = WaterBots.front().botNum;   //Grab the first item in the waterbot list to make sure we don't accidentally access something null
            for(WaterBot &ws: WaterBots){                   //Loop over the list of bots discovered
                if(ws.botNum == botSelect){                 //Funky algorithm to find the bot next to the current bot in the list
                    botSelect = newBotNum;
                    ControlledBot = &ws;
                }
                else newBotNum = ws.botNum;
            }
               
        }
    }
    else{                                                   //If enter was hit over a menu item, the item will be open for modification, and this flag indicates false when modifying items (i.e. editing the value of a menu item)
        for(WaterBot &ws: WaterBots){                       //Loop over all bots in the list of discovered bots
            if(ws.botNum == botSelect){                     //If we find the one we're looking for in the list
                MenuItem *curItem = SelectedItem;           //Take the current selected menu item
                #ifdef VERBOSE
                Serial.println(curItem->itemName);
                #endif
                if(curItem == nullptr) return;              //Make sure that there is a menu item currently selected
                if(curItem->statOnly) return;               //Some menu items are a status-only display and can't be modified, so do nothing
                if(curItem->onOffSetting){                  //If this is an on/off setting, then it must be a boolean, so set it false
                    #ifdef VERBOSE
                    Serial.println("Modified an On/Off Control");
                    Serial.printlnf("Bot: %d, Modified ",ws.botNum);
                    #endif
                    ws.*(curItem->MethodPointerBool) = false;   //Modify the boolean in the waterbot class that this menu item modifies, using the pointer to a element of the waterbot class (see the menuitem class for details)
                }
                else{
                    if(ws.*(curItem->MethodPointer) > curItem->minVal) ws.*(curItem->MethodPointer) -= curItem->stepSize;   //Decrement the counter in the waterbot class that this menu item modifies, using the pointer to a element of the waterbot class (see the menuitem class for details)
                }
                modifiedValue = true;                       //Indicate to the main loop that some bot has a modified a value, so send out a new control packet
                ws.updatedControl = true;                   //Indicate that this bot has a modified value
            }
        }
    }
}

/**
 * @brief Interrupt handler for the Up button press
 * 
 * Handles Up button presses for menu navigation. Moves the menu selection
 * cursor up one item in the menu list. Includes debouncing logic and
 * triggers menu redraw.
 */
void uHandler(){
    redrawMenu = true;                                      //Set redraw flag always so the display is updated with new highlighted item
    if(millis()-debounceTime < DEBOUNCE_MS) return;         //debounce this button, to make sure only one trigger is registered per press
    debounceTime = millis();
    if(menuItem) menuItem--;                                //Go up by one menu item by decrementing the counter by one, as long as we are not at the top already
    SelectedItem = &MenuItems.at(menuItem);                 //Update which item is selected so the redraw function can use it
    #ifdef VERBOSE
    Serial.println("Up trigger");
    #endif
}

/**
 * @brief Interrupt handler for the Down button press
 * 
 * Handles Down button presses for menu navigation. Moves the menu selection
 * cursor down one item in the menu list. Includes debouncing logic and
 * triggers menu redraw.
 */
void dHandler(){
    redrawMenu = true;                                      //Set redraw flag always so the display is updated with new highlighted item
    if(millis()-debounceTime < DEBOUNCE_MS) return;         //debounce this button, to make sure only one trigger is registered per press
    debounceTime = millis();
    if(menuItem < MenuItems.size()-1) menuItem++;             //Go down by one menu item by incrementing the counter by one, as long as we are not at the bottom already
    SelectedItem = &MenuItems.at(menuItem);                 //Update which item is selected so the redraw function can use it
    #ifdef VERBOSE
    Serial.println("Down trigger");
    #endif
}

/**
 * @brief Interrupt handler for joystick button press
 * 
 * Handles joystick button press events. Originally intended for capturing
 * current GPS coordinates and entering sentry mode, but currently serves
 * as a placeholder for future joystick click functionality.
 */
void jHandler(){
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    debounceTime = millis();
    #ifdef VERBOSE
    Serial.println("Joystick trigger");
    #endif
}

/**
 * @brief Interrupt handler for emergency stop button
 * 
 * Handles emergency stop button presses to immediately halt all water bot
 * operations. Toggles between stop and clear modes, creates appropriate
 * pop-up notifications, and sets global stop flags for safety.
 */
void sHandler(){    
    if(millis()-debounceTime < DEBOUNCE_MS) return;         //debounce this button, to make sure only one trigger is registered per press
    debounceTime = millis();
    if(stopActive){                                         //Check if the user has initiated a stop, if so, then exit stop mode
        MenuPopUp m;                                        //Create pop-up to indicate to user that we are exiting stop mode
        sprintf(m.primaryLine,"CLEARED");                 //Main display line
        sprintf(m.secondaryLine,"Motors Resuming");         //Secondary display line
        sprintf(m.tertiaryLine, "Press again to stop");     //Tertiary display line
        m.primaryStart = 20;                                //Horizontal pixel that the main line starts at
        m.secondaryStart = 20;                              //Horizontal pixel that the secondary line starts at
        m.tertiaryStart = 7;                                //Horizontal pixel that the tertiary line starts at
        PopUps.push_back(m);                                //Push the warning item onto the stack of pop-ups
        redrawMenu = true;                                  //Set flag for redraw so main loop will display pop-up
        stopActive = false;                                 //Clear global flag so main loop will stop sending the stop command (can't do that here because it's an ISR)
        LTEStopSent = false;                                //Every time there is a stop, use LTE one time
    }
    else{                                                   //If not in stop mode, initiate stop mode
        MenuPopUp m;                                        //Create pop-up to indicate to user that we are entering stop mode
        sprintf(m.primaryLine,"STOPPED");                 //Main display line
        sprintf(m.secondaryLine,"Motors Stopped!");         //Secondary display line
        sprintf(m.tertiaryLine, "Press again to start");    //Tertiary display line
        m.primaryStart = 20;                                //Horizontal pixel that the main line starts at
        m.secondaryStart = 20;                              //Horizontal pixel that the secondary line starts at
        m.tertiaryStart = 5;                                //Horizontal pixel that the tertiary line starts at
        PopUps.push_back(m);                                //Set flag for redraw so main loop will display pop-up
        redrawMenu = true;                                  //Set global flag so main loop will send the stop command (can't do that here because it's an ISR)
        stopActive = true;                                  //Set global flag so main loop will send the stop command (can't do that here because it's an ISR)
    }
}

/**
 * @brief Process LTE commands received as Particle cloud functions
 * @param cmd Command string received from Particle cloud
 * @return Always returns 1 to indicate successful processing
 * 
 * Converts String commands to char array and processes them through
 * the standard command processing pipeline. Used for remote LTE control.
 */
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

/**
 * @brief Print a help menu showing available commands and their descriptions
 * 
 * Displays a comprehensive help menu over serial communication listing
 * all available commands, their syntax, and brief descriptions for
 * debugging and operator reference.
 */
void printHelpMenu(){
    Serial.println();
    Serial.println("==== CCHub Command Help ====");
    Serial.println("Bot -> Hub commands (from WaterBots):");
    Serial.println("  sup  - Status update (telemetry/state)");
    Serial.println("  sns  - Sensor reading packet");
    Serial.println("  hwd  - Hello World / handshake");
    Serial.println("  pts  - Put string (debug/message)");
    Serial.println("  ldt  - Leak detected (shutoff)");
    Serial.println("  ldb  - Battery leak detected (shutoff)");
    Serial.println("  wld  - Leak warning (no shutoff)");
    Serial.println("  wlb  - Battery leak warning (no shutoff)");
    Serial.println();
    Serial.println("RPi -> Hub commands (from Raspberry Pi/USB):");
    Serial.println("  cbp  - Toggle checksum bypass for RPi commands");
    Serial.println("  ctl  - Apply control: <ID> <lat> <lon> <drive> <offload> <record> <signal>");
    Serial.println("          Example: ctl B3 42.123456 -83.123456 1 0 1 0");
    Serial.println("  tbl  - Enable/disable Serial status table printing for a bot");
    Serial.println("          Example: tbl 3   (enable for bot 3) | tbl -1 (disable)");
    Serial.println();
    Serial.println("Note: RPi commands are prefixed and include a checksum. 'cbp' toggles enforcement.");
    Serial.println("============================");
}