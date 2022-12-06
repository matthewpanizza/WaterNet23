/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/matthewpanizza/Downloads/WaterNet23/WaterNet23CCHub/src/WaterNet23CCHub.ino"
/*
 * Project WaterNet23CCHub
 * Description: Code for the Central Control hub responsible for orchestrating commands to Water Bots
 * Author:
 * Date:
 */


#include "application.h"
#include "SdFat.h"
void init(uint16_t inStep, uint16_t minV, uint16_t maxV, bool switchOnOff, const char * itemString);
void startupPair();
void XBeeLTEPairSet();
void setup();
void loop();
void logMessage(const char *message);
void updateMenu();
void updateBotControl();
void processCommand(const char *command, uint8_t mode, bool sendAck);
void processRPiCommand(const char *command, uint8_t mode);
void setupXBee();
void BLEScan(int BotNumber);
void DataOffloader(uint8_t bot_id);
void RPiStatusUpdate();
void RPiHandler();
void manualMotorControl(uint8_t commandedBot);
static void BLEDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void offloadDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void sendData(const char *dataOut, uint8_t sendMode, bool sendBLE, bool sendXBee, bool sendLTE);
void actionTimer5();
void WaterBotSim(uint8_t count);
void entHandler();
void rHandler();
void lHandler();
void uHandler();
void dHandler();
void jHandler();
void sHandler();
int LTEInputCommand(String cmd);
#line 11 "/Users/matthewpanizza/Downloads/WaterNet23/WaterNet23CCHub/src/WaterNet23CCHub.ino"
#undef min
#undef max
#include <vector>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// Pin Definitions

#define OLED_RESET              A0
#define JOYH_ADC                A2              //Horizontal Joystick
#define JOYV_ADC                A3              //Vertical Joystick ADC pin
#define JOY_BTN                 D23             //Joystick button press
#define L_DPAD                  A4              //Left DPAD button
#define R_DPAD                  A1              //Right DPAD button
#define U_DPAD                  A5              //Up DPAD button
#define D_DPAD                  D7              //Down DPAD button
#define E_DPAD                  D22              //"Enter" DPAD button
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
#define MTR_UPDATE_TIME         750             //Frequency to send manual motor control packet in milliseconds
#define CONTROL_PUB_TIME        5000            //Number of milliseconds between sending control packets to bots
#define STOP_PUB_TIME           5000            //Time between sending stop messages when active
#define WB_MOD_UPDATE_TIME      60000           //Timeout for when status update packets will modify the class, prevents immediate overwrite when changing control variables

//Menu Parameters
#define i2c_Address             0x3c            //initialize with the I2C addr 0x3C Typically eBay OLED's
#define MAX_MENU_ITEMS          7               //Maximum number of settings per bot displayed on the menu (must go to where MenuItems.push_back() and count this)
#define DEBOUNCE_MS             150             //Number of milliseconds to disable button effects after pressing a button
#define OLED_MAX_X              128             //Number of pixels in the X direction
#define OLED_MAX_Y              64              //Number of pixels in the Y direction

//Control Parameters
#define JOY_DEADZONE        35                  //ADC reading offset in the middle to read no change in the the position
#define JOY_MID             2048                //ADC reading when the joystick is centered
#define JOY_MAX             4094                //ADC reading when the joystick is fully forward
#define JOY_MIN             1                   //ADC reading when the joystick is fully backward
#define LTE_MIN_DIFF        3                   //Minimum difference in motor speed to send an update over LTE

//Development Parameters
//#define VERBOSE

// This example does not require the cloud so you can run it in manual mode or
// normal cloud-connected mode
SYSTEM_MODE(SEMI_AUTOMATIC);

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
bool meshPair;                                  //Flag used to start pairing with the closest Bluetooth bot
bool statusTimeout;                             //Timeout for bluetooth and Xbee flag where LTE should be used as backup
int controlUpdateID;                            //Id to publish the next status update to. Updates go in a circle between all discovered bots
bool LTEStopSent;                               //Only send stop command over LTE when first pressed instead of periodically publishing
uint32_t stopTime;                              //Timer for periodically publishing stop command when a stop is active over XBee and BLE
uint32_t controlUpdateTime;                     //Timer for periodically sending control packet to bots     
uint32_t rcTime;                                //Time between sending mtr commands to the currently controlled bot
uint16_t LTEStatuses = MAX_LTE_STATUSES;                            //Counter for number of statuses that have been sent over LTE. Stops sending status over LTE after running out
bool stopActive;                                //Flag set active when the stop button has been pressed, and is cleared after pressing again
uint8_t BLEBotNum;                              //Bot id of the bot currently connected to over BLE
bool ctlSpeedDiff = false;                      //Flag set true when the motor has a significant enough speed change to warrant sending a new speed immediately
uint8_t LSpeed, RSpeed;

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
    float TargetLat = -999.0;       //Target latitude sourced from either the current location (captured for sentry mode) or from the raspberry pi
    float TargetLon = -999.0;       //Target longitude sourced from either the current location (captured for sentry mode) or from the raspberry pi
    float GPSLat = 0.0;             //Current GPS latitude sampled from onboard Ublox module
    float GPSLon= 0.0;              //Current GPS longitude sampled from onboard Ublox module
    uint8_t reqActive = 0;         //Flag set true when a request should be made to get sensor data
    float pH = 0.0;                 //pH sensor reading, populated when a sensor request ("sns" command) is made
    float temp = 0.0;               //Water temperature sensor reading, populated when a sensor request ("sns" command) is made
    float DO = 0.0;                 //Dissolved Oxygen sensor reading, populated when a sensor request ("sns" command) is made
    float Cond = 0.0;               //Conductivity sensor reading, populated when a sensor request ("sns" command) is made
    float MCond = 0.0;              //Mini-conductivity sensor reading, populated when a sensor request ("sns" command) is made
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

class MenuItem{
    public:
        std::vector<String> labels;
        void init(uint16_t inStep, uint16_t minV, uint16_t maxV, bool switchOnOff, const char * itemString){
            minVal = minV;
            maxVal = maxV;
            stepSize = inStep;
            onOffSetting = switchOnOff;
            strcpy(itemName,itemString);
        }
        uint16_t (WaterBot::*MethodPointer);
        bool (WaterBot::*MethodPointerBool);
        uint16_t stepSize;
        bool onOffSetting = false;
        bool customLabel = false;
        bool statOnly = false;
        uint16_t minVal;
        uint16_t maxVal;
        char itemName[10];
};

class MenuPopUp{
    public:
        char primaryLine[10];
        char secondaryLine[30];
        char tertiaryLine [30];
        uint8_t primaryStart = 0;
        uint8_t secondaryStart = 0;
        uint8_t tertiaryStart = 0;
};

WaterBot *ControlledBot;
std::vector<WaterBot> WaterBots;
std::vector<WaterBot> PairBots;
std::vector<PairBot> BLEPair;
std::vector<MenuPopUp> PopUps;

Timer at1(5000,actionTimer5);

MenuItem * SelectedItem;
std::vector<MenuItem> MenuItems;


void BLEScan(int BotNumber = -1);
void XBeeHandler();
void dataLTEHandler(const char *event, const char *data);
void createMenu();

void dataLTEHandler(const char *event, const char *data){
    processCommand(data, 4,false);
    if(logMessages){
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[INFO] Received LTE Message: %s",data);
        logFile.close();
    }
}

void startupPair(){
    startConnect = false;
    oled.clearDisplay();
    oled.setCursor(0,0);
    oled.print("Scanning ");
    oled.fillCircle(115,7,3,SH110X_WHITE);
    oled.display();
    uint8_t loadAnim = 0;
    while(!startConnect){
        oled.setCursor(0,16);
        oled.fillRect(72,16,35,15,0);
        oled.printlnf("Bots: %d",BLEPair.size());
        #ifdef VERBOSE
            Serial.printlnf("Array size: %d",BLEPair.size());
        #endif
        oled.drawCircle(115,7,loadAnim,SH110X_WHITE);
        if(loadAnim-1) oled.fillCircle(115,7,loadAnim-1,0);
        oled.display();
        loadAnim++;
        if(loadAnim > 7){
            loadAnim = 0;
            oled.drawCircle(115,7,7,0);
        }
        BLEScan(-2);
        XBeeHandler();
        XBeeLTEPairSet();
        
        //delay(50);
        if(digitalRead(E_DPAD) == LOW){
            int minRSSI = -999;
            int selectedBot = -1;
            for(PairBot pb: BLEPair){
                if(pb.rssi > minRSSI){
                    minRSSI = pb.rssi;
                    selectedBot = pb.botNum;
                }
                
            }
            if(selectedBot >= 0){
                meshPair = true;    //Did we find any bots over BLE
            }
            uint8_t BLETimeout = 0;
            oled.clearDisplay();
            oled.setCursor(0,0);
            oled.print("Connecting");
            oled.setCursor(0,16);
            oled.printlnf("Bot: %d",selectedBot);
            oled.display();
            botSelect = selectedBot;
            while(meshPair){
                BLEScan(selectedBot);
                BLETimeout++;
                if(WaterBots.size() == 0 && BLETimeout == LTE_BKP_Time) Particle.publish("CCHub", "CCABhwd", PRIVATE);
                if(BLETimeout > BLE_MAX_CONN_TIME){
                    meshPair = false;
                    botSelect = WaterBots.front().botNum;
                }
                delay(100);
            }
            oled.clearDisplay();
            oled.display();
            selectingBots = true;
        }
        
    }
}

void XBeeLTEPairSet(){
    for(WaterBot p: PairBots){
        char replyStr[10];
        sprintf(replyStr,"CCB%dhwa",p.botNum);
        sendData(replyStr,0,true,p.XBeeAvail,p.LTEAvail);
        PairBots.pop_back();
    }
}

void rHandler(void);

void setup() {

    pinMode(E_DPAD,INPUT_PULLDOWN);
    pinMode(U_DPAD,INPUT_PULLDOWN);
    pinMode(D_DPAD,INPUT_PULLDOWN);
    pinMode(L_DPAD,INPUT_PULLDOWN);
    pinMode(R_DPAD,INPUT_PULLDOWN);
    pinMode(JOY_BTN,INPUT_PULLDOWN);
    pinMode(STOP_BTN, INPUT_PULLDOWN);
    
    attachInterrupt(E_DPAD,entHandler,RISING);
    attachInterrupt(U_DPAD,uHandler,RISING);
    attachInterrupt(D_DPAD,dHandler,RISING);
    attachInterrupt(L_DPAD,lHandler,RISING);
    attachInterrupt(R_DPAD,rHandler,RISING);
    attachInterrupt(JOY_BTN,jHandler,RISING);
    attachInterrupt(STOP_BTN,sHandler,RISING);

    delay(5);

    if(digitalRead(U_DPAD) == LOW || digitalRead(D_DPAD) == LOW) Particle.connect();

    debounceTime = millis();
    controlUpdateTime = millis();
    rcTime = millis();
    controlUpdateID = -1;

    Serial.begin(115200);
    Serial1.begin(9600);                        //Start serial for XBee module
    setupXBee();

	BLE.on();
    BLE.setScanTimeout(50);  //100ms scan
    BLE.setTxPower(8);

    peerTxCharacteristic.onDataReceived(BLEDataReceived, &peerTxCharacteristic);
    peerOffloadCharacteristic.onDataReceived(offloadDataReceived, &peerOffloadCharacteristic);

    Particle.subscribe("Bot1dat",dataLTEHandler);
    Particle.function("Input Command", LTEInputCommand);

    offloadingMode = false;
    offloadingDone = false;

    logMessages = true;
    postStatus = false;
    statusTimeout = false;
    stopActive = false;
    LTEStopSent = false;
    
    stopTime = 0;

    char timestamp[16];
    snprintf(timestamp,16,"%02d%02d%04d%02d%02d%02d",Time.month(),Time.day(),Time.year(),Time.hour(),Time.minute(),Time.second());
    strcpy(filenameMessages,DEF_FILENAME);
    strcat(filenameMessages,timestamp);
    strcat(filenameMessages,"_LOG.txt");

    createMenu();

    delay(250);

    oled.begin(i2c_Address, true); // Address 0x3C default
    oled.clearDisplay();
    oled.display();
    oled.setRotation(1);
    
    oled.setTextSize(2);
    oled.setTextColor(SH110X_WHITE);
    oled.setCursor(0,0);
    oled.print(" Starting ");
    oled.display();

    delay(1000);

    if (!sd.begin(chipSelect, SD_SCK_MHZ(8))) {
        #ifdef VERBOSE
        Serial.println("Error: could not connect to SD card!");
        #endif
        logMessages = false;
    }

    MenuPopUp m;
    sprintf(m.primaryLine,"Hello!\0");
    sprintf(m.secondaryLine,"Scanning for Bots\0", 1);
    sprintf(m.tertiaryLine, "OK when bots ready\0",15);
    m.primaryStart = 32;
    m.secondaryStart = 12;
    m.tertiaryStart = 10;
    PopUps.push_back(m);
    
    //startupPair();
    //delay(3000);

    at1.start();

    WaterBotSim(1);

    
}

void loop() {
    if(postStatus){
        sendData("CCABspc",0,false,true,false);                                  
        postStatus = false;
        statusTimeout = false;
    }
    updateMenu();
    updateBotControl();
    manualMotorControl(botSelect);
    if (BLE.connected()) {
        
    }
    else {
    	if (millis() - lastScan >= BLE_SCAN_PERIOD) {
    		lastScan = millis();
    		BLEScan(-1);
    	}

    }
    for(WaterBot &wb: WaterBots){
        if(wb.offloading){
            DataOffloader(wb.botNum);
            wb.offloading = false;
        }
        if(wb.reqActive > 3){
            char tempBuf[10];
            sprintf(tempBuf,"CCB%dsns",wb.reqActive);
            sendData(tempBuf,0,!(wb.XBeeAvail), true, false);
            wb.reqActive = 0;
        }
    }
    XBeeHandler();
    RPiHandler();
    XBeeLTEPairSet();
    RPiStatusUpdate();
    if(stopActive){
        if(millis() - stopTime > STOP_PUB_TIME){
            stopTime = millis();
            sendData("CCABstp",0,true,true,!LTEStopSent);
            LTEStopSent = true;
        }
    }
    delay(10);
}

void logMessage(const char *message){
    if(!logFile.isOpen()){
        logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.println(message);
        logFile.close();
    }
    else logFile.println(message);
}

void printMenuItem(uint8_t id, bool highlighted, bool selected, uint16_t x, uint16_t y, WaterBot wb){
    if(highlighted){
        oled.fillRect(x,y,OLED_MAX_X - 40,16,1);
        oled.setCursor(x+1,y+1);
        oled.setTextSize(2);
        oled.setTextColor(0);
        oled.print(MenuItems.at(id).itemName);
        if(selected){
            oled.fillRect(OLED_MAX_X - 40,y,OLED_MAX_X-1,16,1);
            oled.setCursor(OLED_MAX_X - 39,y+1);
            oled.setTextColor(0);
            if(MenuItems.at(id).onOffSetting){
                if(wb.*(MenuItems.at(id).MethodPointerBool))  oled.printf("On");
                else oled.printf("Off");
            }
            else if(MenuItems.at(id).customLabel){
                oled.printf(MenuItems.at(id).labels.at(wb.*MenuItems.at(id).MethodPointer));
            }
            else oled.printf("%d",wb.*(MenuItems.at(id).MethodPointer));
        }
        else{
            oled.fillRect(OLED_MAX_X - 40,y,OLED_MAX_X-1,16,0);
            oled.setCursor(OLED_MAX_X - 39,y+1);
            oled.setTextColor(1);
            if(MenuItems.at(id).onOffSetting){
                if(wb.*(MenuItems.at(id).MethodPointerBool))  oled.printf("On");
                else oled.printf("Off");
            }
            else if(MenuItems.at(id).customLabel){
                oled.printf(MenuItems.at(id).labels.at(wb.*MenuItems.at(id).MethodPointer));
            }
            else oled.printf("%d",wb.*(MenuItems.at(id).MethodPointer));
        }
        #ifdef VERBOSE
        Serial.printlnf("Printed Highlighted Menu item with name: %s",MenuItems.at(id).itemName);
        #endif
    }
    else{
        oled.fillRect(x,y,OLED_MAX_X - 40,16,0);
        oled.setCursor(x+1,y+1);
        oled.setTextSize(2);
        oled.setTextColor(1);
        oled.print(MenuItems.at(id).itemName);
        oled.fillRect(OLED_MAX_X - 40,y,OLED_MAX_X-1,16,0);
        oled.setCursor(OLED_MAX_X - 39,y+1);
        oled.setTextColor(1);
        if(MenuItems.at(id).onOffSetting){
            if(wb.*(MenuItems.at(id).MethodPointerBool))  oled.printf("On");
            else oled.printf("Off");
        }
        else if(MenuItems.at(id).customLabel){
            oled.printf(MenuItems.at(id).labels.at(wb.*MenuItems.at(id).MethodPointer));
        }
        else oled.printf("%d",wb.*(MenuItems.at(id).MethodPointer));
    }
    
    
    
    MenuItems.at(id);
}

void updateMenu(){
    if(redrawMenu){
        oled.fillRect(0,0,OLED_MAX_X,OLED_MAX_Y,0);
        if(PopUps.size() != 0){  //If there is a queue of pop-ups to be displayed
            oled.drawRect(1,1,126,62,1);
            oled.drawRect(2,2,124,60,1);
            oled.setTextColor(1);
            oled.setCursor(PopUps.back().primaryStart,4);
            oled.setTextSize(2);
            oled.printf(PopUps.back().primaryLine);
            oled.setCursor(PopUps.back().secondaryStart,22);
            oled.setTextSize(1);
            oled.printf(PopUps.back().secondaryLine);
            oled.setCursor(PopUps.back().tertiaryStart,32);
            oled.setTextSize(1);
            oled.printf(PopUps.back().tertiaryLine);
            oled.setCursor(48,45);
            oled.setTextSize(2);
            oled.fillRect(45,44,32,16,1);
            oled.setTextColor(0);
            oled.printf("OK");
            oled.display();
            redrawMenu = false;
            return;
        }
        uint8_t menuSelect = 0;
        for(uint8_t i = 0; i < WaterBots.size(); i++){
            if(WaterBots.at(i).botNum == botSelect){
                oled.setCursor(5+18*i,4);
                oled.setTextSize(1);
                oled.setTextColor(0);
                oled.fillRect(1+i*18,1,14,14,1);
                oled.printf("%d",WaterBots.at(i).botNum);
                menuSelect = i;
            }
            else{
                oled.setCursor(5+18*i,4);
                oled.setTextSize(1);
                oled.setTextColor(1);
                oled.drawRect(1+i*18,1,14,14,1);
                oled.printf("%d",WaterBots.at(i).botNum);
            }
        }
        if(menuItem == 0){
            //Serial.println("Menu item 0");
            if(MenuItems.size() != 0) printMenuItem(0,true,!selectingBots,0,16,WaterBots.at(menuSelect));
            uint8_t loopIter = MenuItems.size();
            if(loopIter > 2) loopIter = 2;
            for(int mi = 1; mi <= loopIter; mi++){
                //Serial.printlnf("Menu item %d", mi);
                printMenuItem(mi,false,!selectingBots,0,16+(16*mi),WaterBots.at(menuSelect));
            }
        }
        else if(menuItem == MAX_MENU_ITEMS-1){
            //Serial.printlnf("Menu item %d", menuItem);
            printMenuItem(menuItem,true,!selectingBots,0,48,WaterBots.at(menuSelect));
            //Serial.printlnf("Menu item %d", menuItem-1);
            printMenuItem(menuItem-1,false,!selectingBots,0,32,WaterBots.at(menuSelect));
            //Serial.printlnf("Menu item %d", menuItem-2);
            printMenuItem(menuItem-2,false,!selectingBots,0,16,WaterBots.at(menuSelect));
        }
        else{
            //Serial.printlnf("Menu item %d", menuItem+1);
            printMenuItem(menuItem+1,false,!selectingBots,0,48,WaterBots.at(menuSelect));
            //Serial.printlnf("Menu item %d", menuItem);
            printMenuItem(menuItem,true,!selectingBots,0,32,WaterBots.at(menuSelect));
            //Serial.printlnf("Menu item %d", menuItem-1);
            printMenuItem(menuItem-1,false,!selectingBots,0,16,WaterBots.at(menuSelect));
        }
        oled.display();
        redrawMenu = false;
    }
}

void updateBotControl(){
    if(updateControl){
        updateControl = false;
        //ControlledBot = nullptr;
        for(WaterBot &wb: WaterBots){
            //if(wb.botNum == botSelect) ControlledBot =  &wb;
            if(wb.updatedControl){
                wb.updatedControl = false;
                wb.publishTime = millis();
                char statusStr[42];
                sprintf(statusStr,"CCB%dctl%0.6f %0.6f %d %d %d",wb.botNum, wb.TargetLat, wb.TargetLon, wb.driveMode, wb.dataRecording, wb.signal);
                #ifdef VERBOSE
                Serial.printlnf("Control Packet: %s",statusStr);
                #endif
                bool rpiLTEStatus = false;
                if(!wb.XBeeAvail && !wb.BLEAvail && LTEStatuses && wb.LTEInitialStatus){
                    rpiLTEStatus = true;
                    LTEStatuses--;
                }
                sendData(statusStr,0,!(wb.XBeeAvail),true,rpiLTEStatus);
                wb.LTEInitialStatus = false;
            }
        }
        //if(ControlledBot == NULL) return;
        //if(ControlledBot->offloading) offloadingMode = true;
        
    }
    if(millis() - controlUpdateTime > CONTROL_PUB_TIME){
        controlUpdateTime = millis();
        if(controlUpdateID == -1){
            if(WaterBots.size() != 0) controlUpdateID = 0;
            else return;
        }
        if(controlUpdateID > WaterBots.size()-1) controlUpdateID = 0;
        WaterBot wb = WaterBots.at(controlUpdateID);
        char statusStr[42];
        sprintf(statusStr,"CCB%dctl%0.6f %0.6f %d %d %d",wb.botNum,wb.TargetLat, wb.TargetLon, wb.driveMode, wb.dataRecording, wb.signal);
        bool sendLTEStat = false;
        if(!wb.XBeeAvail && !wb.BLEAvail && (millis() - wb.LTELastStatTime > LTE_CTL_PERIOD)){
            sendLTEStat = true;
            WaterBots.at(controlUpdateID).LTELastStatTime = millis();
        }
        sendData(statusStr,0,false,true,sendLTEStat);
        if(controlUpdateID < WaterBots.size()-1) controlUpdateID++;
        else controlUpdateID = 0;
    }
}

void processCommand(const char *command, uint8_t mode, bool sendAck){
    //Process if command is addressed to this bot "Bx" or all bots "AB"
    if((command[2] == 'A' && command[3] == 'B') || (command[2] == 'C' && command[3] == 'C')){
        char rxIDBuf[1];
        rxIDBuf[0] = command[1];
        uint8_t rxBotID = atoi(rxIDBuf);
        if(rxBotID > 9) return;
        uint8_t checksum;
        char dataStr[strlen(command)-8];
        dataStr[strlen(command)-9] = '\0';
        char cmdStr[4];
        cmdStr[3] = '\0';
        char checkStr[3];
        checkStr[0] = command[strlen(command)-2];
        checkStr[1] = command[strlen(command)-1];
        checkStr[2] = '\0';
        checksum = (uint8_t)strtol(checkStr, NULL, 16);       // number base 16
        #ifdef VERBOSE
        Serial.printlnf("Checksum: %02x, %03d",checksum,checksum);
        #endif
        for(uint8_t i = 4; i < strlen(command)-2;i++){
            if(i < 7) cmdStr[i-4] = command[i];
            else dataStr[i-7] = command[i];
        }
        if(checksum != strlen(command)-2){
            #ifdef VERBOSE
            Serial.printlnf("String Len: %d, Checksum: %d",strlen(command)-2,checksum);
            Serial.println("Warning, checksum does not match");
            #endif
            logMessage("[WARN] Warning, checksum does not match!");
            if((command[1] >= '0' && command[1] <= '9') || command[1] == 'C'){
                char rxBotNum[2];
                rxBotNum[0] = command[0];
                rxBotNum[1] = command[1];
            }
            return;
        }
        bool newBot = true;
        WaterBot *TargetWB = nullptr;
        int index = 0;
        for(WaterBot w: WaterBots){
            if(rxBotID == w.botNum){
                newBot = false;
                TargetWB = &WaterBots.at(index);
            }
            index++;
        }
        if(newBot){
            WaterBot newWaterbot;
            newWaterbot.botNum = rxBotID;
            WaterBots.push_back(newWaterbot);
            TargetWB = &WaterBots.back();
            redrawMenu = true;
        }
        else if(!strcmp(cmdStr,"sup")){
            for(WaterBot &w: WaterBots){
                if(rxBotID == w.botNum){
                    uint8_t battpct;
                    uint16_t statflags;
                    char testLat[12];
                    char testLon[12];
                    uint16_t panelPwr;
                    uint16_t battPwr;
                    sscanf(dataStr,"%u %u %s %s %d %d",&battpct,&statflags,testLat,testLon, &battPwr, &panelPwr);
                    w.battPercent = battpct;
                    w.LTEAvail = statflags & 1;
                    w.XBeeAvail = (statflags >> 1) & 1;
                    w.BLEAvail = (statflags >> 2) & 1;
                    w.lowBatt = (statflags >> 6) & 1;
                    w.GPSAvail = (statflags >> 8) & 1;
                    w.CompassAvail = (statflags >> 9) & 1;
                    w.SDAvail = (statflags >> 10) & 1;
                    w.GPSLat = atof(testLat);
                    w.GPSLon = atof(testLon);
                    w.panelPower = panelPwr;
                    w.battPower = battPwr;
                    w.updatedStatus = true;
                    if(millis() - w.publishTime > WB_MOD_UPDATE_TIME){
                        w.offloading = (statflags >> 3) & 1;
                        w.driveMode = (statflags >> 4) & 3;
                        w.dataRecording = (statflags >> 7) & 1;
                    }
                    if(w.lowBatt && !w.warnedLowBatt){
                        w.warnedLowBatt = true;
                        MenuPopUp m;
                        sprintf(m.primaryLine,"Warning\0");
                        sprintf(m.secondaryLine,"Bot %d\0", w.botNum);
                        sprintf(m.tertiaryLine, "Low Battery: %d\0",w.battPercent);
                        m.primaryStart = 20;
                        m.secondaryStart = 40;
                        m.tertiaryStart = 20;
                        PopUps.push_back(m);
                        redrawMenu = true;
                        
                    }
                    if(!w.SDAvail && !w.warnedSDCard){
                        w.warnedSDCard = true;
                        MenuPopUp m;
                        sprintf(m.primaryLine,"Warning\0");
                        sprintf(m.secondaryLine,"Bot %d\0", w.botNum);
                        sprintf(m.tertiaryLine, "SD Card Failed\0");
                        m.primaryStart = 20;
                        m.secondaryStart = 40;
                        m.tertiaryStart = 20;
                        PopUps.push_back(m);
                        redrawMenu = true;
                        
                    }
                    if((!w.CompassAvail || !w.GPSAvail) && !w.warnedTelem){
                        MenuPopUp m;
                        w.warnedTelem = true;
                        sprintf(m.primaryLine,"Warning\0");
                        sprintf(m.secondaryLine,"Bot %d\0", w.botNum);
                        sprintf(m.tertiaryLine, "GPS/Compass Error\0");
                        m.primaryStart = 20;
                        m.secondaryStart = 40;
                        m.tertiaryStart = 10;
                        PopUps.push_back(m);
                        redrawMenu = true;
                    }
                    if(botSelect == w.botNum) redrawMenu = true;
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
        if(!strcmp(cmdStr,"sns")){
            char GPSLatstr[12];
            char GPSLonstr[12];
            uint32_t do_in,pH_in,cond_in,mcond_in,temp_in;
            sscanf(dataStr,"%s %s %d %d %d %d %d",GPSLatstr,GPSLonstr,&do_in,&pH_in,&cond_in,&mcond_in,&temp_in);
            TargetWB->DO = ((float)do_in)/1000.0;
            TargetWB->pH = ((float)pH_in)/1000.0;
            TargetWB->Cond = ((float)cond_in)/1000.0;
            TargetWB->MCond = ((float)mcond_in)/1000.0;
            TargetWB->temp = ((float)temp_in)/1000.0;
            #ifdef VERBOSE
            Serial.printlnf("Bot #: %d Temp: %f", TargetWB->botNum,TargetWB->temp);
            #endif
        }
        else if(!strcmp(cmdStr,"hwd")){  //Hello World! - Received startup pairing message
            bool newBot = true;
            for(WaterBot w: WaterBots){
                if(rxBotID == w.botNum) newBot = false;
            }
            if(newBot){
                #ifdef VERBOSE
                Serial.println("Found a new water bot ID");
                #endif
                WaterBot newWaterbot;
                if(mode == 1) newWaterbot.BLEAvail = true;
                else if(mode == 2) newWaterbot.XBeeAvail = true;
                else if(mode == 3) newWaterbot.LTEAvail = true;
                newWaterbot.botNum = rxBotID;
                WaterBots.push_back(newWaterbot);
                PairBots.push_back(newWaterbot);
                redrawMenu = true;
            }
        }
        else if(!strcmp(cmdStr,"pts")){
            if(!logFile.isOpen()){
                logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
                logFile.printlnf("[PUTS] Received String Command: %s",dataStr);
                logFile.close();
            }
            else logFile.printlnf("[PUTS] Received String Command: %s",dataStr);
        }
        else if(!strcmp(cmdStr,"ldt") || !strcmp(cmdStr,"ldb")){
            MenuPopUp m;
            sprintf(m.primaryLine,"Warning\0");
            sprintf(m.secondaryLine,"Bot %d\0", rxBotID);
            sprintf(m.tertiaryLine, "Leak shutoff\0");
            m.primaryStart = 20;
            m.secondaryStart = 40;
            m.tertiaryStart = 30;
            PopUps.push_back(m);
            redrawMenu = true;
        }
        else if(!strcmp(cmdStr,"wld") || !strcmp(cmdStr,"wlb")){
            MenuPopUp m;
            sprintf(m.primaryLine,"Warning\0");
            sprintf(m.secondaryLine,"Bot %d\0", rxBotID);
            sprintf(m.tertiaryLine, "Leak detected\0");
            m.primaryStart = 20;
            m.secondaryStart = 40;
            m.tertiaryStart = 25;
            PopUps.push_back(m);
            redrawMenu = true;
        }
    }
}

void processRPiCommand(const char *command, uint8_t mode){
    if(command[0] == 'R' && command[1] == 'P'){  
        #ifdef VERBOSE
        Serial.println("Received Pi command");
        #endif
        uint8_t checksum;
        char dataStr[strlen(command)-8];
        dataStr[strlen(command)-9] = '\0';
        char cmdStr[4];
        cmdStr[3] = '\0';
        char checkStr[3];
        checkStr[0] = command[strlen(command)-2];
        checkStr[1] = command[strlen(command)-1];
        checkStr[2] = '\0';
        checksum = (uint8_t)strtol(checkStr, NULL, 16);       // number base 16
        #ifdef VERBOSE
        Serial.printlnf("Checksum: %02x, %03d, Checkstr: %s",checksum,checksum,command);
        #endif
        for(uint8_t i = 4; i < strlen(command)-2;i++){
            if(i < 7) cmdStr[i-4] = command[i];
            else dataStr[i-7] = command[i];
        }
        if(checksum != strlen(command)-2){
            #ifdef VERBOSE
            Serial.printlnf("String Len: %d, Checksum: %d",strlen(command)-2,checksum);
            #endif
            if(!logFile.isOpen()){
                logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
                logFile.printlnf("[WARN] RPi Message Checksum Does Not Match!: %s",command);
                logFile.close();
            }
            else logFile.printlnf("[WARN] RPi Message Checksum Does Not Match!: %s",command);
            #ifdef VERBOSE
            Serial.println("Warning, checksum does not match");
            #endif
            if((command[1] >= '0' && command[1] <= '9') || command[1] == 'C'){
                char rxBotNum[2];
                rxBotNum[0] = command[0];
                rxBotNum[1] = command[1];
            }
            return;
        }
        if(!strcmp(cmdStr,"ctl")){
            char idStr[10];
            char GPSLatstr[12];
            char GPSLonstr[12];
            uint8_t offloading, drivemode, recording, signal;
            sscanf(dataStr,"%s %s %s %d %d %d %d",idStr,GPSLatstr,GPSLonstr,&drivemode,&offloading,&recording,&signal);
            char botChar[2] = {command[8], '\0'};
            uint8_t targetBot = atoi(botChar);
            #ifdef VERBOSE
            Serial.printlnf("Got a command packet from Pi for Bot %d",targetBot);
            #endif
            for(WaterBot &wb: WaterBots){
                if(wb.botNum == targetBot){
                    wb.TargetLat = atof(GPSLatstr);
                    wb.TargetLon = atof(GPSLonstr);
                    wb.driveMode = drivemode;
                    wb.offloading = offloading;
                    wb.dataRecording = recording;
                    wb.signal = signal;
                    wb.LTEInitialStatus = true;
                    if(botSelect == wb.botNum) redrawMenu = true;
                    wb.updatedControl = true;
                    updateControl = true;
                    return;
                }
            }
            //RPCCctlB%d %0.6f %0.6f %d %d %d  //Botnumber, target lat, target lon, drive mode, offloading, data recording
        }
    }
}

void setupXBee(){
    Serial1.printf("\n");    //First character to set Bypass mode
    delay(20);              //Wait some time before sending next character
    Serial1.printf("B");     //Second character to set Bypass mode
    delay(20);
}

void BLEScan(int BotNumber){
    size_t count = BLE.scan(scanResults, SCAN_RESULT_COUNT);
	if (count > 0) {
		for (uint8_t ii = 0; ii < count; ii++) {
			BleUuid foundServiceUuid;
			size_t svcCount = scanResults[ii].advertisingData().serviceUUID(&foundServiceUuid, 1);
            if (svcCount > 0 && foundServiceUuid == serviceUuid) {
                uint8_t BLECustomData[CUSTOM_DATA_LEN];
                scanResults[ii].advertisingData().customData(BLECustomData,CUSTOM_DATA_LEN);
                if(BotNumber == -2){
                    #ifdef VERBOSE
                    Serial.printlnf("Found Bot #: %d %d %d %d %d %d %d %d, services: %d",BLECustomData[0],BLECustomData[1],BLECustomData[2],BLECustomData[3],BLECustomData[4],BLECustomData[5],BLECustomData[6],BLECustomData[7], svcCount);
                    #endif
                    bool newBot = true;
                    PairBot *existingBot;
                    for(PairBot p: BLEPair){
                        if(BLECustomData[0] == p.botNum){
                            newBot = false;
                            existingBot = &p;
                            
                        } 
                    }
                    if(newBot){
                        
                        PairBot NewBot;
                        NewBot.rssi = scanResults[ii].rssi();
                        NewBot.botNum = BLECustomData[0];
                        BLEPair.push_back(NewBot);
                        #ifdef VERBOSE
                        Serial.printlnf("Found new bot: %d", BLECustomData[0],BLEPair.size());
                        #endif
                    }
                    else{
                        existingBot->rssi = (scanResults[ii].rssi() + existingBot->rssi) >> 1;
                    }
                }
                if(BotNumber == -1 || BotNumber == BLECustomData[0]){   //Check if a particular bot number was specified
                    peer = BLE.connect(scanResults[ii].address());
				    if (peer.connected()) {
                        meshPair = false;
                        startConnect = true;
                        uint8_t bufName[BLE_MAX_ADV_DATA_LEN];
                        scanResults[ii].advertisingData().customData(bufName, BLE_MAX_ADV_DATA_LEN);
					    peer.getCharacteristicByUUID(peerTxCharacteristic, txUuid);
					    peer.getCharacteristicByUUID(peerRxCharacteristic, rxUuid);
                        peer.getCharacteristicByUUID(peerOffloadCharacteristic, offldUuid);
                        #ifdef VERBOSE
						Serial.printlnf("Connected to Bot %d",bufName[0]);
                        #endif
                        bool newBot = true;
                        WaterBot newWaterbot;
                        newWaterbot.BLEAvail = true;
                        newWaterbot.botNum = bufName[0];
                        PairBots.push_back(newWaterbot);
                        for(WaterBot &w: WaterBots){
                            if(bufName[0] == w.botNum){
                                newBot = false;
                                w.BLEAvail = true;
                                BLEBotNum = w.botNum;
                            }
                        }
                        if(newBot){
                            #ifdef VERBOSE
                            Serial.println("Found a new water bot ID");
                            #endif
                            WaterBots.push_back(newWaterbot);
                            BLEBotNum = newWaterbot.botNum;
                            redrawMenu = true;
                        }
                    }
                    break;
                }
			}
		}
	}
}

void DataOffloader(uint8_t bot_id){
    uint8_t OffloadingBot = bot_id;
    if (!logDir.open("/")) {
        offloadingDone = true;
        MenuPopUp m;
        sprintf(m.primaryLine,"Warning\0");
        sprintf(m.secondaryLine,"CCHub");
        sprintf(m.tertiaryLine, "SD Card Failed\0");
        m.primaryStart = 20;
        m.secondaryStart = 60;
        m.tertiaryStart = 20;
        PopUps.push_back(m);
        redrawMenu = true;
        #ifdef VERBOSE
        Serial.println("Error, could not open root directory on SD Card. Is it inserted?");
        #endif
        return;
    }
    uint32_t startScanTime = millis();
    #ifdef VERBOSE
    Serial.printlnf("Requested SD Card Data from Bot %d Over BLE\n",BLEBotNum);
    #endif
    if(BLEBotNum != bot_id){
        #ifdef VERBOSE
        Serial.printlnf("Currently connected to Bot %s, need to connect to Bot %d",BLEBotNum,OffloadingBot);
        #endif
        offloadingDone = true;
        MenuPopUp m;
        sprintf(m.primaryLine,"Info\0");
        sprintf(m.secondaryLine,"Not connected to BLE\0");
        sprintf(m.tertiaryLine, "Switching BLE conn\0");
        m.primaryStart = 30;
        m.secondaryStart = 5;
        m.tertiaryStart = 10;
        PopUps.push_back(m);
        redrawMenu = true;
        BLE.disconnect(peer);
        while(!BLE.connected() && millis() - startScanTime < 15000){
            BLEScan(bot_id);
            delay(50);
        }
        
        char OffloadCommand[10];
        snprintf(OffloadCommand,10,"CCB%ddmp",OffloadingBot);
        sendData(OffloadCommand,0,true,false,false);
        #ifdef VERBOSE
        if(BLE.connected()) Serial.printlnf("Successfully connected to Bot %d", BLEBotNum);
        #endif
    }
    else{
        char OffloadCommand[10];
        snprintf(OffloadCommand,10,"CCB%ddmp",OffloadingBot);
        sendData(OffloadCommand,0,true,false,false);
    }
    #ifdef VERBOSE
    Serial.printlnf("Starting file transfer from Bot %d",BLEBotNum);
    #endif
    if(millis() - startScanTime > 15000){
        if(logDir.isOpen()) logDir.close();
        return;
    }
    if(BLE.connected()) offloadingDone = false;
    //offloadingDone = false;
    while(!offloadingDone) delay(100);
    #ifdef VERBOSE
    Serial.printlnf("Finished transferring file from Bot %d",BLEBotNum);
    #endif
    if(logDir.isOpen()) logDir.close();
    offloadingMode = false;
}

void RPiStatusUpdate(){
    for(WaterBot &wb: WaterBots){
        if(wb.updatedStatus){
            uint16_t statusFlags;
            statusFlags = 0;
            statusFlags = wb.LTEAvail;
            statusFlags |= wb.XBeeAvail << 1;
            statusFlags |= wb.BLEAvail << 2;
            statusFlags |= wb.offloading << 3;
            statusFlags |= wb.driveMode << 4;
            statusFlags |= wb.lowBatt << 6;
            statusFlags |= wb.dataRecording << 7;
            statusFlags |= wb.GPSAvail << 8;
            statusFlags |= wb.CompassAvail << 9;
            statusFlags |= wb.SDAvail << 10;
            Serial.printlnf("CCRPsupB%d %d %0.6f %0.6f %d %d %d",wb.botNum, wb.battPercent, wb.GPSLat, wb.GPSLon, statusFlags,wb.battPower, wb.panelPower);
            wb.updatedStatus = false;
        }
    }
}

void RPiHandler(){
    while(Serial.available()){
            String data = Serial.readStringUntil('\n');
            #ifdef VERBOSE
            Serial.println(data);
            #endif
            char buffer[data.length() + 2];
            data.toCharArray(buffer,data.length()+1);
            buffer[data.length() + 1] = 0;
            //if(data.length() > 1 && data.charAt(data.length()-1) == '\r') buffer[data.length()-1] = 0;
            //if(data.length() > 1 && data.charAt(data.length()-1) == '\n') buffer[data.length()-1] = 0;
            processRPiCommand(buffer,3);
            if(logMessages){
                if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
                logFile.printlnf("[INFO] Received Raspberry Pi Message: %s",data);
                logFile.close();
            }
    }
}

void XBeeHandler(){  
    while(Serial1.available()){
        String data = Serial1.readStringUntil('\n');
        char buffer[data.length()];
        for(uint16_t i = 0 ; i < data.length(); i++) buffer[i] = data.charAt(i);
        if(data.length() > 1 && data.charAt(data.length()-1) == '\r') buffer[data.length()-1] = 0;
        #ifdef VERBOSE
        Serial.println("New XBee Command:");
        Serial.println(data); 
        #endif
        processCommand(buffer,2,true);
        if(logMessages){
            if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
            logFile.printlnf("[INFO] Received XBee Message: %s",data);
            logFile.close();
        }
    }
}

void manualMotorControl(uint8_t commandedBot){
    static uint8_t lastLSpeed;
    static uint8_t lastRSpeed;

    char mtrStr[15];
    int VRead, HRead, VSet, HSet;
    VRead = 4095-analogRead(JOYV_ADC);
    HRead = analogRead(JOYH_ADC);
    if(VRead < JOY_MID - JOY_DEADZONE){
        VSet = -90 * (VRead - (JOY_MID - JOY_DEADZONE))/(JOY_MIN - (JOY_MID - JOY_DEADZONE));
        if(VSet < -90) VSet = -90;
    }
    else if(VRead > JOY_MID + JOY_DEADZONE){
        VSet = 90 * (VRead - (JOY_MID + JOY_DEADZONE))/(JOY_MAX - (JOY_MID + JOY_DEADZONE));
        if(VSet > 90) VSet = 90;
    }
    else{
        VSet = 0;
    }
    if(HRead < JOY_MID - JOY_DEADZONE){
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
    if(VSet > 0){
        if(HSet > 0){
            if(HSet > VSet){        //H = 90, V = 0 -> LS = 135, RS = 45, H = 90, V = 90 -> LS = 180, RS = 135
                LSpeed = 90 + HSet/2 + VSet/2;
                RSpeed = 90 - HSet/2 + VSet;
            }
            else{
                LSpeed = 90 + VSet;
                RSpeed = 90 - HSet/2 + VSet;
            }
        }
        else{
            if((0-HSet) > VSet){
                RSpeed = 90 - HSet/2 + VSet/2;
                LSpeed = 90 + HSet/2 + VSet;
            }
            else{
                RSpeed = 90 + VSet;
                LSpeed = 90 + HSet/2 + VSet;
            }
        }
    }
    else{
        if(HSet > 0){
            if(HSet > (0-VSet)){        // H = 90, V = 0 -> LS = 135, RS = 45, H = 90, V = -90 -> LS = 0; RS = 45
                LSpeed = (90 + HSet/2 + VSet*1.5 );
                RSpeed = (90 - HSet/2);      
            }
            else{
                LSpeed = 90 + VSet;
                RSpeed = 90 + HSet/2 + VSet;
            }
        }
        else{
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

    if(lastLSpeed - LSpeed > LTE_MIN_DIFF || LSpeed - lastLSpeed > LTE_MIN_DIFF){
        lastLSpeed = LSpeed;
        ctlSpeedDiff = true;
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

    if(!stopActive && ((millis() - rcTime > MTR_UPDATE_TIME) || (ctlSpeedDiff && millis() - rcTime > 75))){
        ctlSpeedDiff = false;
        rcTime = millis();
        for(WaterBot wb: WaterBots){
            if(wb.driveMode == 0 && wb.botNum == botSelect){
                sprintf(mtrStr,"CCB%dmtr%03d%03d",commandedBot, LSpeed, RSpeed);
                bool sendMTRLTE = false;
                if(!wb.XBeeAvail && !wb.BLEAvail && ctlSpeedDiff && (millis() - wb.LastMtrTime > MTR_LTE_PERIOD)){
                    wb.LastMtrTime = millis();
                    sendMTRLTE = true;
                }
                sendData(mtrStr,0,!(wb.XBeeAvail), true, sendMTRLTE);
                //Serial.printlnf("Time :%d, Speed: %d %d", millis(), LSpeed, RSpeed);
            }
        }
    }
}

static void BLEDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    char btBuf[len+1];
    for (size_t ii = 0; ii < len; ii++) btBuf[ii] = data[ii];
    if(btBuf[len-1] != '\0') btBuf[len] = '\0';
    else btBuf[len-1] = '\0';
    #ifdef VERBOSE
    Serial.print("New BT Command: ");
    Serial.println(btBuf);
    #endif
    processCommand(btBuf,1,true);
    if(logMessages){
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[INFO] Received BLE Message: %s",btBuf);
        logFile.close();
    }
}

void offloadDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    char fileCommand[8 + MAX_FILENAME_LEN];
    memset(fileCommand,0,8 + MAX_FILENAME_LEN);
    memcpy(fileCommand,data,8);
    
    if(fileCommand[0] == 'f'){
        //Serial.printlnf("Found an 'f' command %s",fileCommand);
        if(!strcmp(fileCommand,"filename")){
            if(myFile.isOpen()) myFile.close();
            memcpy(fileCommand,data,8 + MAX_FILENAME_LEN);
            memset(offloadFilename,0,MAX_FILENAME_LEN);
            strncpy(offloadFilename,fileCommand+8,MAX_FILENAME_LEN);
            if(sd.exists(offloadFilename)){
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
        else if(!strcmp(fileCommand,"filecomp")){
            #ifdef VERBOSE
            Serial.printlnf("Reached end of file: %s",offloadFilename);
            #endif
            if(myFile.isOpen()) myFile.close();
            return;
        }
        else if(!strcmp(fileCommand,"filedone")){
            #ifdef VERBOSE
            Serial.println("Received done command");
            #endif
            offloadingDone = true;
            if(myFile.isOpen()) myFile.close();
            return;
        }
    }
    
    char dataStr[len];
    memcpy(dataStr,data,len);
    myFile.print(dataStr);
    #ifdef VERBOSE
    Serial.println(dataStr);
    #endif
}

void sendData(const char *dataOut, uint8_t sendMode, bool sendBLE, bool sendXBee, bool sendLTE){
    char outStr[strlen(dataOut)+2];
    sprintf(outStr,"%s%02x",dataOut,strlen(dataOut));
    #ifdef VERBOSE
    Serial.println(outStr);
    #endif
    if(sendLTE || sendMode == 4){
        Particle.publish("CCHub", outStr, PRIVATE);
        sendLTE = false;
    }
    if((sendBLE || sendMode == 1) && BLE.connected()){
        uint8_t txBuf_tmp[strlen(outStr)];
        memcpy(txBuf_tmp,outStr,strlen(outStr));
        peerRxCharacteristic.setValue(txBuf_tmp, strlen(outStr));
    }
    if(sendXBee || sendMode == 2){
        Serial1.println(outStr);
    }
}

void actionTimer5(){
    postStatus = true;
    for(WaterBot &w: WaterBots){
        w.reqActive++;
    }
    //if(!BLE.connected)
}

void WaterBotSim(uint8_t count){
    if(count + WaterBots.size() > 10) count = 10-WaterBots.size();
    uint8_t botloop = count+WaterBots.size();
    for(uint8_t temp = 0; temp < botloop; temp++){
        int dupeBot = false;
        for(WaterBot wb: WaterBots){
            if(wb.botNum == temp) dupeBot = true;
        }
        if(dupeBot) continue;
        WaterBot simBot;
        simBot.driveMode = 0;
        simBot.botNum = temp;
        simBot.BLEAvail = false;
        simBot.XBeeAvail = true;
        simBot.LTEAvail = false;
        simBot.SDAvail = true;
        simBot.battPercent = random(100);
        WaterBots.push_back(simBot);
    }

}

void createMenu(){
    MenuItem dataRecord;
    dataRecord.init(1,0,1,true,"Record");
    dataRecord.MethodPointerBool = &WaterBot::dataRecording;

    MenuItem battStat;
    battStat.init(1,0,100,false,"Battery");
    battStat.statOnly = true;
    battStat.MethodPointer = &WaterBot::battPercent;

    MenuItem offloadItem;
    offloadItem.init(1,0,1,true,"Offload");
    offloadItem.MethodPointerBool = &WaterBot::offloading;

    MenuItem sentryToggle;
    sentryToggle.init(1,0,2,false,"Sentry");
    sentryToggle.customLabel = true;
    sentryToggle.statOnly = false;
    sentryToggle.labels.push_back("Rem");
    sentryToggle.labels.push_back("Sen");
    sentryToggle.labels.push_back("Aut");
    sentryToggle.MethodPointer = &WaterBot::driveMode;

    MenuItem signalToggle;
    signalToggle.init(1,0,1,true,"Signal");
    signalToggle.statOnly = false;
    signalToggle.MethodPointerBool = &WaterBot::signal;

    MenuItem solStat;
    solStat.init(1,0,999,false,"SolPwr");
    solStat.statOnly = true;
    solStat.MethodPointer = &WaterBot::panelPower;

    MenuItem battPwr;
    battPwr.init(1,0,999,false,"BatPwr");
    battPwr.statOnly = true;
    battPwr.MethodPointer = &WaterBot::battPower;

    MenuItems.push_back(dataRecord);
    MenuItems.push_back(battStat);
    MenuItems.push_back(sentryToggle);
    MenuItems.push_back(offloadItem);
    MenuItems.push_back(signalToggle);
    MenuItems.push_back(solStat);
    MenuItems.push_back(battPwr);

    SelectedItem = &MenuItems.at(menuItem);
}

void entHandler(){
    redrawMenu = true;  
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    #ifdef VERBOSE
    Serial.println("Enter trigger");
    #endif
    debounceTime = millis();
    if(PopUps.size() != 0){
        PopUps.pop_back();
        return;
    }
    selectingBots = !selectingBots;
    if(modifiedValue){
        updateControl = true;
        modifiedValue = false;
    }
}

void rHandler(){
    redrawMenu = true;  
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    debounceTime = millis();
    #ifdef VERBOSE
    Serial.println("Right trigger");
    #endif
    if(selectingBots){
        if(botSelect != WaterBots.back().botNum){
            bool findCurrent = false;
            for(WaterBot &ws: WaterBots){
                if(findCurrent){
                    botSelect = ws.botNum;
                    ControlledBot = &ws;
                    break;
                }
                if(ws.botNum == botSelect) findCurrent = true;
            }
        }
    }
    else{
        //int index = 0;
        for(WaterBot &ws: WaterBots){
            if(ws.botNum == botSelect){
                MenuItem *curItem = SelectedItem;
                #ifdef VERBOSE
                Serial.println(curItem->itemName);
                #endif
                if(curItem == nullptr) return;
                if(curItem->statOnly) return;
                if(curItem->onOffSetting){
                    #ifdef VERBOSE
                    Serial.println("Modified an On/Off Control");
                    Serial.printlnf("Bot: %d, Modified ",ws.botNum);
                    #endif
                    ws.*(curItem->MethodPointerBool) = true;
                }
                else{
                    
                    if(ws.*(curItem->MethodPointer) < curItem->maxVal) ws.*(curItem->MethodPointer) += curItem->stepSize;
                }
                modifiedValue = true;
                ws.updatedControl = true;
            }
            //index++;
        }
    }
}
//Up button interrupt handler - used to move between which bot is selected and also modify the value of menu items
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

//Up button interrupt handler - used to move up in list of menu items
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

//Down button interrupt handler - used to move down in list of menu items
void dHandler(){
    redrawMenu = true;                                      //Set redraw flag always so the display is updated with new highlighted item
    if(millis()-debounceTime < DEBOUNCE_MS) return;         //debounce this button, to make sure only one trigger is registered per press
    debounceTime = millis();
    if(menuItem < MAX_MENU_ITEMS-1) menuItem++;             //Go down by one menu item by incrementing the counter by one, as long as we are not at the bottom already
    SelectedItem = &MenuItems.at(menuItem);                 //Update which item is selected so the redraw function can use it
    #ifdef VERBOSE
    Serial.println("Down trigger");
    #endif
}

//Joystick click handler, used to capture current latitude and longitude and enter sentry mode with those captured points
void jHandler(){
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    debounceTime = millis();
    #ifdef VERBOSE
    Serial.println("Joystick trigger");
    #endif
}

//Stop button interrupt handler - sets stop flag for all bots and creates a pop-up showing that a stop was set or cleared
void sHandler(){    
    if(millis()-debounceTime < DEBOUNCE_MS) return;         //debounce this button, to make sure only one trigger is registered per press
    debounceTime = millis();
    if(stopActive){                                         //Check if the user has initiated a stop, if so, then exit stop mode
        MenuPopUp m;                                        //Create pop-up to indicate to user that we are exiting stop mode
        sprintf(m.primaryLine,"CLEARED\0");                 //Main display line
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
        sprintf(m.primaryLine,"STOPPED\0");                 //Main display line
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