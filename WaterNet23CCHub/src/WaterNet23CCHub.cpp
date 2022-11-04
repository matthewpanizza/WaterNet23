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
void init(uint8_t inStep, uint8_t minV, uint8_t maxV, bool switchOnOff, const char * itemString);
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
void actionTimer60();
void WaterBotSim(uint8_t count);
void entHandler();
void rHandler();
void lHandler();
void uHandler();
void dHandler();
void jHandler();
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

//Program Parameters

#define DEF_FILENAME            "WaterBot"
#define BLE_OFFLD_BUF           100
#define CUSTOM_DATA_LEN         8
#define MAX_FILENAME_LEN        30
#define MAX_ERR_BUF_SIZE        15              //Buffer size for error-return string
#define MAX_LTE_STATUSES        25

//Timing Control

#define XBEE_BLE_MAX_TIMEOUT    36
#define BLE_MAX_CONN_TIME       200             //20 second max time to successfully pair to bot
#define LTE_BKP_Time            100             //Send LTE request after 10 seconds if not connected to any bot
#define MTR_UPDATE_TIME         200             //Frequency to send manual motor control packet in milliseconds
#define CONTROL_PUB_TIME        5000
#define WB_MOD_UPDATE_TIME      60000           //Timeout for when status update packets will modify the class, prevents immediate overwrite when changing control variables

//Menu Parameters
#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
#define MAX_MENU_ITEMS          5
#define DEBOUNCE_MS             150
#define OLED_MAX_X              128
#define OLED_MAX_Y              64

//Control Parameters
#define JOY_DEADZONE        35
#define JOY_MID             2048
#define JOY_MAX             4094
#define JOY_MIN             1

//Development Parameters
//#define VERBOSE

// This example does not require the cloud so you can run it in manual mode or
// normal cloud-connected mode
SYSTEM_MODE(MANUAL);

// These UUIDs were defined by Nordic Semiconductor and are now the defacto standard for
// UART-like services over BLE. Many apps support the UUIDs now, like the Adafruit Bluefruit app.
const BleUuid serviceUuid("b4206910-dc4b-5743-c8b1-92d0e75182b0");
const BleUuid rxUuid("b4206912-dc4b-5743-c8b1-92d0e75182b0");
const BleUuid txUuid("b4206913-dc4b-5743-c8b1-92d0e75182b0");
const BleUuid offldUuid("b4206914-dc4b-5743-c8b1-92d0e75182b0");

const size_t UART_TX_BUF_SIZE = 30;
const size_t SCAN_RESULT_COUNT = 20;

BleScanResult scanResults[SCAN_RESULT_COUNT];

BleCharacteristic peerTxCharacteristic;
BleCharacteristic peerRxCharacteristic;
BleCharacteristic peerOffloadCharacteristic;
BlePeerDevice peer;

//OLED Object
Adafruit_SH1107 oled = Adafruit_SH1107(64, 128, &Wire, OLED_RESET);

//SD File system object
SdFat sd((SPIClass*)&SPI1);

File myFile;
File logFile;
File logDir;

size_t txLen = 0;

const unsigned long SCAN_PERIOD_MS = 2000;
unsigned long lastScan = 0;
bool offloadingMode;
bool offloadingDone;
char offloadFilename[MAX_FILENAME_LEN];
char filenameMessages[MAX_FILENAME_LEN];
bool remoteRx = false;
bool logMessages;
bool startConnect;
bool postStatus;
bool meshPair;
bool botPairRx;
bool statusTimeout;
int controlUpdateID;
uint32_t controlUpdateTime;
uint32_t rcTime;
uint8_t errCmdMode;
uint8_t errModeReply;
char errCmdStr[3];
char txBuf[UART_TX_BUF_SIZE];
char errBuf[MAX_ERR_BUF_SIZE];
uint8_t LTEStatuses;

//Menu variables
uint8_t botSelect = 0;
bool redrawMenu = true;
bool selectingBots = true;
uint8_t menuItem = 0;
bool selectingItem = false;
bool modifiedValue = false;
bool updateControl = false;
uint32_t debounceTime;

class WaterBot{
    public:
    bool updatedStatus = true;
    bool updatedControl = true;
    uint8_t botNum;
    uint8_t battPercent = 0;
    bool BLEAvail;
    bool LTEAvail;
    bool XBeeAvail;
    bool GPSAvail;
    bool CompassAvail;
    uint8_t driveMode = 0;
    bool signal = false;
    bool lowBatt = false;
    bool warnedLowBatt = false;
    bool dataRecording = true;
    bool offloading = false;
    float TargetLat = -999.0;
    float TargetLon = -999.0;
    float GPSLat = 0.0;
    float GPSLon= 0.0;
    float pH = 0.0;
    float temp = 0.0;
    float DO = 0.0;
    float Cond = 0.0;
    float MCond = 0.0;
    uint32_t publishTime = 0;
    uint32_t timeoutCount;
};

class PairBot{
    public:
    uint8_t botNum;
    int rssi;
};

class MenuItem{
    public:
        std::vector<String> labels;
        void init(uint8_t inStep, uint8_t minV, uint8_t maxV, bool switchOnOff, const char * itemString){
            minVal = minV;
            maxVal = maxV;
            stepSize = inStep;
            onOffSetting = switchOnOff;
            strcpy(itemName,itemString);
        }
        uint8_t (WaterBot::*MethodPointer);
        bool (WaterBot::*MethodPointerBool);
        uint8_t stepSize;
        bool onOffSetting = false;
        bool customLabel = false;
        bool statOnly = false;
        uint8_t minVal;
        uint8_t maxVal;
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

WaterBot *BLEBot;   //Waterbot that is currently connected to over BLE
WaterBot *ControlledBot;
std::vector<WaterBot> WaterBots;
std::vector<WaterBot> PairBots;
std::vector<PairBot> BLEPair;
std::vector<MenuPopUp> PopUps;

Timer at1(5000,actionTimer5);
Timer at2(60000,actionTimer60);

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
                if(WaterBots.size() == 0 && BLETimeout == LTE_BKP_Time) Particle.publish("Bot1dat", "CCABhwd", PRIVATE);
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
    
    attachInterrupt(E_DPAD,entHandler,RISING);
    attachInterrupt(U_DPAD,uHandler,RISING);
    attachInterrupt(D_DPAD,dHandler,RISING);
    attachInterrupt(L_DPAD,lHandler,RISING);
    attachInterrupt(R_DPAD,rHandler,RISING);
    attachInterrupt(JOY_BTN,jHandler,RISING);

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

    char timestamp[16];
    snprintf(timestamp,16,"%02d%02d%04d%02d%02d%02d",Time.month(),Time.day(),Time.year(),Time.hour(),Time.minute(),Time.second());
    strcpy(filenameMessages,DEF_FILENAME);
    strcat(filenameMessages,timestamp);
    strcat(filenameMessages,"_LOG.txt");

    createMenu();

    delay(250);

    //oled.setup(); 
    oled.begin(i2c_Address, true); // Address 0x3C default
    oled.clearDisplay();
    oled.display();
    oled.setRotation(1);

    /*BleAdvertisingData advData;                 //Advertising data
    BLE.addCharacteristic(txCharacteristic);    //Add BLE Characteristics for BLE serial
    BLE.addCharacteristic(rxCharacteristic);
    advData.appendServiceUUID(RemoteService); // Add the app service
    advData.appendLocalName("RemoteTest");           //Local advertising name
    BLE.advertise(&advData);                    //Start advertising the characteristics*/

    
    
    oled.setTextSize(2);
    oled.setTextColor(SH110X_WHITE);
    oled.setCursor(0,0);
    oled.print(" Starting ");
    oled.display();

    delay(100);

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
    //delay(10000);
    PopUps.push_back(m);
    
    //startupPair();
    //delay(3000);

    at1.start();
    at2.start();

    WaterBotSim(1);

    
}

void loop() {
    if(postStatus){
        char statusStr[30];
        if(ControlledBot != NULL) sprintf(statusStr,"CCABspcB%1d",ControlledBot->botNum);
        else sprintf(statusStr,"CCABspcNB");
        sendData(statusStr,0,true,true,statusTimeout);                                  
        postStatus = false;
        statusTimeout = false;
    }
    updateMenu();
    updateBotControl();
    if((millis() - rcTime) > MTR_UPDATE_TIME){
        manualMotorControl(botSelect);
        rcTime = millis();
    }
    if (BLE.connected()) {
        
    }
    else {
    	if (millis() - lastScan >= SCAN_PERIOD_MS) {
    		lastScan = millis();
    		BLEScan(-1);
    	}

    }
    for(WaterBot &wb: WaterBots){
        if(wb.offloading){
            DataOffloader(wb.botNum);
            wb.offloading = false;
        }
    }
    XBeeHandler();
    RPiHandler();
    XBeeLTEPairSet();
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
                sendData(statusStr,0,true,true,statusTimeout);
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
        sendData(statusStr,0,true,true,statusTimeout);
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
        bool newBot = true;
        WaterBot *TargetWB;
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
            Serial.printlnf("String Len: %d, Checksum: %d",strlen(command)-2,checksum);
            logMessage("[WARN] Warning, checksum does not match!");
            Serial.println("Warning, checksum does not match");
            if((command[1] >= '0' && command[1] <= '9') || command[1] == 'C'){
                char rxBotNum[2];
                rxBotNum[0] = command[0];
                rxBotNum[1] = command[1];
                sprintf(errBuf,"CC%2snak%3s",rxBotNum,cmdStr);
                errModeReply = mode;
            }
            
            return;
        }
        if(!strcmp(cmdStr,"ack")){  //Acknowledgement for XBee and BLE
            if(mode == 1){  //Acknowledge from XBee

            }
            else if(mode == 2){ //Acknowledge from BLE
                
            }
            return;
        }
        else if(!strcmp(cmdStr,"sup")){
            for(WaterBot &w: WaterBots){
                if(rxBotID == w.botNum){
                    uint8_t battpct;
                    uint8_t statflags;
                    float latRX;
                    float lonRX;
                    char testLat[12];
                    char testLon[12];
                    sscanf(dataStr,"%u %u %s %s",&battpct,&statflags,testLat,testLon);
                    latRX = atof(testLat);
                    lonRX = atof(testLon);
                    w.battPercent = battpct;
                    w.LTEAvail = statflags & 1;
                    w.XBeeAvail = (statflags >> 1) & 1;
                    w.BLEAvail = (statflags >> 2) & 1;
                    w.lowBatt = (statflags >> 6) & 1;
                    w.GPSAvail = (statflags >> 8) & 1;
                    w.CompassAvail = (statflags >> 9) & 1;
                    w.GPSLat = latRX;
                    w.GPSLon = lonRX;
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
                    if(botSelect = w.botNum) redrawMenu = true;
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
            Serial.printlnf("Bot #: %d Temp: %f", TargetWB->botNum,TargetWB->temp);
        }
        else if(!strcmp(cmdStr,"nak")){  //Acknowledgement for XBee and BLE
            strncpy(errCmdStr,dataStr,3);
            errCmdMode = mode;
        }
        else if(!strcmp(cmdStr,"hwd")){  //Hello World! - Received startup pairing message
            bool newBot = true;
            for(WaterBot w: WaterBots){
                if(rxBotID == w.botNum) newBot = false;
            }
            if(newBot){
                Serial.println("Found a new water bot ID");
                WaterBot newWaterbot;
                if(mode == 1) newWaterbot.BLEAvail = true;
                else if(mode == 2) newWaterbot.XBeeAvail = true;
                else if(mode == 3) newWaterbot.LTEAvail = true;
                newWaterbot.botNum = rxBotID;
                WaterBots.push_back(newWaterbot);
                PairBots.push_back(newWaterbot);
                redrawMenu = true;
            }
            botPairRx = true;
        }
        else if(!strcmp(cmdStr,"pts")){
            if(!logFile.isOpen()){
                logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
                logFile.printlnf("[PUTS] Received String Command: %s",dataStr);
                logFile.close();
            }
            else logFile.printlnf("[PUTS] Received String Command: %s",dataStr);
        }
        else{   //Didn't recognize command (may be corrupted?) send back error signal
            if(mode == 1){

            }
            if(mode == 2){

            }
        }

        if(sendAck){    //Transmit out acknowledgement if needed

        }

    }
}

void processRPiCommand(const char *command, uint8_t mode){
    if(command[0] == 'R' && command[1] == 'P'){  
        Serial.println("Received Pi command");
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
        Serial.printlnf("Checksum: %02x, %03d, Checkstr: %s",checksum,checksum,command);
        for(uint8_t i = 4; i < strlen(command)-2;i++){
            if(i < 7) cmdStr[i-4] = command[i];
            else dataStr[i-7] = command[i];
        }
        if(checksum != strlen(command)-2){
            Serial.printlnf("String Len: %d, Checksum: %d",strlen(command)-2,checksum);
            if(!logFile.isOpen()){
                logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
                logFile.printlnf("[WARN] RPi Message Checksum Does Not Match!: %s",command);
                logFile.close();
            }
            else logFile.printlnf("[WARN] RPi Message Checksum Does Not Match!: %s",command);
            Serial.println("Warning, checksum does not match");
            if((command[1] >= '0' && command[1] <= '9') || command[1] == 'C'){
                char rxBotNum[2];
                rxBotNum[0] = command[0];
                rxBotNum[1] = command[1];
                sprintf(errBuf,"CC%2snak%3s",rxBotNum,cmdStr);
                errModeReply = mode;
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
            Serial.printlnf("Got a command packet from Pi for Bot %d",targetBot);
            for(WaterBot &wb: WaterBots){
                if(wb.botNum == targetBot){
                    wb.TargetLat = atof(GPSLatstr);
                    wb.TargetLon = atof(GPSLonstr);
                    wb.driveMode = drivemode;
                    wb.offloading = offloading;
                    wb.dataRecording = recording;
                    wb.signal = signal;
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
                    Serial.printlnf("Found Bot #: %d %d %d %d %d %d %d %d, services: %d",BLECustomData[0],BLECustomData[1],BLECustomData[2],BLECustomData[3],BLECustomData[4],BLECustomData[5],BLECustomData[6],BLECustomData[7], svcCount);
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
                        Serial.printlnf("Found new bot: %d", BLECustomData[0],BLEPair.size());
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
						Serial.printlnf("Connected to Bot %d",bufName[0]);
                        bool newBot = true;
                        WaterBot newWaterbot;
                        newWaterbot.BLEAvail = true;
                        newWaterbot.botNum = bufName[0];
                        PairBots.push_back(newWaterbot);
                        for(WaterBot &w: WaterBots){
                            if(bufName[0] == w.botNum){
                                newBot = false;
                                w.BLEAvail = true;
                                BLEBot = &w;
                            }
                        }
                        if(newBot){
                            Serial.println("Found a new water bot ID");
                            WaterBots.push_back(newWaterbot);
                            BLEBot = &WaterBots.back();
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
        Serial.println("Error, could not open root directory on SD Card. Is it inserted?");
        return;
    }
    uint32_t startScanTime = millis();
    Serial.printlnf("Requested SD Card Data from Bot %d Over BLE\n",BLEBot->botNum);
    if(BLEBot->botNum != OffloadingBot){
        Serial.printlnf("Currently connected to Bot %s, need to connect to Bot %d",BLEBot->botNum,OffloadingBot);
        BLE.disconnect();
        while(!BLE.connected() && startScanTime - millis() < 15000){
            BLEScan(OffloadingBot);
            delay(50);
        }
        char OffloadCommand[10];
        snprintf(OffloadCommand,10,"CCB%ddmp",OffloadingBot);
        sendData(OffloadCommand,0,true,false,false);
        if(BLE.connected()) Serial.printlnf("Successfully connected to Bot %d", BLEBot->botNum);
    }
    else{
        char OffloadCommand[10];
        snprintf(OffloadCommand,10,"CCB%ddmp",OffloadingBot);
        sendData(OffloadCommand,0,true,false,false);
    }
    Serial.printlnf("Starting file transfer from Bot %d",BLEBot->botNum);
    if(startScanTime - millis() > 15000){
        if(logDir.isOpen()) logDir.close();
        return;
    }
    offloadingDone = false;
    while(!offloadingDone) delay(100);
    Serial.printlnf("Finished transferring file from Bot %d",BLEBot->botNum);
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
            Serial.printlnf("CCRPsupB%d %d %0.6f %0,6f %d",wb.botNum, wb.battPercent, wb.GPSLat, wb.GPSLon, statusFlags);
        }
    }
}

void RPiHandler(){
    while(Serial.available()){
            String data = Serial.readStringUntil('\n');
            Serial.println(data);
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
        processCommand(buffer,2,true);
        Serial.println("New XBee Command:");
        Serial.println(data); 
        if(logMessages){
            if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
            logFile.printlnf("[INFO] Received XBee Message: %s",data);
            logFile.close();
        }
    }
}

void manualMotorControl(uint8_t commandedBot){

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
    uint8_t LSpeed, RSpeed;
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
    
    
    for(WaterBot wb: WaterBots){
        if(wb.driveMode == 0){
            sprintf(mtrStr,"CCB%dmtr%03d%03d",commandedBot, LSpeed, RSpeed);
            Serial.println(mtrStr);
            sendData(mtrStr,0,true,false, false);
        }
    }
}

static void BLEDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    char btBuf[len+1];
    for (size_t ii = 0; ii < len; ii++) btBuf[ii] = data[ii];
    if(btBuf[len-1] != '\0') btBuf[len] = '\0';
    else btBuf[len-1] = '\0';
    Serial.print("New BT Command: ");
    Serial.println(btBuf);
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
                Serial.printlnf("File '%s' already exists, deleting and overwriting",offloadFilename);
                sd.remove(offloadFilename);
            }
            Serial.printlnf("Starting offload of file: %s",offloadFilename);
            myFile.open(offloadFilename, O_RDWR | O_CREAT | O_AT_END);
            return;
        }
        else if(!strcmp(fileCommand,"filecomp")){
            Serial.printlnf("Reached end of file: %s",offloadFilename);
            if(myFile.isOpen()) myFile.close();
            return;
        }
        else if(!strcmp(fileCommand,"filedone")){
            Serial.println("Received done command");
            offloadingDone = true;
            if(myFile.isOpen()) myFile.close();
            return;
        }
    }
    
    char dataStr[len];
    memcpy(dataStr,data,len);
    myFile.print(dataStr);
    Serial.println(dataStr);
}

void sendData(const char *dataOut, uint8_t sendMode, bool sendBLE, bool sendXBee, bool sendLTE){
    char outStr[strlen(dataOut)+2];
    sprintf(outStr,"%s%02x",dataOut,strlen(dataOut));
    #ifdef VERBOSE
    Serial.println(outStr);
    #endif
    if(sendLTE || sendMode == 4){
        Particle.publish("Bot1dat", outStr, PRIVATE);
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
        w.timeoutCount++;
    }
    //if(!BLE.connected)
}

void actionTimer60(){
    bool reqLTEStatus = false;
    for(WaterBot &w: WaterBots){
        if(w.timeoutCount > XBEE_BLE_MAX_TIMEOUT){
            reqLTEStatus = true;
            w.timeoutCount = 0;            
        }
    }
    if(reqLTEStatus && LTEStatuses < MAX_LTE_STATUSES){
        LTEStatuses++;
        statusTimeout = true;
    }
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

    MenuItems.push_back(dataRecord);
    MenuItems.push_back(battStat);
    MenuItems.push_back(sentryToggle);
    MenuItems.push_back(offloadItem);
    MenuItems.push_back(signalToggle);

    SelectedItem = &MenuItems.at(menuItem);
}

void entHandler(){
    redrawMenu = true;  
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    Serial.println("Enter trigger");
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
    Serial.println("Right trigger");
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
                Serial.println(curItem->itemName);
                if(curItem == nullptr) return;
                if(curItem->statOnly) return;
                if(curItem->onOffSetting){
                    Serial.println("Modified an On/Off Control");
                    ws.*(curItem->MethodPointerBool) = true;
                    Serial.printlnf("Bot: %d, Modified ",ws.botNum);
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

void lHandler(){
    redrawMenu = true;  
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    Serial.println("Right trigger");
    debounceTime = millis();
    redrawMenu = true;
    if(selectingBots){
        if(botSelect != WaterBots.front().botNum){
            uint8_t newBotNum = WaterBots.front().botNum;
            for(WaterBot &ws: WaterBots){
                if(ws.botNum == botSelect){
                    botSelect = newBotNum;
                    ControlledBot = &ws;
                }
                else newBotNum = ws.botNum;
            }
               
        }
    }
    else{
        for(WaterBot &ws: WaterBots){
            if(ws.botNum == botSelect){
                MenuItem *curItem = SelectedItem;
                Serial.println(curItem->itemName);
                if(curItem == nullptr) return;
                if(curItem->statOnly) return;
                if(curItem->onOffSetting){
                    Serial.println("Modified an On/Off Control");
                    ws.*(curItem->MethodPointerBool) = false;
                    Serial.printlnf("Bot: %d, Modified ",ws.botNum);
                }
                else{
                    
                    if(ws.*(curItem->MethodPointer) > curItem->minVal) ws.*(curItem->MethodPointer) -= curItem->stepSize;
                }
                modifiedValue = true;
                ws.updatedControl = true;
            }
        }
    }
}

void uHandler(){
    redrawMenu = true;  
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    debounceTime = millis();
    if(menuItem) menuItem--;
    SelectedItem = &MenuItems.at(menuItem);
    Serial.println("Up trigger");
}

void dHandler(){
    redrawMenu = true;  
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    debounceTime = millis();
    if(menuItem < MAX_MENU_ITEMS-1) menuItem++;
    SelectedItem = &MenuItems.at(menuItem);
    Serial.println("Down trigger");
}

void jHandler(){
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    debounceTime = millis();
    Serial.println("Joystick trigger");
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