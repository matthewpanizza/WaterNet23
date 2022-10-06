/*
 * Project WaterNet23CCHub
 * Description: Code for the Central Control hub responsible for orchestrating commands to Water Bots
 * Author:
 * Date:
 */


#include "application.h"
#include "SdFat.h"
#undef min
#undef max
#include <vector>
#include "oled-wing-adafruit.h"

// Pin Definitions

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
#define XBEE_BLE_MAX_TIMEOUT    36
#define BLE_MAX_CONN_TIME       200             //20 second max time to successfully pair to bot
#define MAX_LTE_STATUSES        25
#define LTE_BKP_Time            100             //Send LTE request after 10 seconds if not connected to any bot

//Menu Parameters
#define MAX_MENU_ITEMS          4
#define DEBOUNCE_MS             100
#define OLED_MAX_X              128
#define OLED MAX_Y              32

//Development Parameters
#define VERBOSE 1

// This example does not require the cloud so you can run it in manual mode or
// normal cloud-connected mode
SYSTEM_MODE(MANUAL);

// These UUIDs were defined by Nordic Semiconductor and are now the defacto standard for
// UART-like services over BLE. Many apps support the UUIDs now, like the Adafruit Bluefruit app.
const BleUuid serviceUuid("b4206910-dc4b-5743-c8b1-92d0e75182b0");
const BleUuid rxUuid("b4206912-dc4b-5743-c8b1-92d0e75182b0");
const BleUuid txUuid("b4206913-dc4b-5743-c8b1-92d0e75182b0");
const BleUuid offldUuid("b4206914-dc4b-5743-c8b1-92d0e75182b0");


//const BleUuid RemoteService("b4206920-dc4b-5743-c8b1-92d0e75182b0");
//const BleUuid rxUuid2("b4206922-dc4b-5743-c8b1-92d0e75182b0");
//const BleUuid txUuid2("b4206923-dc4b-5743-c8b1-92d0e75182b0");

//BleCharacteristic txCharacteristic("txr", BleCharacteristicProperty::NOTIFY, txUuid, RemoteService);
//BleCharacteristic rxCharacteristic("rxr", BleCharacteristicProperty::WRITE_WO_RSP, rxUuid, RemoteService, BLEDataReceived, NULL);

const size_t UART_TX_BUF_SIZE = 30;
const size_t SCAN_RESULT_COUNT = 20;

BleScanResult scanResults[SCAN_RESULT_COUNT];

BleCharacteristic peerTxCharacteristic;
BleCharacteristic peerRxCharacteristic;
BleCharacteristic peerOffloadCharacteristic;
BlePeerDevice peer;

//OLED Object
OledWingAdafruit oled;

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
    uint8_t botNum;
    uint8_t battPercent;
    bool BLEAvail;
    bool LTEAvail;
    bool XBeeAvail;
    uint8_t driveMode;
    bool lowBatt;
    bool dataRecording;
    bool offloading = false;
    float TargetLat = -999;
    float TargetLon = -999;
    float GPSLat = 0.0;
    float GPSLon= 0.0;
    float pH = 0.0;
    float temp = 0.0;
    float DO = 0.0;
    float Cond = 0.0;
    float MCond = 0.0;
    uint32_t timeoutCount;
};

class PairBot{
    public:
    uint8_t botNum;
    int rssi;
};

class MenuItem{
    public:
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
        bool onOffSetting;
        bool statOnly = false;
        uint8_t minVal;
        uint8_t maxVal;
        char itemName[10];
};

WaterBot *BLEBot;   //Waterbot that is currently connected to over BLE
WaterBot *ControlledBot;
std::vector<WaterBot> WaterBots;
std::vector<WaterBot> PairBots;
std::vector<PairBot> BLEPair;

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
    oled.fillCircle(115,7,3,WHITE);
    oled.display();
    uint8_t loadAnim = 0;
    while(!startConnect){
        oled.setCursor(0,16);
        oled.fillRect(72,16,35,15,0);
        oled.printlnf("Bots: %d",BLEPair.size());
        Serial.printlnf("Array size: %d",BLEPair.size());
        oled.drawCircle(115,7,loadAnim,WHITE);
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
                    Serial.println("Found a local bot");
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

    pinMode(E_DPAD,INPUT_PULLUP);
    pinMode(U_DPAD,INPUT_PULLUP);
    pinMode(D_DPAD,INPUT_PULLUP);
    pinMode(L_DPAD,INPUT_PULLUP);
    pinMode(R_DPAD,INPUT_PULLUP);
    pinMode(JOY_BTN,INPUT_PULLUP);
    
    attachInterrupt(E_DPAD,entHandler,FALLING);
    attachInterrupt(U_DPAD,uHandler,FALLING);
    attachInterrupt(D_DPAD,dHandler,FALLING);
    attachInterrupt(L_DPAD,lHandler,FALLING);
    attachInterrupt(R_DPAD,rHandler,FALLING);
    attachInterrupt(JOY_BTN,jHandler,FALLING);

    debounceTime = millis();

    Serial.begin(115200);
    Serial1.begin(9600);                        //Start serial for XBee module
    setupXBee();

	BLE.on();
    BLE.setScanTimeout(50);  //100ms scan
    BLE.setTxPower(8);

    peerTxCharacteristic.onDataReceived(BLEDataReceived, &peerTxCharacteristic);
    peerOffloadCharacteristic.onDataReceived(offloadDataReceived, &peerOffloadCharacteristic);

    Particle.subscribe("Bot1dat",dataLTEHandler);

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

    oled.setup(); 
    oled.clearDisplay();
    oled.display();

    /*BleAdvertisingData advData;                 //Advertising data
    BLE.addCharacteristic(txCharacteristic);    //Add BLE Characteristics for BLE serial
    BLE.addCharacteristic(rxCharacteristic);
    advData.appendServiceUUID(RemoteService); // Add the app service
    advData.appendLocalName("RemoteTest");           //Local advertising name
    BLE.advertise(&advData);                    //Start advertising the characteristics*/

    if (!sd.begin(chipSelect, SD_SCK_MHZ(8))) {
        Serial.println("Error: could not connect to SD card!");
        logMessages = false;
    }
    oled.setTextSize(2);
    oled.setTextColor(WHITE);
    oled.setCursor(0,0);
    oled.print(" Starting ");
    oled.display();
    
    //startupPair();
    delay(3000);

    at1.start();
    at2.start();

    WaterBotSim(2);
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
    //Serial.printlnf("Selected Bot: %d ",botSelect);

    if(!logMessages) Serial.println("Error, SD Card Not working");
    if(updateControl){
        updateControl = false;
        ControlledBot = NULL;
        for(uint8_t i = 0; i < WaterBots.size(); i++){
            if(WaterBots.at(i).botNum == botSelect) ControlledBot =  &WaterBots.at(i);
        }
        if(ControlledBot == NULL) return;
        if(ControlledBot->offloading) offloadingMode = true;
        char statusStr[30];
        sprintf(statusStr,"CCB%dcnf%1d",ControlledBot->botNum,int(ControlledBot->dataRecording));
        sendData(statusStr,0,true,true,statusTimeout);
    }

    if (BLE.connected()) {
        //if(BLEBot) Serial.printlnf("Connected to Waterbot %d", BLEBot->botNum);
        //char testStr[30] = "CCB1ptsHello from CC Hub!";
        //uint8_t testBuf[30];
        //memcpy(testStr,testBuf,30);
        //peerRxCharacteristic.setValue(testStr);
        //sendData("CCB1ptsbigbot",0,true,false,false);
        if(!digitalRead(D_DPAD)) sendData("CCB1req",0,true,false,false);//offloadingMode = true;

        for(WaterBot ws: WaterBots) Serial.printlnf("Temp: %0.6f",ws.temp);

        //char sendStr[18];
        //sprintf(sendStr,"CCB1mtr%03d%03d",(int)(analogRead(JOYV_ADC)/22.75)%1000,(int)(analogRead(JOYV_ADC)/22.75)%1000);
        //Serial.printlnf("Motor Speed: %03d",(int)(analogRead(JOYV_ADC)/22.75));
        //Serial.println(sendStr);
        //sendData(sendStr,0,true,false,false);
        //digitalWrite(D7,HIGH);
        delay(250);
    }
    else {
        //digitalWrite(D7,LOW);
    	if (millis() - lastScan >= SCAN_PERIOD_MS) {
    		// Time to scan
    		lastScan = millis();
    		BLEScan(-1);
    	}

    }
    if(offloadingMode){
        DataOffloader(ControlledBot->botNum);
        ControlledBot->offloading = false;
    }
    XBeeHandler();
    XBeeLTEPairSet();
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
            else oled.printf("%d",wb.*(MenuItems.at(id).MethodPointer));
        }
        Serial.printlnf("Printed Highlighted Menu item with name: %s",MenuItems.at(id).itemName);
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
        else oled.printf("%d",wb.*(MenuItems.at(id).MethodPointer));
    }
    
    
    
    MenuItems.at(id);
}

void updateMenu(){
    if(redrawMenu){
        oled.fillRect(0,0,OLED_MAX_X,15,0);
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
            Serial.println("Menu item 0");
            if(MenuItems.size()) printMenuItem(0,true,!selectingBots,0,16,WaterBots.at(menuSelect));
            uint8_t loopIter = MenuItems.size();
            if(loopIter > 2) loopIter = 2;
            for(int mi = 1; mi <= loopIter; mi++){
                Serial.printlnf("Menu item %d", mi);
                printMenuItem(mi,false,!selectingBots,0,16+(16*mi),WaterBots.at(menuSelect));
            }
        }
        else if(menuItem == MAX_MENU_ITEMS-1){
            Serial.printlnf("Menu item %d", menuItem);
            printMenuItem(menuItem,true,!selectingBots,0,48,WaterBots.at(menuSelect));
            Serial.printlnf("Menu item %d", menuItem-1);
            printMenuItem(menuItem-1,false,!selectingBots,0,32,WaterBots.at(menuSelect));
            Serial.printlnf("Menu item %d", menuItem-2);
            printMenuItem(menuItem-2,false,!selectingBots,0,16,WaterBots.at(menuSelect));
        }
        else{
            Serial.printlnf("Menu item %d", menuItem+1);
            printMenuItem(menuItem+1,false,!selectingBots,0,48,WaterBots.at(menuSelect));
            Serial.printlnf("Menu item %d", menuItem);
            printMenuItem(menuItem,true,!selectingBots,0,32,WaterBots.at(menuSelect));
            Serial.printlnf("Menu item %d", menuItem-1);
            printMenuItem(menuItem-1,false,!selectingBots,0,16,WaterBots.at(menuSelect));
        }
        oled.display();
        redrawMenu = false;
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
        Serial.printlnf("Checksum: %02x, %03d",checksum,checksum);
        for(uint8_t i = 4; i < strlen(command)-2;i++){
            if(i < 7) cmdStr[i-4] = command[i];
            else dataStr[i-7] = command[i];
        }
        if(checksum != strlen(command)-2){
            Serial.printlnf("String Len: %d, Checksum: %d",strlen(command)-2,checksum);
            if(!logFile.isOpen()){
                logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
                logFile.printlnf("[WARN] Message Checksum Does Not Match!: %s",command);
                logFile.close();
            }
            else logFile.printlnf("[WARN] Message Checksum Does Not Match!: %s",command);
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
            for(WaterBot w: WaterBots){
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
                    w.offloading = (statflags >> 3) & 1;
                    w.driveMode = (statflags >> 4) & 3;
                    w.lowBatt = (statflags >> 6) & 1;
                    w.dataRecording = (statflags >> 7) & 1;
                    w.GPSLat = latRX;
                    w.GPSLon = lonRX;
                    Serial.println("Status Update!");
                    Serial.println("##########################");
                    Serial.println("##     STATUS UPDATE    ##");
                    Serial.printlnf("##       Bot #: %1d      ##",w.botNum);
                    Serial.printlnf("##      Batt %: %03d     ##",w.battPercent);
                    Serial.println("##    LTE  BLE  XBee    ##");
                    Serial.printlnf("##     %d    %d     %d     ##",w.LTEAvail,w.BLEAvail,w.XBeeAvail);
                    Serial.println("##  Latitude Longitude  ##");
                    Serial.printlnf("## %.6f %.6f ##",w.GPSLat,w.GPSLon);
                    Serial.println("##########################");
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
            }
            botPairRx = true;
        }
        else if(!strcmp(cmdStr,"pts")){
            Serial.println(dataStr);
            myFile.open("RawWrite.txt", O_RDWR | O_CREAT | O_AT_END);
            String modeStr[3] = {"LTE", "XBee", "Bluetooth"};
            myFile.printf("New string from %s: ", modeStr[mode]);
            myFile.println(dataStr);
            delay(5);
            myFile.close();
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
                        for(WaterBot w: WaterBots){
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
    //while(OffloadingBot <= WaterBots.size()){
        char OffloadCommand[10];
        uint8_t OffloadBuf[10];
        snprintf(OffloadCommand,10,"CCB%ddmp",OffloadingBot);
        //Particle.publish("CCHub", OffloadCommand, PRIVATE);
        memcpy(OffloadBuf,OffloadCommand,10);
        sendData(OffloadCommand,0,true,false,false);
        Serial.printlnf("Requested SD Card Data from Bot %d Over BLE\n",BLEBot->botNum);
        delay(1000);
        if(BLEBot->botNum != OffloadingBot){
            Serial.printlnf("Currently connected to Bot %f, need to connect to Bot %d",BLEBot->botNum,OffloadingBot);
            BLE.disconnect();
            while(!BLE.connected()){
                BLEScan(OffloadingBot);
                delay(50);
            }
            Serial.printlnf("Successfully connected to Bot %d", BLEBot->botNum);
        }
        Serial.printlnf("Starting file transfer from Bot %d",BLEBot->botNum);
        offloadingDone = false;
        while(!offloadingDone) delay(100);
        Serial.printlnf("Finished transferring file from Bot %d",BLEBot->botNum);
        OffloadingBot++;
    //}
    if(logDir.isOpen()) logDir.close();
    offloadingMode = false;
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
    if(VERBOSE) Serial.println(outStr);
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
    for(WaterBot w: WaterBots){
        w.timeoutCount++;
    }
    //if(!BLE.connected)
}

void actionTimer60(){
    bool reqLTEStatus = false;
    for(WaterBot w: WaterBots){
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
    sentryToggle.init(1,0,1,true,"Sentry");
    sentryToggle.MethodPointer = &WaterBot::driveMode;

    MenuItems.push_back(dataRecord);
    MenuItems.push_back(battStat);
    MenuItems.push_back(offloadItem);
    MenuItems.push_back(sentryToggle);

    SelectedItem = &MenuItems.at(menuItem);
}

void entHandler(){
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    Serial.println("Enter trigger");
    debounceTime = millis();
    
    redrawMenu = true;  
    selectingBots = !selectingBots;
    if(modifiedValue) updateControl = true;
}

void rHandler(){
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    debounceTime = millis();
    Serial.println("Right trigger");
    redrawMenu = true;  
    if(selectingBots){
        if(botSelect != WaterBots.back().botNum){
            bool findCurrent = false;
            for(WaterBot ws: WaterBots){
                if(findCurrent){
                    botSelect = ws.botNum;
                    break;
                }
                if(ws.botNum == botSelect) findCurrent = true;
            }
             
        }
    }
    else{
        int index = 0;
        for(WaterBot ws: WaterBots){
            if(ws.botNum == botSelect){
                MenuItem curItem = *SelectedItem;
                Serial.println(curItem.itemName);
                if(curItem.statOnly) return;
                if(curItem.onOffSetting){
                    Serial.println("Modified an On/Off Control");
                    WaterBots.at(index).*(curItem.MethodPointerBool) = true;
                    Serial.printlnf("Bot: %d, Modified ",WaterBots.at(index).botNum);
                }
                else{
                    if(WaterBots.at(index).*(curItem.MethodPointer) < curItem.maxVal) WaterBots.at(index).*(curItem.MethodPointer) += curItem.stepSize;
                }
                modifiedValue = true;
            }
            index++;
        }
    }
}

void lHandler(){
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    Serial.println("Right trigger");
    debounceTime = millis();
    redrawMenu = true;
    if(selectingBots){
        if(botSelect != WaterBots.front().botNum){
            uint8_t newBotNum = WaterBots.front().botNum;
            for(WaterBot ws: WaterBots){
                if(ws.botNum == botSelect) botSelect = newBotNum;
                else newBotNum = ws.botNum;
            }
               
        }
    }
    else{
        int index = 0;
        for(WaterBot ws: WaterBots){
            if(ws.botNum == botSelect){
                MenuItem curItem = *SelectedItem;
                Serial.println(curItem.itemName);
                if(curItem.statOnly) return;
                if(curItem.onOffSetting){
                    Serial.println("Modified an On/Off Control");
                    WaterBots.at(index).*(curItem.MethodPointerBool) = false;//!(WaterBots.at(index).*(curItem.MethodPointerBool));
                    Serial.printlnf("Bot: %d, Modified ",WaterBots.at(index).botNum);
                }
                else{
                    if(WaterBots.at(index).*(curItem.MethodPointer) > curItem.minVal) WaterBots.at(index).*(curItem.MethodPointer) -= curItem.stepSize;
                }
                modifiedValue = true;
            }
            index++;
        }
    }
}

void uHandler(){
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    debounceTime = millis();
    if(menuItem) menuItem--;
    SelectedItem = &MenuItems.at(menuItem);
    Serial.println("Up trigger");
    redrawMenu = true;  
}

void dHandler(){
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    debounceTime = millis();
    if(menuItem < MAX_MENU_ITEMS-1) menuItem++;
    SelectedItem = &MenuItems.at(menuItem);
    Serial.println("Down trigger");
    redrawMenu = true;  
}

void jHandler(){
    if(millis()-debounceTime < DEBOUNCE_MS) return;
    debounceTime = millis();
    Serial.println("Joystick trigger");
}