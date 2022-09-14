/*
 * Project WaterNet23CCHub
 * Description: Code for the Central Control hub responsible for orchestrating commands to Water Bots
 * Author:
 * Date:
 */

//https://www.digikey.com/en/products/detail/sparkfun-electronics/COM-09032/6823623

#include "application.h"
#include "SdFat.h"
#undef min
#undef max
#include <vector>

#define chipSelect D8

#define DEF_FILENAME            "WaterBot"
#define BLE_OFFLD_BUF           100
#define CUSTOM_DATA_LEN         8
#define MAX_FILENAME_LEN        30
#define MAX_ERR_BUF_SIZE        15              //Buffer size for error-return string
#define XBEE_BLE_MAX_TIMEOUT    36
#define BLE_MAX_CONN_TIME       200             //20 second max time to successfully pair to bot
#define MAX_LTE_STATUSES        25
#define LTE_BKP_Time            100             //Send LTE request after 10 seconds if not connected to any bots
#define PAIR_BUTTON             A3
#define OFFLOAD_BTN             A2
#define JOYH_ADC                A0              //Horizontal Joystick
#define JOYV_ADC                A1              //Vertical Joystick ADC pin


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

class WaterBot{
    public:
    uint8_t botNum;
    uint8_t battPercent;
    bool BLEAvail;
    bool LTEAvail;
    bool XBeeAvail;
    bool manualRC;
    bool lowBatt;
    bool dataRecording;
    bool offloading;
    float GPSLat;
    float GPSLon;
    uint32_t timeoutCount;
};

class PairBot{
    public:
    uint8_t botNum;
    int rssi;
};

WaterBot *BLEBot;   //Waterbot that is currently connected to over BLE
WaterBot *ControlledBot;
std::vector<WaterBot> WaterBots;
std::vector<WaterBot> PairBots;
std::vector<PairBot> BLEPair;

Timer at1(5000,actionTimer5);
Timer at2(60000,actionTimer60);


void BLEScan(int BotNumber = -1);
void XBeeHandler();
void dataLTEHandler(const char *event, const char *data);

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
    while(!startConnect){
        BLEScan(-2);
        XBeeHandler();
        XBeeLTEPairSet();
        delay(100);
        if(digitalRead(PAIR_BUTTON)){
            int minRSSI = -999;
            int selectedBot = -1;
            for(PairBot pb: BLEPair){
                if(pb.rssi > minRSSI){
                    minRSSI = pb.rssi;
                    selectedBot = pb.botNum;
                }
                
            }
            if(selectedBot > 0){
                meshPair = true;    //Did we find any bots over BLE
            }
            uint8_t BLETimeout = 0;
            while(meshPair){
                BLEScan(selectedBot);
                BLETimeout++;
                if(WaterBots.size() == 0 && BLETimeout == LTE_BKP_Time) Particle.publish("Bot1dat", "CCABhwd", PRIVATE);
                if(BLETimeout > BLE_MAX_CONN_TIME) meshPair = false;
                delay(100);
            }
        }
    }
}

void XBeeLTEPairSet(){
    for(WaterBot p: PairBots){
        char replyStr[10];
        sprintf(replyStr,"CCB%dhwa",p.botNum);
        sendData(replyStr,0,false,p.XBeeAvail,p.LTEAvail);
        PairBots.pop_back();
    }
}

void setup() {

    pinMode(PAIR_BUTTON,INPUT_PULLDOWN);
    pinMode(OFFLOAD_BTN,INPUT_PULLDOWN);
    pinMode(D7, OUTPUT);

    Serial.begin(115200);
    Serial1.begin(9600);                        //Start serial for XBee module
    setupXBee();

	BLE.on();

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

    /*BleAdvertisingData advData;                 //Advertising data
    BLE.addCharacteristic(txCharacteristic);    //Add BLE Characteristics for BLE serial
    BLE.addCharacteristic(rxCharacteristic);
    advData.appendServiceUUID(RemoteService); // Add the app service
    advData.appendLocalName("RemoteTest");           //Local advertising name
    BLE.advertise(&advData);                    //Start advertising the characteristics*/
    if (!sd.begin(chipSelect, SD_SCK_MHZ(4))) {
        Serial.println("Error: could not connect to SD card!");
        logMessages = false;
    }
    
    //startupPair();

    at1.start();
    at2.start();
}

void loop() {
    if(postStatus){
        char statusStr[30];
        if(ControlledBot != NULL) sprintf(statusStr,"CCABspcB%d",ControlledBot->botNum);
        else sprintf(statusStr,"CCABspcNB");
        sendData(statusStr,0,true,true,statusTimeout);                                  
        postStatus = false;
        statusTimeout = false;
    }


    if (BLE.connected()) {
        //if(BLEBot) Serial.printlnf("Connected to Waterbot %d", BLEBot->botNum);
        //char testStr[30] = "CCB1ptsHello from CC Hub!";
        //uint8_t testBuf[30];
        //memcpy(testStr,testBuf,30);
        //peerRxCharacteristic.setValue(testStr);
        //sendData("CCB1ptsbigbot",0,true,false,false);
        if(digitalRead(OFFLOAD_BTN)) offloadingMode = true;

        char sendStr[18];
        
        sprintf(sendStr,"CCB1mtr%03d%03d",(int)(analogRead(JOYV_ADC)/22.75)%1000,(int)(analogRead(JOYV_ADC)/22.75)%1000);
        Serial.printlnf("Motor Speed: %03d",(int)(analogRead(JOYV_ADC)/22.75));
        Serial.println(sendStr);
        sendData(sendStr,0,true,false,false);
        digitalWrite(D7,HIGH);
        delay(250);
    }
    else {
        digitalWrite(D7,LOW);
    	if (millis() - lastScan >= SCAN_PERIOD_MS) {
    		// Time to scan
    		lastScan = millis();
    		BLEScan(-1);
    	}

    }
    //if(!logMessages) Serial.println("DANGER SD CARD NOT WORKING");
    if(offloadingMode) DataOffloader();
    XBeeHandler();
    XBeeLTEPairSet();
}

void processCommand(const char *command, uint8_t mode, bool sendAck){
    //Process if command is addressed to this bot "Bx" or all bots "AB"
    if((command[2] == 'A' && command[3] == 'B') || (command[2] == 'C' && command[3] == 'C')){
        uint8_t checksum;
        char dataStr[strlen(command)-8];
        dataStr[strlen(command)-9] = '\0';
        char rxIDBuf[1];
        rxIDBuf[0] = command[1];
        uint8_t rxBotID = atoi(rxIDBuf);
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
            
            bool newBot = true;
            for(WaterBot w: WaterBots){
                if(rxBotID == w.botNum){
                    newBot = false;
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
                    w.manualRC = (statflags >> 4) & 1;
                    w.lowBatt = (statflags >> 5) & 1;
                    w.dataRecording = (statflags >> 6) & 1;
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
            if(newBot){
                Serial.println("Found a new water bot ID");
                WaterBot newWaterbot;
                newWaterbot.BLEAvail = true;
                newWaterbot.botNum = rxBotID;
                newBot = false;
                uint8_t battpct;
                uint8_t statflags;
                float latRX;
                float lonRX;
                sscanf(dataStr,"%u %u %f %f",&battpct,&statflags,&latRX,&lonRX);
                newWaterbot.battPercent = battpct;
                newWaterbot.LTEAvail = statflags & 1;
                newWaterbot.XBeeAvail = (statflags >> 1) & 1;
                newWaterbot.BLEAvail = (statflags >> 2) & 1;
                newWaterbot.offloading = (statflags >> 3) & 1;
                newWaterbot.manualRC = (statflags >> 4) & 1;
                newWaterbot.lowBatt = (statflags >> 5) & 1;
                newWaterbot.dataRecording = (statflags >> 6) & 1;
                newWaterbot.GPSLat = latRX;
                newWaterbot.GPSLon = lonRX;
                WaterBots.push_back(newWaterbot);
            }
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
            uint8_t BLECustomData[CUSTOM_DATA_LEN];
            scanResults->advertisingData().customData(BLECustomData,CUSTOM_DATA_LEN);
            if (svcCount > 0 && foundServiceUuid == serviceUuid) {
                if(BotNumber == -2){
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
                        NewBot.rssi = scanResults->rssi();
                        NewBot.botNum = BLECustomData[0];
                        BLEPair.push_back(NewBot);
                    }
                    else{
                        existingBot->rssi = (scanResults->rssi() + existingBot->rssi) >> 1;
                    }
                }
                if(BotNumber == -1 || BotNumber == BLECustomData[0]){   //Check if a particular bot number was specified
                    peer = BLE.connect(scanResults[ii].address());
				    if (peer.connected()) {
                        meshPair = false;
                        uint8_t bufName[BLE_MAX_ADV_DATA_LEN];
                        scanResults[ii].advertisingData().customData(bufName, BLE_MAX_ADV_DATA_LEN);
					    peer.getCharacteristicByUUID(peerTxCharacteristic, txUuid);
					    peer.getCharacteristicByUUID(peerRxCharacteristic, rxUuid);
                        peer.getCharacteristicByUUID(peerOffloadCharacteristic, offldUuid);
						Serial.printlnf("Connected to Bot %d",bufName[0]);
                        bool newBot = true;
                        for(WaterBot w: WaterBots){
                            if(bufName[0] == w.botNum){
                                newBot = false;
                                w.BLEAvail = true;
                            }
                        }
                        if(newBot){
                            Serial.println("Found a new water bot ID");
                            WaterBot newWaterbot;
                            newWaterbot.BLEAvail = true;
                            newWaterbot.botNum = bufName[0];
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

void DataOffloader(){
    uint8_t OffloadingBot = 1;
    if (!logDir.open("/")) {
        offloadingDone = true;
        Serial.println("Error, could not open root directory on SD Card. Is it inserted?");
        return;
    }
    while(OffloadingBot <= WaterBots.size()){
        char OffloadCommand[10];
        uint8_t OffloadBuf[10];
        snprintf(OffloadCommand,10,"CCB%ddmp",OffloadingBot);
        //Particle.publish("CCHub", OffloadCommand, PRIVATE);
        memcpy(OffloadBuf,OffloadCommand,10);
        peerRxCharacteristic.setValue(OffloadBuf,10);
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
    }
    if(logDir.isOpen()) logDir.close();
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