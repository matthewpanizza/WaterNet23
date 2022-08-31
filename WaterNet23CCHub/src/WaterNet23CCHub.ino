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

#define chipSelect D8

#define DEF_FILENAME        "WaterBot"
#define BLE_OFFLD_BUF       100
#define CUSTOM_DATA_LEN     8
#define MAX_FILENAME_LEN    30
#define MAX_ERR_BUF_SIZE    15              //Buffer size for error-return string


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

uint8_t txBuf[UART_TX_BUF_SIZE];
size_t txLen = 0;

const unsigned long SCAN_PERIOD_MS = 2000;
unsigned long lastScan = 0;
bool offloadingMode;
bool offloadingDone;
char offloadFilename[MAX_FILENAME_LEN];
char filenameMessages[MAX_FILENAME_LEN];
bool remoteRx = false;
bool logMessages;
uint8_t errCmdMode;
uint8_t errModeReply;
char errCmdStr[3];
char txBuf[UART_TX_BUF_SIZE];
char errBuf[MAX_ERR_BUF_SIZE];

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
    float GPSLat;
    float GPSLon;
};

WaterBot *BLEBot;   //Waterbot that is currently connected to over BLE
std::vector<WaterBot> WaterBots;

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

void setup() {

    Serial.begin(115200);
    pinMode(A0, INPUT_PULLDOWN);
    pinMode(D7, OUTPUT);
	BLE.on();
    peerTxCharacteristic.onDataReceived(BLEDataReceived, &peerTxCharacteristic);
    peerOffloadCharacteristic.onDataReceived(offloadDataReceived, &peerOffloadCharacteristic);

    Particle.subscribe("Bot1dat",dataLTEHandler);

    offloadingMode = false;
    offloadingDone = false;

    logMessages = true;

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
}

void loop() {
    
    if (BLE.connected()) {
        if(BLEBot) Serial.printlnf("Connected to Waterbot %d", BLEBot->botNum);
        char testStr[30] = "CCB1ptsHello from CC Hub!";
        //uint8_t testBuf[30];
        //memcpy(testStr,testBuf,30);
        peerRxCharacteristic.setValue(testStr);
        digitalWrite(D7,HIGH);
        delay(1000);
    }
    else {
        digitalWrite(D7,LOW);
    	if (millis() - lastScan >= SCAN_PERIOD_MS) {
    		// Time to scan
    		lastScan = millis();
    		BLEScan(-1);
    	}

    }
    if(offloadingMode) DataOffloader();
    XBeeHandler();
}

void processCommand(const char *command, uint8_t mode, bool sendAck){
    //Process if command is addressed to this bot "Bx" or all bots "AB"
    if((command[2] == 'A' && command[3] == 'B') || (command[2] == 'C' && command[3] == 'C')){
        uint8_t checksum;
        char dataStr[strlen(command)-9];
        char cmdStr[3];
        char checkStr[2];
        checkStr[0] = command[strlen(command)-1];
        checkStr[1] = command[strlen(command)-2];
        checksum = (uint8_t)strtol(checkStr, NULL, 16);       // number base 16
        Serial.printlnf("Checksum: %02x, %03d",checksum,checksum);
        for(uint8_t i = 4; i < strlen(command)-3;i++){
            if(i < 7) cmdStr[i-4] = command[i];
            else dataStr[i-7] = command[i];
        }
        if(checksum != strlen(command)-3){
            Serial.printlnf("String Len: %d, Checksum: %d",strlen(command)-3,checksum);
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
            //command[1];
        }
        else if(!strcmp(cmdStr,"nak")){  //Acknowledgement for XBee and BLE
            strncpy(errCmdStr,dataStr,3);
            errCmdMode = mode;
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
        else if(!strcmp(cmdStr,"ccs")){  //Incoming communication status
            
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

void BLEScan(int BotNumber){
    size_t count = BLE.scan(scanResults, SCAN_RESULT_COUNT);
	if (count > 0) {
		for (uint8_t ii = 0; ii < count; ii++) {
			BleUuid foundServiceUuid;
			size_t svcCount = scanResults[ii].advertisingData.serviceUUID(&foundServiceUuid, 1);
            uint8_t BLECustomData[CUSTOM_DATA_LEN];
            scanResults->advertisingData.customData(BLECustomData,CUSTOM_DATA_LEN);
            if (svcCount > 0 && foundServiceUuid == serviceUuid) {
                if(BotNumber == -1 || BotNumber == BLECustomData[0]){   //Check if a particular bot number was specified
				    peer = BLE.connect(scanResults[ii].address);
				    if (peer.connected()) {
                        uint8_t bufName[BLE_MAX_ADV_DATA_LEN];
                        scanResults[ii].advertisingData.customData(bufName, BLE_MAX_ADV_DATA_LEN);
					    peer.getCharacteristicByUUID(peerTxCharacteristic, txUuid);
					    peer.getCharacteristicByUUID(peerRxCharacteristic, rxUuid);
                        peer.getCharacteristicByUUID(peerOffloadCharacteristic, offldUuid);
                        if(bufName){
						    Serial.printlnf("Connected to Bot %d",bufName[0]);
                            bool newBot = true;
                            for(WaterBot w: WaterBots){
                                if(bufName[0] == w.botNum) newBot = false;
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
        for(int i = 0 ; i < data.length(); i++) buffer[i] = data.charAt(i);
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
    char btBuf[len];
    for (size_t ii = 0; ii < len; ii++) btBuf[ii] = data[ii];
    Serial.println("New BT Command:");
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
